package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._

/** Static placement matches the twenty-job behavioral schedule. QK is stored transposed (west K_j,
  * north Q_i); its downstream consumer must honor this.
  */
object SharedGramLayout {
  val jobs = for (kind <- 0 until 2; i <- 0 until 4; j <- 0 to i) yield (kind, i, j)
  def words(side: Int, job: Int): Seq[Int] = {
    val (kind, i, j) = jobs(job)
    val n = (side + 7) / 8
    val west = if (kind == 0) i else j
    val north = if (kind == 0) j else i
    ((0 until n).map(w => west * n + w) ++ (0 until n).map(w =>
      (if (kind == 0) 0 else 4 * n) + north * n + w
    )).distinct
  }
}
class GramRow(val side: Int) extends Bundle {
  val job = UInt(5.W)
  val row = UInt(log2Ceil(side).W)
  val values = Vec(side, UInt(32.W))
}

/** Synthesizable bounded Gram scheduler. This owns no MAC or SRAM: its ports connect to the
  * existing physical panels/banks. One registered frame per job, two reserved words per bank,
  * ordered feature reduction, contexts 1 and 3.
  */
class SharedGramController(
    val side: Int = 16,
    val diagonalFirst: Boolean = false,
    val partitioned: Boolean = false,
    val retiredStart: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  val words = (side + 7) / 8
  val activeBanks = 8 * words
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W))) // 1..128 features
    // 0: all jobs, 1: first sixteen (C1 only), 2: remaining four (C3).
    val partition = if (partitioned) Some(Input(UInt(2.W))) else None
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(20.W))
    val bankRead = Vec(16, Decoupled(UInt(9.W)))
    val bankResponse = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val in = Vec(16, Decoupled(new RetimedPanelInput(side, 2)))
    val clear = Vec(16, Decoupled(UInt(2.W)))
    val read = Vec(16, Decoupled(new RetimedRow(side)))
    val response = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val contextIdle = Input(Vec(16, Vec(4, Bool())))
    val result = Vec(4, Decoupled(new GramRow(side)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(Vec(16, Valid(UInt(5.W))))
    val feature = Output(Vec(16, UInt(7.W)))
    val captured = Output(UInt(20.W))
    val allocated = Output(UInt(16.W))
    val bufferOccupancy = Output(Vec(16, UInt(2.W)))
  })
  val active = RegInit(false.B)
  val lastFeature = Reg(UInt(8.W))
  // Decode only at command acceptance. In particular, expectedRows must not
  // decode a global partition on the result-fire -> completion -> ready path.
  val includedGroups = if (partitioned) Some(RegInit(VecInit(Seq.fill(2)(true.B)))) else None
  val rowLimit = if (partitioned) Some(RegInit((20 * side).U(log2Ceil(20 * side + 1).W))) else None
  def included(j: Int): Bool = includedGroups.map(_(if (j < 16) 0 else 1)).getOrElse(true.B)
  val expectedRows = rowLimit.getOrElse((20 * side).U)
  // Register terminal predicates on the accepting edge. Comparing the common
  // limit in request/arbitration logic otherwise feeds a long shared-ready cone.
  // These flags become visible on exactly the edge feature/next reach the limit.
  val featureDone = RegInit(VecInit(Seq.fill(20)(false.B)))
  val bankDone = RegInit(VecInit(Seq.fill(activeBanks)(false.B)))
  val cleared = RegInit(VecInit(Seq.fill(20)(false.B)))
  val feature = RegInit(VecInit(Seq.fill(20)(0.U(8.W))))
  val captured = RegInit(VecInit(Seq.fill(20)(0.U((2 * words).W))))
  val rows = RegInit(VecInit(Seq.fill(20)(0.U(log2Ceil(side + 1).W))))
  val frame = Seq.fill(20)(Reg(Vec(2 * words, UInt(128.W))))
  val stored = RegInit(0.U(log2Ceil(20 * side + 1).W))
  val next = RegInit(VecInit(Seq.fill(activeBanks)(0.U(8.W))))
  val capture = Wire(Vec(20, UInt((2 * words).W)))
  val accepted = Wire(Vec(20, Bool()))
  for (j <- 0 until 20) {
    capture(j) := 0.U
    accepted(j) := false.B
  }
  val empty = Wire(Vec(16, Bool()))
  // Completion already proves every reserved word and response has retired.
  // Keep that invariant executable instead of repeating its wide reduction on
  // the shared command/operand-selection path of the column candidate.
  io.start.ready := !active && (if (retiredStart) true.B else empty.asUInt.andR)
  if (retiredStart) when(!active) {
    assert(empty.asUInt.andR, "idle Gram must have retired every buffer and tag")
  }
  io.busy := active
  io.done := false.B
  when(io.start.fire) {
    assert(io.start.bits >= 1.U && io.start.bits <= 128.U)
    active := true.B
    lastFeature := io.start.bits - 1.U
    stored := 0.U
    if (partitioned) {
      val p = io.partition.get
      assert(p <= 2.U)
      includedGroups.get(0) := p =/= 2.U
      includedGroups.get(1) := p =/= 1.U
      rowLimit.get := Mux(p === 1.U, (16 * side).U, Mux(p === 2.U, (4 * side).U, (20 * side).U))
    }
    for (j <- 0 until 20) {
      cleared(j) := false.B
      feature(j) := 0.U
      featureDone(j) := false.B
      captured(j) := 0.U
      rows(j) := 0.U
    }
    for (b <- 0 until activeBanks) {
      next(b) := 0.U
      bankDone(b) := false.B
    }
  }
  val captureBits = Seq.fill(20)(collection.mutable.ArrayBuffer[(Int, Bool)]())
  val allocations = Wire(Vec(16, Bool()))
  for (b <- 0 until 16) {
    allocations(b) := false.B
    io.bufferOccupancy(b) := 0.U
    empty(b) := true.B
    io.bankRead(b).valid := false.B
    io.bankRead(b).bits := 0.U
    io.bankResponse(b).ready := false.B
    if (b < activeBanks) {
      val members = (0 until 20).filter(j => SharedGramLayout.words(side, j).contains(b))
      val buffer = Module(new GramOperandBuffer(members.size))
      val tags = Module(new Queue(Bool(), 2, pipe = false, flow = false))
      val want = active && !bankDone(b) && io.bankEnable(b)
      io.bankRead(b).valid := want && buffer.io.allocate.ready && tags.io.enq.ready
      io.bankRead(b).bits := next(b)
      buffer.io.allocate.valid := want && io.bankRead(b).ready && tags.io.enq.ready
      buffer.io.allocate.bits := next(b)(6, 0)
      tags.io.enq.valid := io.bankRead(b).fire
      tags.io.enq.bits := buffer.io.allocatedSlot
      when(io.bankRead(b).fire) {
        assert(buffer.io.allocate.fire)
        next(b) := next(b) + 1.U
        when(next(b) === lastFeature) { bankDone(b) := true.B }
      }
      buffer.io.returned.valid := io.bankResponse(b).valid && tags.io.deq.valid
      buffer.io.returned.bits.slot := tags.io.deq.bits
      buffer.io.returned.bits.data := io.bankResponse(b).bits
      tags.io.deq.ready := io.bankResponse(b).valid
      io.bankResponse(b).ready := tags.io.deq.valid
      for ((j, m) <- members.zipWithIndex) {
        val index = SharedGramLayout.words(side, j).indexOf(b)
        buffer.io.request(m).valid := active && !featureDone(j) && !captured(j)(index)
        buffer.io.request(m).bits := feature(j)(6, 0)
        buffer.io.response(m).ready := true.B
        val take = buffer.io.response(m).fire
        when(take) { frame(j)(index) := buffer.io.response(m).bits }
        captureBits(j) += ((index, take))
      }
      allocations(b) := io.bankRead(b).fire
      io.bufferOccupancy(b) := buffer.io.occupied
      empty(b) := buffer.io.occupied === 0.U && !tags.io.deq.valid
    }
  }
  io.allocated := allocations.asUInt
  for (j <- 0 until 20) {
    capture(j) := captureBits(j)
      .map { case (i, v) => Mux(v, (1 << i).U((2 * words).W), 0.U) }
      .reduce(_ | _)
    // Non-executed consumers still retire their shared-word reservations. They
    // never clear/read a C32 or issue arithmetic, and cannot strand the buffer.
    if (partitioned)
      when(
        active && !included(j) && !featureDone(j) &&
          captured(j) === ((1 << SharedGramLayout.words(side, j).size) - 1).U
      ) { accepted(j) := true.B }
    when(active) { captured(j) := captured(j) | capture(j) }
    when(accepted(j)) {
      captured(j) := 0.U
      feature(j) := feature(j) + 1.U
      when(feature(j) === lastFeature) { featureDone(j) := true.B }
    }
  }
  io.captured := VecInit(capture.map(_.orR)).asUInt
  val resultArb = Seq.fill(4)(Module(new RRArbiter(new GramRow(side), 4)))
  val diagonal = VecInit(
    (0 until 16).map(t =>
      (Set(0, 2, 5, 9).contains(t)).B &&
        (if (t < 4) io.response(t).bits.context === 1.U
         else true.B)
    )
  )
  val urgent = VecInit(
    (0 until 4).map(p =>
      (0 until 4).map(i => io.response(4 * i + p).valid && diagonal(4 * i + p)).reduce(_ || _)
    )
  )
  for (p <- 0 until 4) {
    val output = Module(new Queue(new GramRow(side), 2, pipe = false, flow = false))
    io.result(p) <> output.io.deq
    output.io.enq <> resultArb(p).io.out
  }
  for (t <- 0 until 16) {
    val jobs = (0 until 20).filter(_ % 16 == t)
    val rr = RegInit(false.B)
    val readyJobs = jobs.map(j =>
      active && included(j) && cleared(j) && !featureDone(j) &&
        captured(j) === ((1 << SharedGramLayout.words(side, j).size) - 1).U && io.jobEnable(j)
    )
    val locked = RegInit(false.B)
    val held = Reg(Bool())
    val priority =
      if (jobs.size == 1) 0.U(1.W) else Mux(readyJobs(1) && (!readyJobs(0) || rr), 1.U, 0.U)
    val selected = Mux(locked, held.asUInt, priority)
    when(io.in(t).valid && !io.in(t).ready) {
      locked := true.B
      held := selected.asBool
    }
    when(io.in(t).fire) { locked := false.B }
    val choose = Wire(Vec(jobs.size, Bool()))
    for (i <- jobs.indices) choose(i) := selected === i.U
    io.in(t).valid := locked || readyJobs.reduce(_ || _)
    io.in(t).bits.os := true.B
    io.in(t).bits.context := Mux1H(choose, jobs.map(j => (if (j < 16) 1 else 3).U(2.W)))
    io.in(t).bits.tag := io.in(t).bits.context
    for (k <- 0 until side) {
      val north = jobs.map { j =>
        val (kind, i, r) = SharedGramLayout.jobs(j)
        val group = if (kind == 0) r else i
        val b = (if (kind == 0) 0 else 4 * words) + group * words + k / 8
        frame(j)(SharedGramLayout.words(side, j).indexOf(b))(16 * (k % 8) + 15, 16 * (k % 8))
      }
      val west = jobs.map { j =>
        val (kind, i, r) = SharedGramLayout.jobs(j)
        val b = (if (kind == 0) i else r) * words + k / 8
        frame(j)(SharedGramLayout.words(side, j).indexOf(b))(16 * (k % 8) + 15, 16 * (k % 8))
      }
      io.in(t).bits.north(k) := Mux1H(choose, north)
      io.in(t).bits.west(k) := Mux1H(choose, west)
    }
    for ((j, i) <- jobs.zipWithIndex) when(io.in(t).fire && choose(i)) { accepted(j) := true.B }
    when(io.in(t).fire) { rr := !selected.asBool }
    io.issued(t).valid := io.in(t).fire
    io.issued(t).bits := Mux1H(choose, jobs.map(_.U(5.W)))
    io.feature(t) := Mux1H(choose, jobs.map(j => feature(j)(6, 0)))
    val clearChoice = PriorityEncoderOH(VecInit(jobs.map(j => included(j) && !cleared(j))).asUInt)
    io.clear(t).valid := active && clearChoice.orR
    io.clear(t).bits := Mux1H(clearChoice, jobs.map(j => (if (j < 16) 1 else 3).U(2.W)))
    for ((j, i) <- jobs.zipWithIndex) when(io.clear(t).fire && clearChoice(i)) {
      cleared(j) := true.B
    }
    val readChoice = PriorityEncoderOH(
      VecInit(
        jobs.map(j =>
          included(j) && featureDone(j) && rows(j) < side.U && io.contextIdle(t)(
            if (j < 16) 1 else 3
          )
        )
      ).asUInt
    )
    io.read(t).valid := active && readChoice.orR
    io.read(t).bits.context := Mux1H(readChoice, jobs.map(j => (if (j < 16) 1 else 3).U(2.W)))
    io.read(t).bits.row := Mux1H(readChoice, jobs.map(j => rows(j)))
    for ((j, i) <- jobs.zipWithIndex) when(io.read(t).fire && readChoice(i)) {
      rows(j) := rows(j) + 1.U
    }
    val publishAllowed = if (diagonalFirst) !urgent(t % 4) || diagonal(t) else true.B
    resultArb(t % 4).io.in(t / 4).valid := io.response(t).valid && publishAllowed
    resultArb(t % 4).io
      .in(t / 4)
      .bits
      .job := (if (t < 4) Mux(io.response(t).bits.context === 3.U, (t + 16).U, t.U) else t.U)
    resultArb(t % 4).io.in(t / 4).bits.row := io.response(t).bits.row
    resultArb(t % 4).io.in(t / 4).bits.values := io.response(t).bits.values
    io.response(t).ready := resultArb(t % 4).io.in(t / 4).ready && publishAllowed
  }
  val commits = PopCount(VecInit(io.result.map(_.fire)))
  when(commits =/= 0.U) {
    stored := stored + commits
    when(stored +& commits === expectedRows) {
      active := false.B
      io.done := true.B
      assert(empty.asUInt.andR)
    }
  }
}
