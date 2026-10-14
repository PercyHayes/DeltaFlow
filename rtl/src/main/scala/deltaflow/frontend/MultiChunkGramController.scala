package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._

class MultiGramRow(side: Int) extends Bundle {
  val job = UInt(6.W)
  val row = UInt(log2Ceil(side).W)
  val values = Vec(side, UInt(32.W))
}
class MultiGramWave(side: Int) extends Bundle {
  val job = UInt(6.W)
  val feature = UInt(7.W)
  val west = Vec(side, UInt(16.W))
  val north = Vec(side, UInt(16.W))
}

/** Three state-independent chunks fill the existing 64 accumulator contexts. Independent two-word
  * buffers per chunk share each physical SRAM read port. Registered per-panel issue queues cut
  * selection from the arithmetic path. A batch drains completely before its local chunk/feature
  * tags may be reused.
  */
class MultiChunkGramController(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val chunks = 3
  val jobsCount = 60
  val words = (side + 7) / 8
  val activeBanks = 8 * words
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W)))
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(60.W))
    val bankRead = Vec(16, Decoupled(UInt(9.W)))
    val bankResponse = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val in = Vec(16, Decoupled(new RetimedPanelInput(side, 2)))
    val clear = Vec(16, Decoupled(UInt(2.W)))
    val read = Vec(16, Decoupled(new RetimedRow(side)))
    val response = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val contextIdle = Input(Vec(16, Vec(4, Bool())))
    val result = Vec(4, Decoupled(new MultiGramRow(side)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(Vec(16, Valid(UInt(6.W))))
    val feature = Output(Vec(16, UInt(7.W)))
    val reads = Output(UInt(16.W))
    val returns = Output(UInt(16.W))
    val captured = Output(UInt(60.W))
    val occupancy = Output(Vec(16, UInt(3.W)))
  })
  val active = RegInit(false.B)
  val limit = Reg(UInt(8.W))
  val cleared = RegInit(VecInit(Seq.fill(jobsCount)(false.B)))
  val feature = RegInit(VecInit(Seq.fill(jobsCount)(0.U(8.W))))
  val launched = RegInit(VecInit(Seq.fill(jobsCount)(0.U(8.W))))
  val captured = RegInit(VecInit(Seq.fill(jobsCount)(0.U((2 * words).W))))
  val rows = RegInit(VecInit(Seq.fill(jobsCount)(0.U(log2Ceil(side + 1).W))))
  val frame = Seq.fill(jobsCount)(Reg(Vec(2 * words, UInt(128.W))))
  val stored = RegInit(0.U(log2Ceil(jobsCount * side + 1).W))
  val empty = Wire(Vec(16, Bool()))
  val captureBits = Seq.fill(jobsCount)(collection.mutable.ArrayBuffer[(Int, Bool)]())
  val accepted = Wire(Vec(jobsCount, Bool()))
  accepted.foreach(_ := false.B)
  io.busy := active
  io.done := false.B
  io.start.ready := !active && empty.asUInt.andR
  when(io.start.fire) {
    assert(io.start.bits >= 1.U && io.start.bits <= 128.U)
    active := true.B
    limit := io.start.bits
    stored := 0.U
    for (j <- 0 until jobsCount) {
      cleared(j) := false.B
      feature(j) := 0.U
      launched(j) := 0.U
      captured(j) := 0.U
      rows(j) := 0.U
    }
  }
  for (b <- 0 until 16) {
    io.bankRead(b).valid := false.B
    io.bankRead(b).bits := 0.U
    io.bankResponse(b).ready := false.B
    empty(b) := true.B
    io.occupancy(b) := 0.U
    if (b < activeBanks) {
      val members = (0 until 20).filter(j => SharedGramLayout.words(side, j).contains(b))
      val buffers = Seq.fill(chunks)(Module(new GramOperandBuffer(members.size)))
      val next = RegInit(VecInit(Seq.fill(chunks)(0.U(8.W))))
      // Three entries cover the two-cycle return without a same-cycle credit
      // bypass; the physical SRAM still reserves its own two response slots.
      val tags = Module(new Queue(UInt(3.W), 3, pipe = false, flow = false))
      val arb = Module(new RRArbiter(UInt(9.W), chunks))
      for (ch <- 0 until chunks) {
        val buf = buffers(ch)
        val want = active && next(ch) < limit && io.bankEnable(b)
        arb.io.in(ch).valid := want && buf.io.allocate.ready
        arb.io.in(ch).bits := (ch * 128).U + next(ch)
        buf.io.allocate.valid := arb.io.in(ch).fire
        buf.io.allocate.bits := next(ch)(6, 0)
        when(arb.io.in(ch).fire) {
          assert(buf.io.allocate.fire)
          next(ch) := next(ch) + 1.U
        }
        when(io.start.fire) { next(ch) := 0.U }
        buf.io.returned.valid := io.bankResponse(b).fire && tags.io.deq.bits(2, 1) === ch.U
        buf.io.returned.bits.slot := tags.io.deq.bits(0)
        buf.io.returned.bits.data := io.bankResponse(b).bits
        for ((local, m) <- members.zipWithIndex) {
          val j = ch * 20 + local
          val index = SharedGramLayout.words(side, local).indexOf(b)
          buf.io.request(m).valid := active && feature(j) < limit && !captured(j)(index)
          buf.io.request(m).bits := feature(j)(6, 0)
          buf.io.response(m).ready := true.B
          val take = buf.io.response(m).fire
          when(take) { frame(j)(index) := buf.io.response(m).bits }
          captureBits(j) += ((index, take))
        }
      }
      io.bankRead(b).valid := arb.io.out.valid && tags.io.enq.ready
      io.bankRead(b).bits := arb.io.out.bits
      arb.io.out.ready := io.bankRead(b).ready && tags.io.enq.ready
      tags.io.enq.valid := io.bankRead(b).fire
      tags.io.enq.bits := Cat(
        arb.io.chosen,
        VecInit(buffers.map(_.io.allocatedSlot))(arb.io.chosen)
      )
      io.bankResponse(b).ready := tags.io.deq.valid
      tags.io.deq.ready := io.bankResponse(b).valid
      empty(b) := buffers.map(_.io.occupied === 0.U).reduce(_ && _) && !tags.io.deq.valid
      io.occupancy(b) := buffers.map(_.io.occupied.pad(3)).reduce(_ + _)
    }
  }
  io.reads := VecInit(io.bankRead.map(_.fire)).asUInt
  io.returns := VecInit(io.bankResponse.map(_.fire)).asUInt
  val captures = Wire(Vec(jobsCount, Bool()))
  for (j <- 0 until jobsCount) {
    val mask =
      captureBits(j).map { case (i, v) => Mux(v, (1 << i).U((2 * words).W), 0.U) }.reduce(_ | _)
    captures(j) := mask.orR
    when(active) { captured(j) := captured(j) | mask }
    when(accepted(j)) {
      captured(j) := 0.U
      feature(j) := feature(j) + 1.U
    }
  }
  io.captured := captures.asUInt
  val resultArb = Seq.fill(4)(Module(new RRArbiter(new MultiGramRow(side), 4)))
  for (p <- 0 until 4) {
    val output = Module(new Queue(new MultiGramRow(side), 2, pipe = false, flow = false))
    io.result(p) <> output.io.deq
    output.io.enq <> resultArb(p).io.out
  }
  for (t <- 0 until 16) {
    val jobs = (0 until jobsCount).filter(_ % 16 == t)
    val rr = RegInit(0.U(2.W))
    val choices = Wire(Vec(4, Bool()))
    val order = Wire(Vec(4, UInt(2.W)))
    val ready = VecInit(
      (0 until 4).map(i =>
        if (i < jobs.size) {
          val j = jobs(i)
          active && cleared(j) && feature(j) < limit &&
          captured(j) ===
            ((1 <<
              SharedGramLayout
                .words(side, j % 20)
                .size) - 1).U && io.jobEnable(j)
        } else false.B
      )
    )
    for (i <- 0 until 4) {
      order(i) := rr + i.U
      choices(i) := ready(order(i))
    }
    val selected = order(PriorityEncoder(choices.asUInt))
    val choose = (0 until jobs.size).map(i => selected === i.U)
    val issue = Module(new Queue(new MultiGramWave(side), 2, pipe = false, flow = false))
    issue.io.enq.valid := choices.asUInt.orR
    issue.io.enq.bits.job := Mux1H(choose, jobs.map(_.U(6.W)))
    issue.io.enq.bits.feature := Mux1H(choose, jobs.map(j => feature(j)(6, 0)))
    for (k <- 0 until side) {
      val west = jobs.map { j =>
        val (kind, i, r) = SharedGramLayout.jobs(j % 20)
        val b = (if (kind == 0) i else r) * words + k / 8
        frame(j)(SharedGramLayout.words(side, j % 20).indexOf(b))(16 * (k % 8) + 15, 16 * (k % 8))
      }
      val north = jobs.map { j =>
        val (kind, i, r) = SharedGramLayout.jobs(j % 20)
        val b = (if (kind == 0) 0 else 4 * words) + (if (kind == 0) r else i) * words + k / 8
        frame(j)(SharedGramLayout.words(side, j % 20).indexOf(b))(16 * (k % 8) + 15, 16 * (k % 8))
      }
      issue.io.enq.bits.west(k) := Mux1H(choose, west)
      issue.io.enq.bits.north(k) := Mux1H(choose, north)
    }
    when(issue.io.enq.fire) { rr := selected + 1.U }
    for ((j, i) <- jobs.zipWithIndex) accepted(j) := issue.io.enq.fire && choose(i)
    io.in(t).valid := issue.io.deq.valid
    issue.io.deq.ready := io.in(t).ready
    io.in(t).bits.os := true.B
    io.in(t).bits.context := issue.io.deq.bits.job(5, 4)
    io.in(t).bits.tag := issue.io.deq.bits.job(5, 4)
    io.in(t).bits.north := issue.io.deq.bits.north
    for (k <- 0 until side) io.in(t).bits.west(k) := issue.io.deq.bits.west(k)
    when(io.in(t).fire) { launched(issue.io.deq.bits.job) := launched(issue.io.deq.bits.job) + 1.U }
    io.issued(t).valid := io.in(t).fire
    io.issued(t).bits := issue.io.deq.bits.job
    io.feature(t) := issue.io.deq.bits.feature
    val clearChoice = PriorityEncoderOH(VecInit(jobs.map(j => !cleared(j))).asUInt)
    io.clear(t).valid := active && clearChoice.orR
    io.clear(t).bits := Mux1H(clearChoice, jobs.map(j => (j / 16).U(2.W)))
    for ((j, i) <- jobs.zipWithIndex) when(io.clear(t).fire && clearChoice(i)) {
      cleared(j) := true.B
    }
    val readChoice = PriorityEncoderOH(
      VecInit(
        jobs.map(j => launched(j) === limit && rows(j) < side.U && io.contextIdle(t)(j / 16))
      ).asUInt
    )
    io.read(t).valid := active && readChoice.orR
    io.read(t).bits.context := Mux1H(readChoice, jobs.map(j => (j / 16).U(2.W)))
    io.read(t).bits.row := Mux1H(readChoice, jobs.map(j => rows(j)))
    for ((j, i) <- jobs.zipWithIndex) when(io.read(t).fire && readChoice(i)) {
      rows(j) := rows(j) + 1.U
    }
    resultArb(t % 4).io.in(t / 4).valid := io.response(t).valid
    resultArb(t % 4).io.in(t / 4).bits.job := Cat(io.response(t).bits.context, t.U(4.W))
    resultArb(t % 4).io.in(t / 4).bits.row := io.response(t).bits.row
    resultArb(t % 4).io.in(t / 4).bits.values := io.response(t).bits.values
    io.response(t).ready := resultArb(t % 4).io.in(t / 4).ready
  }
  val commits = PopCount(VecInit(io.result.map(_.fire)))
  when(commits =/= 0.U) {
    stored := stored + commits
    when(stored +& commits === (jobsCount * side).U) {
      active := false.B
      io.done := true.B
      assert(empty.asUInt.andR)
    }
  }
}
