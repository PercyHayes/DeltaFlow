package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.arithmetic.FloatArithmetic

class TransformCommand extends Bundle {
  val keyColumns = UInt(8.W)
  val valueColumns = UInt(8.W)
}
class TransformCoefficient(side: Int) extends Bundle {
  val job = UInt(4.W)
  val coefficient = new WaveCoefficient(side)
}

/** Ten dependent CS jobs use ten of the existing sixteen physical panels. No MAC, extra PE
  * accumulator, SRAM, or floating point multiplier here. Completed diagonal vectors drive
  * successors through one-read publishers; off-diagonal residuals retain FP32 and use the same
  * reservation protocol.
  */
class ReadyTransformController(
    val side: Int = 16,
    val returnFlow: Boolean = false,
    val eagerMulticast: Boolean = false,
    val registeredDrain: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(!eagerMulticast || returnFlow)
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(9.W)))
    val enable = Input(Bool())
    val coefficient = Flipped(Vec(4, Decoupled(new GramRow(side))))
    val matrix = Decoupled(new GramRow(side)) // transposed causal QK, retained FP32
    val load = Vec(10, Decoupled(new WaveCoefficient(side)))
    val rhs = Flipped(Vec(4, Decoupled(new PublishedVector(side))))
    val input = Vec(10, Decoupled(new RetimedPanelInput(side, 2)))
    val returned = Flipped(Vec(10, Decoupled(new RetimedResult(side, 2))))
    val result = Vec(4, Decoupled(new PublishedVector(side))) // BF16 in low bits
    val loaded = Output(UInt(10.W))
    val busy = Output(Bool())
    val idle = Output(Bool())
    val reserved = Output(UInt(10.W))
    val captured = Output(Vec(10, UInt(4.W)))
    val fed = Output(UInt(10.W))
    val occupied = Output(Vec(10, UInt(7.W)))
  })
  val active = RegInit(false.B)
  val columns = Reg(UInt(9.W))
  val loaded = RegInit(VecInit(Seq.fill(10)(0.U(side.W))))
  val resultDepth = if (returnFlow) 16 else 65
  val pub = SharedGramLayout.jobs.take(10).map { case (_, i, j) =>
    Module(
      new CompletionPublisher(
        side,
        resultDepth,
        false,
        if (i == j) 15 else 0,
        onReturn = returnFlow,
        compactBF16 = returnFlow && i == j,
        eagerReturn = eagerMulticast
      )
    )
  }
  val issueKey = Reg(Vec(10, UInt(16.W)))
  val inputs =
    Seq.fill(10)(Module(new Queue(new RetimedPanelInput(side, 2), 2, pipe = false, flow = false)))
  val base = Reg(UInt(16.W))
  val emitted = RegInit(VecInit(Seq.fill(4)(0.U(9.W))))
  val count = RegInit(0.U(log2Ceil(20 * side + 1).W))
  val reservations = Wire(Vec(10, Bool()))
  val feeds = Wire(Vec(10, Bool()))
  // A registered raw row precedes shared conversion and routing. M keeps its
  // physical transpose; no implicit transpose network is hidden here.
  val arb = Module(new RRArbiter(new GramRow(side), 4))
  arb.io.in <> io.coefficient
  val raw = Module(new Queue(new GramRow(side), 2, pipe = false, flow = false))
  raw.io.enq <> arb.io.out
  val converted = Module(new Queue(new TransformCoefficient(side), 2, pipe = false, flow = false))
  val isMatrix = raw.io.deq.bits.job >= 10.U
  val diagonal = VecInit(SharedGramLayout.jobs.take(10).map { case (_, i, j) => (i == j).B })
  converted.io.enq.valid := raw.io.deq.valid && !isMatrix
  converted.io.enq.bits.job := raw.io.deq.bits.job
  converted.io.enq.bits.coefficient.column := false.B
  converted.io.enq.bits.coefficient.index := raw.io.deq.bits.row
  for (k <- 0 until side)
    converted.io.enq.bits.coefficient.values(k) := FloatArithmetic.narrow(
      raw.io.deq.bits
        .values(k) ^ Mux(diagonal(raw.io.deq.bits.job(3, 0)), 0.U(32.W), "h80000000".U(32.W))
    )
  io.matrix.valid := raw.io.deq.valid && isMatrix
  io.matrix.bits := raw.io.deq.bits
  raw.io.deq.ready := Mux(isMatrix, io.matrix.ready, converted.io.enq.ready)
  for (j <- 0 until 10) {
    io.load(j).valid := converted.io.deq.valid && converted.io.deq.bits.job === j.U
    io.load(j).bits := converted.io.deq.bits.coefficient
    when(io.load(j).fire) {
      val mask = (1.U(side.W) << io.load(j).bits.index)(side - 1, 0)
      assert(active && !(loaded(j) & mask).orR)
      loaded(j) := loaded(j) | mask
    }
  }
  converted.io.deq.ready := VecInit(io.load.map(_.ready))(converted.io.deq.bits.job)
  when(raw.io.deq.fire) { count := count + 1.U }
  for (i <- 0 until 4) {
    io.rhs(i).ready := false.B
    io.result(i).valid := false.B
    io.result(i).bits := 0.U.asTypeOf(io.result(i).bits)
  }
  val panelDrained = pub.map(_.io.idle).reduce(_ && _) &&
    inputs.map(_.io.count === 0.U).reduce(_ && _)
  val coefficientDrained = raw.io.count === 0.U && converted.io.count === 0.U
  val drained = panelDrained && coefficientDrained
  // Retirement observes the original current-cycle drain. On that same edge
  // this status captures true, so the next inactive cycle remains ready.
  // No publisher/input entry can be created while inactive: their producers
  // belong to this active epoch, which retires only after every column drains.
  // Coefficients retain their current-cycle empty checks, including admission
  // on the start edge. Only the expensive panel/FIFO reduction is registered.
  val panelStatus = if (registeredDrain) RegNext(panelDrained, true.B) else panelDrained
  val available = !active && panelStatus && coefficientDrained
  if (registeredDrain) {
    when(!active) {
      assert(panelStatus === panelDrained, "inactive transform retained a publication")
    }
    assert(available === (!active && drained), "transform readiness changed a cycle")
  }
  io.start.ready := available
  when(io.start.fire) {
    assert(io.start.bits > 0.U && io.start.bits <= 256.U)
    active := true.B
    columns := io.start.bits
    count := 0.U
    base := pub.head.io.ticket.key
    for (j <- 0 until 10) {
      loaded(j) := 0.U
      issueKey(j) := pub(j).io.ticket.key
      assert(pub(j).io.ticket.key === pub.head.io.ticket.key)
    }
    for (i <- 0 until 4) emitted(i) := 0.U
  }
  for (j <- 0 until 10) {
    val p = pub(j)
    val (_, i, k) = SharedGramLayout.jobs(j)
    val mask = if (i == k) (1 << (3 - i)) - 1 | 8 else 8
    val q = inputs(j)
    val admission = if (returnFlow) true.B else p.io.reserve.ready
    io.input(j).valid := active && io.enable && loaded(j).andR && q.io.deq.valid && admission
    io.input(j).bits := q.io.deq.bits
    q.io.deq.ready := active && io.enable && loaded(j).andR && io.input(j).ready && admission
    p.io.reserve.valid := (if (returnFlow) io.returned(j).valid else io.input(j).fire)
    p.io.reserve.bits := mask.U
    when(io.input(j).fire) {
      assert(io.input(j).bits.tag === issueKey(j)(1, 0))
      issueKey(j) := issueKey(j) + 1.U
    }
    p.io.returned.valid := io.returned(j).valid
    p.io.returned.bits := io.returned(j).bits.values
    io.returned(j).ready := p.io.returned.ready
    if (!returnFlow) when(io.returned(j).valid) { assert(p.io.returned.ready) }
    when(io.returned(j).valid) { assert(io.returned(j).bits.tag === p.io.expectedKey(1, 0)) }
    for (c <- 0 until 4) {
      val requested = RegInit(0.U(9.W))
      val slot = Reg(UInt(log2Ceil(resultDepth).W))
      val key = Reg(UInt(16.W))
      p.io.request(c).valid := active && ((mask & (1 << c)) != 0).B && requested < columns
      p.io.request(c).bits.slot := slot
      p.io.request(c).bits.key := key
      p.io.response(c).ready := false.B
      when(p.io.request(c).fire) {
        requested := requested + 1.U
        key := key + 1.U
        slot := Mux(slot === (resultDepth - 1).U, 0.U, slot + 1.U)
      }
      when(io.start.fire) {
        requested := 0.U
        slot := p.io.ticket.slot
        key := p.io.ticket.key
      }
    }
    // Join sources at the registered input queue. Source data is released only
    // when this queue captures the complete vector and its full epoch key.
    val west = if (k == 0) io.rhs(i) else pub(j - 1).io.response(3)
    val westKey = if (k == 0) (base + west.bits.key)(15, 0) else west.bits.key
    val north = if (i != k) Some(pub(k * (k + 1) / 2 + k).io.response(i - k - 1)) else None
    val both = west.valid && north.map(_.valid).getOrElse(true.B)
    q.io.enq.valid := active && both
    q.io.enq.bits.os := false.B
    q.io.enq.bits.context := 0.U
    q.io.enq.bits.tag := westKey(1, 0)
    for (r <- 0 until side) {
      q.io.enq.bits.north(r) :=
        north
          .map(_.bits.values(r)(15, 0))
          .getOrElse(FloatArithmetic.narrow(west.bits.values(r)))
      q.io.enq.bits.west(r) := (if (i == k) 0.U else west.bits.values(r))
    }
    west.ready := active && q.io.enq.ready && north.map(_.valid).getOrElse(true.B)
    north.foreach { n =>
      n.ready := active && q.io.enq.ready && west.valid
      when(q.io.enq.fire) {
        assert(n.bits.key === westKey, "mismatched RHS/residual and solved panel")
      }
    }
    if (i == k) {
      io.result(i).valid := p.io.response(3).valid
      io.result(i).bits := p.io.response(3).bits
      io.result(i).bits.key := p.io.response(3).bits.key - base
      p.io.response(3).ready := io.result(i).ready
      when(io.result(i).fire) {
        assert(io.result(i).bits.key === emitted(i))
        emitted(i) := emitted(i) + 1.U
      }
    }
    reservations(j) := io.input(j).fire
    io.captured(j) := p.io.captured
    io.occupied(j) := p.io.occupied
    feeds(j) := q.io.enq.fire
  }
  when(active && count === (20 * side).U && emitted.map(_ === columns).reduce(_ && _) && drained) {
    active := false.B
  }
  io.loaded := VecInit(loaded.map(_.andR)).asUInt
  io.busy := active
  io.idle := available
  io.reserved := reservations.asUInt
  io.fed := feeds.asUInt
}
