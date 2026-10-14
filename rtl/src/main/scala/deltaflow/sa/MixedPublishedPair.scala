package deltaflow.sa
import chisel3._
import chisel3.util._

/** Prepared-vector boundary for coupled U/W CS and independent OS work. Two existing four-C32
  * arrays, one 65-slot publisher. No SRAM/HBM instantiated. The new queues and control are explicit
  * resources, never free memory ports.
  */
class MixedPublishedPair(val side: Int = 16, val cutResponseReady: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val cs = Flipped(Decoupled(new RetimedPanelInput(side, 2)))
    val os = Flipped(Vec(2, Decoupled(new RetimedPanelInput(side, 2))))
    val rhs = Flipped(Decoupled(new PublishedVector(side)))
    val coefficient = Flipped(Vec(2, Decoupled(new WaveCoefficient(side))))
    val seed = Flipped(Vec(2, Decoupled(new RetimedRowData(side))))
    val read = Flipped(Vec(2, Decoupled(new RetimedRow(side))))
    val response = Vec(2, Decoupled(new RetimedRowData(side)))
    val request = Flipped(Vec(4, Decoupled(new CompletionTicket(65))))
    val observer = Vec(3, Decoupled(new PublishedVector(side)))
    val result = Decoupled(new RetimedResult(side, 2))
    val osEnable = Input(Bool()) // false gives the same-hardware stage-barrier control
    val idle = Output(Bool())
    val occupied = Output(UInt(7.W))
    val issue = Output(Vec(2, Valid(new RetimedPanelInput(side, 2))))
    val returned = Output(Bool())
    val captured = Output(UInt(4.W))
    val advance = Output(Vec(2, Bool()))
  })
  val panels = Seq.fill(2)(Module(new RetimedWavePanel(side)))
  val arbiters = Seq.fill(2)(Module(new ReadyWaveIssue(side)))
  val csq = Module(new Queue(new RetimedPanelInput(side, 2), 2, pipe = true, flow = false))
  val osq =
    Seq.fill(2)(Module(new Queue(new RetimedPanelInput(side, 2), 2, pipe = true, flow = false)))
  val rhsq = Module(new Queue(new PublishedVector(side), 2, pipe = true, flow = false))
  val pub = Module(new CompletionPublisher(side, 65, reuseConsumedCredit = !cutResponseReady))
  // In the timing-cut integration, configuration waits for a registered idle
  // permit. Acceptance clears it on the same edge, so a stale idle observation
  // can never permit coefficients/seed to race with newly accepted operands.
  val controlPermit = if (cutResponseReady) {
    val permit = RegInit(true.B)
    when(io.cs.fire || io.rhs.fire || io.os.map(_.fire).reduce(_ || _)) { permit := false.B }
      .elsewhen(
        pub.io.idle &&
          panels
            .map(_.io.idle)
            .reduce(_ && _) && csq.io.count === 0.U && rhsq.io.count === 0.U &&
          osq
            .map(_.io.count === 0.U)
            .reduce(_ && _)
      ) { permit := true.B }
    permit
  } else true.B
  csq.io.enq <> io.cs
  rhsq.io.enq <> io.rhs
  for (i <- 0 until 2) {
    osq(i).io.enq <> io.os(i)
    arbiters(i).io.os.valid := osq(i).io.deq.valid && io.osEnable
    arbiters(i).io.os.bits := osq(i).io.deq.bits
    osq(i).io.deq.ready := arbiters(i).io.os.ready && io.osEnable
    panels(i).io.in <> arbiters(i).io.out
    arbiters(i).io.advance := panels(i).io.advance
    // The caller may initialize before admission and read after idle. Guard
    // accepted queued work, rather than only the PE's current in-flight waves.
    val controlsSafe = (if (cutResponseReady) controlPermit && !io.rhs.valid
                        else
                          csq.io.count === 0.U && rhsq.io.count === 0.U &&
                          osq
                            .map(_.io.count === 0.U)
                            .reduce(_ && _)) && !io.cs.valid && !io.os.map(_.valid).reduce(_ || _)
    val coefficientSafe =
      controlsSafe && (if (cutResponseReady) true.B else pub.io.idle && !io.rhs.valid)
    panels(i).io.coefficient.valid := io.coefficient(i).valid && coefficientSafe
    panels(i).io.coefficient.bits := io.coefficient(i).bits
    io.coefficient(i).ready := panels(i).io.coefficient.ready && coefficientSafe
    panels(i).io.seed.valid := io.seed(i).valid && controlsSafe
    panels(i).io.seed.bits := io.seed(i).bits
    io.seed(i).ready := panels(i).io.seed.ready && controlsSafe
    panels(i).io.clear.valid := false.B
    panels(i).io.clear.bits := 0.U
    panels(i).io.read.valid := io.read(i).valid && controlsSafe
    panels(i).io.read.bits := io.read(i).bits
    io.read(i).ready := panels(i).io.read.ready && controlsSafe
    io.response(i) <> panels(i).io.response
    io.issue(i).valid := panels(i).io.in.fire
    io.issue(i).bits := panels(i).io.in.bits
    io.advance(i) := panels(i).io.advance
  }
  arbiters(0).io.cs <> csq.io.deq
  arbiters(0).io.cs.bits.tag := pub.io.ticket.key(1, 0)
  arbiters(0).io.allowCS := pub.io.reserve.ready
  pub.io.reserve.valid := panels(0).io.in.fire && !panels(0).io.in.bits.os
  pub.io.reserve.bits := 15.U
  pub.io.returned.valid := panels(0).io.out.valid
  pub.io.returned.bits := panels(0).io.out.bits.values
  panels(0).io.out.ready := pub.io.returned.ready
  when(panels(0).io.out.valid) {
    assert(pub.io.returned.ready)
    assert(panels(0).io.out.bits.tag === pub.io.expectedKey(1, 0))
  }
  pub.io.request <> io.request
  val matched = pub.io
    .response(0)
    .valid && rhsq.io.deq.valid && pub.io.response(0).bits.key === rhsq.io.deq.bits.key
  arbiters(1).io.cs.valid := matched
  arbiters(1).io.cs.bits.os := false.B
  arbiters(1).io.cs.bits.context := 0.U
  arbiters(1).io.cs.bits.tag := pub.io.response(0).bits.key(1, 0)
  arbiters(1).io.cs.bits.north := VecInit(pub.io.response(0).bits.values.map(_(15, 0)))
  arbiters(1).io.cs.bits.west := rhsq.io.deq.bits.values
  pub.io.response(0).ready := matched && arbiters(1).io.cs.ready
  rhsq.io.deq.ready := matched && arbiters(1).io.cs.ready
  arbiters(1).io.allowCS := true.B
  for (i <- 0 until 3) io.observer(i) <> pub.io.response(i + 1)
  io.result <> panels(1).io.out
  io.returned := pub.io.returned.fire
  io.captured := pub.io.captured
  io.occupied := pub.io.occupied
  io.idle :=
    panels
      .map(_.io.idle)
      .reduce(_ && _) && pub.io.idle && csq.io.count === 0.U && rhsq.io.count === 0.U &&
      osq
        .map(_.io.count === 0.U)
        .reduce(_ && _)
}
