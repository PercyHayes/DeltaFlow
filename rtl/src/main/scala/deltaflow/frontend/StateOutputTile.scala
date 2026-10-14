package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.memory._
import deltaflow.arithmetic.FloatArithmetic

/** Standalone H/O verification shell, one 16-panel array, NOT an additional array inside
  * ReadyTransformTile. Input SRAM adapters / shared FP32 scaling are outside this shell. Only the
  * output is backed by real synchronous SRAM. H is FP32 at base 256 (row*2+valueHalf); O is BF16 at
  * base 0. Host reads are allowed only when idle; reading an unwritten word is illegal.
  */
class StateOutputTile(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val t = 4 * side
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new StateOutputCommand))
    val oldStateReleased = Input(Bool())
    val seed = Flipped(Decoupled(new StateOutputSeed(side)))
    val column = Flipped(Decoupled(new StateOutputColumn(side)))
    val panelEnable = Input(UInt(16.W))
    val bankEnable = Input(UInt(16.W))
    val hostRead = Flipped(Decoupled(UInt(14.W)))
    val hostResponse = Decoupled(UInt(128.W))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(UInt(16.W))
    val issueContext = Output(Vec(16, UInt(2.W)))
    val issueToken = Output(Vec(16, UInt(2.W)))
    val frames = Output(UInt(2.W))
    val released = Output(Valid(UInt(log2Ceil(t).W)))
    val committed = Output(Valid(UInt(log2Ceil(3 * t).W)))
    val writeFires = Output(UInt(16.W))
    val committedEpoch = Output(UInt(32.W))
    val committedHalf = Output(Bool())
  })
  val controller = Module(new StateOutputController(side))
  val panels =
    Seq.fill(16)(Module(new RetimedWavePanel(side, rowResponseDepth = 2, registerInput = true)))
  val memory = Module(new MacroScratchpad(16, 1024, 16))
  val hostPending = RegInit(false.B)
  val hostBank = Reg(UInt(4.W))
  val allowed = !hostPending && !io.hostRead.valid
  controller.io.start.valid := io.start.valid && allowed
  controller.io.start.bits := io.start.bits
  io.start.ready := controller.io.start.ready && allowed
  controller.io.oldStateReleased := io.oldStateReleased
  controller.io.seed <> io.seed
  controller.io.column <> io.column
  controller.io.panelEnable := io.panelEnable
  io.busy := controller.io.busy
  io.done := controller.io.done
  io.issued := controller.io.issued
  io.frames := controller.io.frames
  io.released := controller.io.released
  for (p <- 0 until 16) {
    val panel = panels(p)
    panel.io.in <> controller.io.input(p)
    panel.io.seed <> controller.io.panelSeed(p)
    panel.io.read <> controller.io.rowRead(p)
    controller.io.rowResponse(p) <> panel.io.response
    panel.io.coefficient.valid := false.B
    panel.io.coefficient.bits := 0.U.asTypeOf(panel.io.coefficient.bits)
    panel.io.clear.valid := false.B
    panel.io.clear.bits := 0.U
    panel.io.out.ready := true.B
    assert(!panel.io.out.valid)
    io.issueContext(p) := controller.io.input(p).bits.context
    io.issueToken(p) := controller.io.input(p).bits.tag
  }
  val result = controller.io.result
  val isO = result.bits.index >= (2 * t).U
  val stored = RegInit(0.U(16.W))
  val needed = Mux(isO, ((BigInt(1) << (t / 8)) - 1).U(16.W), ((BigInt(1) << (t / 4)) - 1).U(16.W))
  val narrow = Wire(Vec(t, UInt(16.W)))
  for (i <- 0 until t) narrow(i) := FloatArithmetic.narrow(result.bits.values(i))
  val fires = VecInit(memory.io.write.map(_.fire)).asUInt
  result.ready := ((stored | fires) & needed) === needed
  when(result.valid) { stored := stored | fires }
  when(result.fire) { stored := 0.U }
  io.committed.valid := result.fire
  io.committed.bits := result.bits.index
  io.writeFires := fires
  io.committedEpoch := result.bits.epoch
  io.committedHalf := result.bits.half
  for (b <- 0 until 16) {
    val w = memory.io.write(b)
    w.valid := result.valid && needed(b) && !stored(b) && io.bankEnable(b)
    w.bits.address := Mux(
      isO,
      (result.bits.index - (2 * t).U) * 2.U + result.bits.half,
      256.U + result.bits.index * 2.U + result.bits.half
    )
    val wide =
      if (b < t / 4) Cat((0 until 4).reverse.map(i => result.bits.values(b * 4 + i)))
      else 0.U(128.W)
    val small = if (b < t / 8) Cat((0 until 8).reverse.map(i => narrow(b * 8 + i))) else 0.U(128.W)
    w.bits.data := Mux(isO, small, wide)
    w.bits.mask := 65535.U
    memory.io
      .read(b)
      .valid := io.hostRead.valid && !controller.io.busy && !io.start.valid && !hostPending &&
      io.hostRead
        .bits(3, 0) === b.U
    memory.io.read(b).bits := io.hostRead.bits(13, 4)
    memory.io.response(b).ready := hostPending && hostBank === b.U && io.hostResponse.ready
  }
  io.hostRead.ready := !controller.io.busy && !io.start.valid && !hostPending &&
    memory.io
      .read(io.hostRead.bits(3, 0))
      .ready
  when(io.hostRead.fire) {
    hostPending := true.B
    hostBank := io.hostRead.bits(3, 0)
  }
  io.hostResponse.valid := hostPending && memory.io.response(hostBank).valid
  io.hostResponse.bits := memory.io.response(hostBank).bits
  when(io.hostResponse.fire) { hostPending := false.B }
}
