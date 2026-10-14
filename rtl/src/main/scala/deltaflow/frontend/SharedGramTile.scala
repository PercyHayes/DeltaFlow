package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.memory._

/** Integration harness: ONE set of sixteen retimed panels and ONE scratchpad. Result rows are
  * simultaneously published and committed, so stalled consumers keep their data stable and never
  * observe an uncommitted result.
  */
class SharedGramTile(val side: Int = 16, val diagonalFirst: Boolean = false) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W)))
    val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(20.W))
    val result = Vec(4, Decoupled(new GramRow(side)))
    val hostRead = Flipped(Decoupled(UInt(13.W)))
    val hostResponse = Decoupled(UInt(128.W))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(Vec(16, Valid(UInt(5.W))))
    val feature = Output(Vec(16, UInt(7.W)))
    val captured = Output(UInt(20.W))
    val bankReads = Output(UInt(16.W))
    val bankReturns = Output(UInt(16.W))
    val bufferOccupancy = Output(Vec(16, UInt(2.W)))
  })
  val scheduler = Module(new SharedGramController(side, diagonalFirst))
  val memory = Module(new MacroScratchpad(16, 512, 16))
  val panels = Seq.fill(16)(Module(new RetimedWavePanel(side, rowResponseDepth = 1)))
  val hostPending = RegInit(false.B)
  val hostBank = Reg(UInt(4.W))
  // Word-level initialized bits enforce the external host contract without
  // resetting SRAM. Inputs are checked by the test/controller caller; only
  // committed output words can be read through the host result interface.
  val indexBits = log2Ceil(20 * side)
  val written = RegInit(VecInit(Seq.fill(20 * side)(false.B)))
  val offset = io.hostRead.bits - 4096.U
  val hostRow = offset / (side / 4).U
  val hostAllowed = !scheduler.io.busy && !io.start.valid && io.hostRead.bits >= 4096.U &&
    hostRow < (20 * side).U && written(hostRow(indexBits - 1, 0))
  io.hostRead.ready := !hostPending && hostAllowed && memory.io.read(io.hostRead.bits(3, 0)).ready
  when(io.hostRead.fire) {
    hostPending := true.B
    hostBank := io.hostRead.bits(3, 0)
  }
  io.hostResponse.valid := hostPending && memory.io.response(hostBank).valid
  io.hostResponse.bits := memory.io.response(hostBank).bits
  when(io.hostResponse.fire) { hostPending := false.B }
  scheduler.io.start.valid := io.start.valid && !hostPending && !io.init.map(_.valid).reduce(_ || _)
  scheduler.io.start.bits := io.start.bits
  io.start.ready := scheduler.io.start.ready && !hostPending && !io.init.map(_.valid).reduce(_ || _)
  scheduler.io.bankEnable := io.bankEnable
  scheduler.io.jobEnable := io.jobEnable
  io.result <> scheduler.io.result
  when(scheduler.io.start.fire) { for (i <- 0 until 20 * side) written(i) := false.B }
  for (p <- 0 until 4) when(io.result(p).fire) {
    val r = io.result(p).bits
    val physicalRow = (r.job >> 2) * (4 * side).U + r.row * 4.U + p.U
    written(physicalRow(indexBits - 1, 0)) := true.B
  }
  for (t <- 0 until 16) {
    val panel = panels(t)
    panel.io.in <> scheduler.io.in(t)
    panel.io.clear <> scheduler.io.clear(t)
    panel.io.read <> scheduler.io.read(t)
    scheduler.io.response(t) <> panel.io.response
    scheduler.io.contextIdle(t) := panel.io.contextIdle
    panel.io.coefficient.valid := false.B
    panel.io.coefficient.bits := 0.U.asTypeOf(panel.io.coefficient.bits)
    panel.io.seed.valid := false.B
    panel.io.seed.bits := 0.U.asTypeOf(panel.io.seed.bits)
    panel.io.out.ready := true.B
    assert(!panel.io.out.valid, "Gram issued an unexpected CS wave")
  }
  for (b <- 0 until 16) {
    memory.io.read(b) <> scheduler.io.bankRead(b)
    scheduler.io.bankResponse(b) <> memory.io.response(b)
    when(io.hostRead.valid && !hostPending && hostAllowed && io.hostRead.bits(3, 0) === b.U) {
      memory.io.read(b).valid := true.B
      memory.io.read(b).bits := io.hostRead.bits(12, 4)
    }
    when(hostPending) {
      scheduler.io.bankResponse(b).valid := false.B
      memory.io.response(b).ready := hostBank === b.U && io.hostResponse.ready
    }
    val p = (b / (side / 4)) % 4
    val result = io.result(p)
    val base =
      4096.U + (result.bits.job >> 2) * (side * side).U + result.bits.row * side.U +
        (p * side / 4).U
    val hits = (0 until side / 4).map(w => (base + w.U)(3, 0) === b.U)
    val commit = result.fire && hits.reduce(_ || _)
    val initAllowed = !scheduler.io.busy && !hostPending && !io.start.valid && !io.hostRead.valid
    memory.io.write(b).valid := commit || (io.init(b).valid && initAllowed)
    memory.io.write(b).bits := io.init(b).bits
    when(commit) {
      memory.io.write(b).bits.address := Mux1H(
        hits,
        (0 until side / 4).map(w => (base + w.U)(12, 4))
      )
      memory.io.write(b).bits.data := Mux1H(
        hits,
        (0 until side / 4).map(w =>
          Cat((0 until 4).reverse.map(k => result.bits.values(4 * w + k)))
        )
      )
      memory.io.write(b).bits.mask := "hffff".U
      assert(memory.io.write(b).ready)
    }
    io.init(b).ready := initAllowed && memory.io.write(b).ready
    when(io.init(b).fire) {
      assert(io.init(b).bits.address < 128.U, "host may only initialize Gram operands")
    }
  }
  io.busy := scheduler.io.busy
  io.done := scheduler.io.done
  io.issued := scheduler.io.issued
  io.feature := scheduler.io.feature
  io.captured := scheduler.io.captured
  io.bufferOccupancy := scheduler.io.bufferOccupancy
  io.bankReads := scheduler.io.allocated
  io.bankReturns := VecInit(scheduler.io.bankResponse.map(_.fire)).asUInt
}
