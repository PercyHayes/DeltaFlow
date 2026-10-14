package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.memory._
import chisel3.experimental.hierarchy._

/** Three-chunk integration harness: ONE set of sixteen retimed panels and independent operand/work
  * scratchpads. Result rows are simultaneously published and committed, so stalled consumers keep
  * their data stable and never observe an uncommitted result.
  */
class MultiChunkGramTile(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W)))
    val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(60.W))
    val result = Vec(4, Decoupled(new MultiGramRow(side)))
    val hostRead = Flipped(Decoupled(UInt(13.W)))
    val hostResponse = Decoupled(UInt(128.W))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(Vec(16, Valid(UInt(6.W))))
    val feature = Output(Vec(16, UInt(7.W)))
    val captured = Output(UInt(60.W))
    val bankReads = Output(UInt(16.W))
    val bankReturns = Output(UInt(16.W))
    val bufferOccupancy = Output(Vec(16, UInt(3.W)))
  })
  val scheduler = Module(new MultiChunkGramController(side))
  val memory = Module(new MacroScratchpad(16, 512, 16))
  val work = Module(new MacroScratchpad(16, 512, 16))
  val definition = Definition(new RetimedWavePanel(side, rowResponseDepth = 1))
  val panels = Seq.fill(16)(Instance(definition))
  val hostPending = RegInit(false.B)
  val hostBank = Reg(UInt(4.W))
  // Word-level initialized bits enforce the external host contract without
  // resetting SRAM. Inputs are checked by the test/controller caller; only
  // committed output words can be read through the host result interface.
  val indexBits = log2Ceil(60 * side)
  val written = RegInit(VecInit(Seq.fill(60 * side)(false.B)))
  val offset = io.hostRead.bits
  val hostRow = offset / (side / 4).U
  val hostAllowed = !scheduler.io.busy && !io.start.valid && hostRow < (60 * side).U &&
    written(hostRow(indexBits - 1, 0))
  io.hostRead.ready := !hostPending && hostAllowed && work.io.read(io.hostRead.bits(3, 0)).ready
  when(io.hostRead.fire) {
    hostPending := true.B
    hostBank := io.hostRead.bits(3, 0)
  }
  io.hostResponse.valid := hostPending && work.io.response(hostBank).valid
  io.hostResponse.bits := work.io.response(hostBank).bits
  when(io.hostResponse.fire) { hostPending := false.B }
  scheduler.io.start.valid := io.start.valid && !hostPending && !io.init.map(_.valid).reduce(_ || _)
  scheduler.io.start.bits := io.start.bits
  io.start.ready := scheduler.io.start.ready && !hostPending && !io.init.map(_.valid).reduce(_ || _)
  scheduler.io.bankEnable := io.bankEnable
  scheduler.io.jobEnable := io.jobEnable
  io.result <> scheduler.io.result
  when(scheduler.io.start.fire) { for (i <- 0 until 60 * side) written(i) := false.B }
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
    work.io.read(b).valid := io.hostRead.valid && !hostPending && hostAllowed && io.hostRead.bits(
      3,
      0
    ) === b.U
    work.io.read(b).bits := io.hostRead.bits(12, 4)
    work.io.response(b).ready := hostPending && hostBank === b.U && io.hostResponse.ready
    val p = (b / (side / 4)) % 4
    val result = io.result(p)
    val base =
      ((result.bits.job >> 2) * (side * side).U + result.bits.row * side.U + (p * side / 4).U)
        .pad(13)
    val hits = (0 until side / 4).map(w => (base + w.U)(3, 0) === b.U)
    val commit = result.fire && hits.reduce(_ || _)
    val initAllowed = !scheduler.io.busy && !hostPending && !io.start.valid && !io.hostRead.valid
    memory.io.write(b).valid := io.init(b).valid && initAllowed
    memory.io.write(b).bits := io.init(b).bits
    work.io.write(b).valid := commit
    work.io.write(b).bits := 0.U.asTypeOf(work.io.write(b).bits)
    when(commit) {
      work.io.write(b).bits.address := Mux1H(hits, (0 until side / 4).map(w => (base + w.U)(12, 4)))
      work.io.write(b).bits.data := Mux1H(
        hits,
        (0 until side / 4).map(w =>
          Cat((0 until 4).reverse.map(k => result.bits.values(4 * w + k)))
        )
      )
      work.io.write(b).bits.mask := "hffff".U
      assert(work.io.write(b).ready)
    }
    io.init(b).ready := initAllowed && memory.io.write(b).ready
    when(io.init(b).fire) {
      assert(io.init(b).bits.address < 384.U, "host may only initialize Gram operands")
    }
  }
  io.busy := scheduler.io.busy
  io.done := scheduler.io.done
  io.issued := scheduler.io.issued
  io.feature := scheduler.io.feature
  io.captured := scheduler.io.captured
  io.bufferOccupancy := scheduler.io.occupancy
  io.bankReads := scheduler.io.reads
  io.bankReturns := VecInit(scheduler.io.bankResponse.map(_.fire)).asUInt
}
