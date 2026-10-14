package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.isa._

class FourTileGemmSystem(val side: Int = 16) extends Module {
  val t = 4 * side
  val io = IO(new Bundle {
    val instruction = Flipped(Decoupled(UInt(128.W)))
    val completion = Decoupled(new GemmCompletion)
    val load = Flipped(Vec(16, Decoupled(new GemmLoad)))
    val hostRead = Flipped(Vec(4, Decoupled(new GemmHostRead)))
    val hostResponse = Vec(4, Decoupled(UInt(128.W)))
    val storeEnable = Input(Vec(4, Bool()))
    val issued = Output(Vec(4, Bool()))
    val committed = Vec(4, Valid(new GemmStoreTrace(t)))
    val idle = Output(Bool())
  })
  val tiles = (0 until 4).map(i => Module(new GemmInstructionTile(side, i)))
  val mask = io.instruction.bits(11, 8)
  val ready = (0 until 4).map(i => !mask(i) || tiles(i).io.instruction.ready).reduce(_ && _)
  val replies = Module(new RRArbiter(new GemmCompletion, 5))
  val invalid = Module(new Queue(new GemmCompletion, 2))
  invalid.io.enq.valid := io.instruction.valid && !mask.orR
  invalid.io.enq.bits.tile := 0.U
  invalid.io.enq.bits.tag := io.instruction.bits(31, 16)
  invalid.io.enq.bits.opcode := io.instruction.bits(7, 0)
  invalid.io.enq.bits.error := true.B
  io.instruction.ready := Mux(mask.orR, ready, invalid.io.enq.ready)
  for (i <- 0 until 4) {
    tiles(i).io.instruction.valid := io.instruction.valid && mask(i) && ready
    tiles(i).io.instruction.bits := io.instruction.bits
    for (p <- 0 until 4) tiles(i).io.load(p) <> io.load(i * 4 + p)
    tiles(i).io.hostRead <> io.hostRead(i)
    io.hostResponse(i) <> tiles(i).io.hostResponse
    tiles(i).io.storeEnable := io.storeEnable(i)
    io.issued(i) := tiles(i).io.issued
    io.committed(i) := tiles(i).io.committed
    replies.io.in(i) <> tiles(i).io.completion
  }
  replies.io.in(4) <> invalid.io.deq
  io.completion <> replies.io.out
  io.idle := tiles.map(_.io.idle).reduce(_ && _) && !io.completion.valid && !invalid.io.deq.valid
}
