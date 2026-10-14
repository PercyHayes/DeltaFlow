package deltaflow.system
import chisel3._
import chisel3.util._
import deltaflow.dma._

class SystemCompletion extends Bundle {
  val tile = UInt(2.W)
  val port = UInt(2.W)
  val dma = Bool()
  val tag = UInt(16.W)
  val opcode = UInt(8.W)
  val error = Bool()
}

/** Unified instruction stream, four identical shared compute/storage tiles. Physical AXI masters
  * are ordered tile*4+port; offsets refer to independently bound HBM windows. No CPU/debug path
  * moves operand or output payloads.
  */
class FourTileSystem(val side: Int = 16, val compactPe: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val instruction = Flipped(Decoupled(UInt(128.W)))
    val completion = Decoupled(new SystemCompletion)
    val hbm = Vec(16, new AxiMaster)
    val idle = Output(Bool())
    val issued = Output(Vec(4, UInt(16.W)))
    val gdnMode = Output(UInt(4.W))
    val faultMask = Output(UInt(4.W))
  })
  val tiles = (0 until 4).map(i => Module(new UnifiedComputeTile(side, i, compactPe)))
  val engines = Seq.fill(16)(Module(new AxiDma))
  val instructions = Module(new Queue(UInt(128.W), 8))
  instructions.io.enq <> io.instruction
  val replies = Module(new RRArbiter(new SystemCompletion, 21))
  val status = Module(new Queue(new SystemCompletion, 2))
  replies.io.in(20) <> status.io.deq
  io.completion <> replies.io.out
  val faults = RegInit(0.U(4.W))
  io.faultMask := faults
  // System status has no tile source; its default tile field must not poison
  // tile zero when another tile fails a masked command or fence.
  val engineFault = io.completion.fire && io.completion.bits.error && !status.io.deq.fire
  when(engineFault) { faults := faults | (1.U(4.W) << io.completion.bits.tile) }
  val word = instructions.io.deq.bits
  val op = word(7, 0)
  val mask = word(11, 8)
  val ports = word(15, 12)
  val dma = op === 16.U || op === 17.U
  val fence = op === 18.U || op === 19.U || op === 20.U
  val known =
    op <= 4.U || dma || fence || op === 33.U || op === 34.U || op === 35.U || op === 36.U ||
      op === 37.U
  val invalid =
    !known || !mask.orR || (dma && (!ports.orR || word(127, 114).orR)) ||
      (fence &&
        (word(
          127,
          32
        ).orR || ports.orR))
  val dmaReady = (0 until 16)
    .map(i => !mask(i / 4) || !ports(i % 4) || engines(i).io.command.ready)
    .reduce(_ && _)
  val computeReady = (0 until 4).map(i => !mask(i) || tiles(i).io.instruction.ready).reduce(_ && _)
  val fenceReady = (0 until 4)
    .map(i =>
      !mask(i) || Mux(
        op === 20.U,
        engines.slice(4 * i, 4 * i + 4).map(_.io.idle).reduce(_ && _),
        Mux(op === 19.U, tiles(i).io.buffersIdle, tiles(i).io.idle)
      )
    )
    .reduce(_ && _)
  instructions.io.deq.ready := Mux(
    invalid,
    status.io.enq.ready,
    Mux(dma, dmaReady, Mux(fence, fenceReady && status.io.enq.ready, computeReady))
  )
  status.io.enq.valid := instructions.io.deq.valid && (invalid || fence && fenceReady)
  status.io.enq.bits := 0.U.asTypeOf(status.io.enq.bits)
  status.io.enq.bits.tag := word(31, 16)
  status.io.enq.bits.opcode := op
  status.io.enq.bits.error := invalid || (fence && (faults & mask).orR)
  when(status.io.enq.fire && invalid) {
    faults := faults | mask | Mux(engineFault, 1.U(4.W) << io.completion.bits.tile, 0.U)
  }
  for (i <- 0 until 4) {
    val tile = tiles(i)
    tile.io.instruction.valid := instructions.io.deq.valid && !invalid && !dma && !fence && mask(
      i
    ) && computeReady
    tile.io.instruction.bits := word
    tile.io.dmaIdle := engines.slice(4 * i, 4 * i + 4).map(_.io.idle).reduce(_ && _)
    io.issued(i) := tile.io.issued
    val reply = replies.io.in(i)
    reply.valid := tile.io.completion.valid
    tile.io.completion.ready := reply.ready
    reply.bits := 0.U.asTypeOf(reply.bits)
    reply.bits.tile := i.U
    reply.bits.tag := tile.io.completion.bits.tag
    reply.bits.opcode := tile.io.completion.bits.opcode
    reply.bits.error := tile.io.completion.bits.error
  }
  io.gdnMode := VecInit(tiles.map(_.io.gdnMode)).asUInt
  for (i <- 0 until 16) {
    val e = engines(i)
    io.hbm(i) <> e.io.axi
    tiles(i / 4).io.dma(i % 4) <> e.io.local
    e.io.command.valid := instructions.io.deq.valid && dma && !invalid && mask(i / 4) && ports(
      i % 4
    ) && dmaReady
    e.io.command.bits.store := op === 17.U
    e.io.command.bits.region := word(112)
    e.io.command.bits.stride2 := word(113)
    e.io.command.bits.address := word(79, 32)
    e.io.command.bits.local := word(95, 80)
    e.io.command.bits.beats := word(111, 96)
    e.io.command.bits.tag := word(31, 16)
    val reply = replies.io.in(4 + i)
    reply.valid := e.io.completion.valid
    e.io.completion.ready := reply.ready
    reply.bits.tile := (i / 4).U
    reply.bits.port := (i % 4).U
    reply.bits.dma := true.B
    reply.bits.tag := e.io.completion.bits.tag
    reply.bits.opcode := Mux(e.io.completion.bits.store, 17.U, 16.U)
    reply.bits.error := e.io.completion.bits.error
  }
  io.idle := tiles.map(_.io.idle).reduce(_ && _) &&
    engines
      .map(_.io.idle)
      .reduce(_ && _) && !instructions.io.deq.valid && !io.completion.valid
}
