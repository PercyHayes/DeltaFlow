package deltaflow.sa
import chisel3._
import chisel3.util._

class WavePanel(side: Int, tagBits: Int) extends Bundle {
  val os = Bool()
  val context = Bool()
  val tag = UInt(tagBits.W)
  val north = Vec(side, UInt(16.W))
  val west = Vec(side, UInt(32.W))
}
class WaveResult(side: Int, tagBits: Int) extends Bundle {
  val context = Bool()
  val tag = UInt(tagBits.W)
  val values = Vec(side, UInt(32.W))
}

/** Registered skew + array + deskew: (pipelineStages+1)*side enabled cycles. A blocked CS result
  * freezes every path. C32 row responses are independent.
  */
class MixedWavePanel(
    val side: Int,
    val contexts: Int = 2,
    val perWave: Boolean = true,
    tagBits: Int = 2,
    val pipelineStages: Int = 1
) extends Module {
  require(pipelineStages == 1 || pipelineStages == 5)
  val latency = (pipelineStages + 1) * side
  val io = IO(new Bundle {
    val commandOS = Input(Bool())
    val in = Flipped(Decoupled(new WavePanel(side, tagBits)))
    val out = Decoupled(new WaveResult(side, tagBits))
    val coefficient = Flipped(Decoupled(new WaveCoefficient(side)))
    val seed = Flipped(Decoupled(new ContextRowData(side)))
    val clear = Flipped(Decoupled(Bool()))
    val read = Flipped(Decoupled(new ContextRow(side)))
    val response = Decoupled(new ContextRowData(side))
    val contextIdle = Output(Vec(2, Bool()))
    val csIdle = Output(Bool())
    val idle = Output(Bool())
    val configured = Output(Bool())
    val advance = Output(Bool())
  })
  val tile = Module(new MixedWaveMicroTile(side, contexts, perWave, tagBits, pipelineStages))
  val advance = !io.out.valid || io.out.ready
  val os = if (perWave) io.in.bits.os else if (contexts > 0) io.commandOS else false.B
  val ctx = if (contexts == 2) io.in.bits.context else false.B
  val countWidth = log2Ceil(latency + 2)
  val csCount = RegInit(0.U(countWidth.W))
  val osCount = RegInit(VecInit(Seq.fill(2)(0.U(countWidth.W))))
  val incoming = Wire(Valid(new WaveToken(1, tagBits)))
  incoming.valid := io.in.fire
  incoming.bits.value := 0.U
  incoming.bits.os := os
  incoming.bits.context := ctx
  incoming.bits.tag := io.in.bits.tag
  val retired = StreamPipes.delay(incoming, latency, advance)
  io.out.valid := retired.valid && !retired.bits.os
  io.out.bits.tag := retired.bits.tag
  io.out.bits.context := retired.bits.context
  io.advance := advance
  val cooldown = RegInit(VecInit(Seq.fill(2)(0.U(2.W))))
  if (pipelineStages == 5) {
    when(advance) {
      for (i <- 0 until 2) {
        when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
        when(io.in.fire && os && ctx === i.U) { cooldown(i) := 3.U }
      }
    }
  }
  io.in.ready := advance &&
    (!os || cooldown(
      ctx
    ) === 0.U) && (os || (tile.io.coefficientsReady && !io.coefficient.valid))
  tile.io.advance := advance
  tile.io.commandOS := io.commandOS
  for (i <- 0 until side) {
    val n = Wire(Valid(new WaveToken(16, tagBits)))
    val w = Wire(Valid(new WaveToken(32, tagBits)))
    n.valid := io.in.fire
    w.valid := io.in.fire
    n.bits.os := os
    w.bits.os := os
    n.bits.context := ctx
    w.bits.context := ctx
    n.bits.tag := io.in.bits.tag
    w.bits.tag := io.in.bits.tag
    n.bits.value := io.in.bits.north(i)
    w.bits.value := io.in.bits.west(i)
    tile.io.north(i) := StreamPipes.delay(n, pipelineStages * i, advance)
    tile.io.west(i) := StreamPipes.delay(w, i, advance)
    val result = StreamPipes.delay(tile.io.east(i), side - i, advance)
    io.out.bits.values(i) := result.bits.value
    when(retired.valid) {
      assert(
        result.valid && result.bits.os === retired.bits.os &&
          result.bits.tag === retired.bits.tag && result.bits.context === retired.bits.context,
        "panel deskew mismatch"
      )
    }
  }
  val finish = retired.valid && advance
  val csStart = io.in.fire && !os
  val csFinish = finish && !retired.bits.os
  when(csStart =/= csFinish) { csCount := Mux(csStart, csCount + 1.U, csCount - 1.U) }
  when(csFinish) { assert(csCount =/= 0.U) }
  for (i <- 0 until 2) {
    val start = io.in.fire && os && ctx === i.U
    val done = finish && retired.bits.os && retired.bits.context === i.U
    when(start =/= done) { osCount(i) := Mux(start, osCount(i) + 1.U, osCount(i) - 1.U) }
    when(done) { assert(osCount(i) =/= 0.U) }
    io.contextIdle(i) := osCount(i) === 0.U && tile.io.contextIdle(
      i
    ) && !(io.in.valid && os && ctx === i.U)
  }
  io.csIdle := csCount === 0.U && tile.io.csIdle && !(io.in.valid && !os)
  io.idle := io.csIdle && io.contextIdle.asUInt.andR
  io.configured := tile.io.coefficientsReady
  val canLoad = csCount === 0.U && (if (perWave) true.B else osCount.asUInt === 0.U)
  tile.io.coefficient.valid := io.coefficient.valid && canLoad
  tile.io.coefficient.bits := io.coefficient.bits
  io.coefficient.ready := tile.io.coefficient.ready && canLoad
  tile.io.read.valid := io.read.valid && io.contextIdle(io.read.bits.context)
  tile.io.read.bits := io.read.bits
  io.read.ready := tile.io.read.ready && io.contextIdle(io.read.bits.context)
  tile.io.seed.valid := io.seed.valid && io.contextIdle(io.seed.bits.context)
  tile.io.seed.bits := io.seed.bits
  io.seed.ready := tile.io.seed.ready && io.contextIdle(io.seed.bits.context)
  tile.io.clear.valid := io.clear.valid && io.contextIdle(io.clear.bits)
  tile.io.clear.bits := io.clear.bits
  io.clear.ready := tile.io.clear.ready && io.contextIdle(io.clear.bits)
  io.response <> tile.io.response
}
