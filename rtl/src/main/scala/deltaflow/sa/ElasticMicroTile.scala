package deltaflow.sa

import chisel3._
import chisel3.util._

class Panel(val lanes: Int, val bits: Int, val tagBits: Int = 9) extends Bundle {
  val tag = UInt(tagBits.W)
  val values = Vec(lanes, UInt(bits.W))
}
class MacPanel(side: Int, tagBits: Int) extends Bundle {
  val tag = UInt(tagBits.W)
  val operand = Vec(side, UInt(16.W))
  val initial = Vec(side, UInt(32.W))
}

class MicroPreparation(side: Int) extends Bundle {
  val active = Input(Bool())
  val gram = Input(Bool())
  val clear = Input(Bool())
  val north = Input(Vec(side, Valid(UInt(16.W))))
  val west = Input(Vec(side, Valid(UInt(32.W))))
  val accumulators = Output(Vec(side, Vec(side, UInt(32.W))))
}

/** A complete column transaction around one coefficient-stationary microtile. Independent local
  * enable; fixed skew/deskew only advances with this tile.
  */
class ElasticMicroTile(val side: Int, val tagBits: Int = 9, val withPreparation: Boolean = false)
    extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  require(tagBits >= 1 && tagBits <= 16)
  val latency = 2 * side - 1
  val io = IO(new Bundle {
    val coefficient = Flipped(Decoupled(new RowCoefficient(side)))
    val in = Flipped(Decoupled(new MacPanel(side, tagBits)))
    val out = Decoupled(new Panel(side, 32, tagBits))
    val idle = Output(Bool())
    val configured = Output(Bool())
    val stalled = Output(Bool())
    val preparation = if (withPreparation) Some(new MicroPreparation(side)) else None
  })
  val tile = Module(new MicroTile(side, withRows = true))
  val outstanding = RegInit(0.U(log2Ceil(latency + 2).W))
  val preparing = io.preparation.map(_.active).getOrElse(false.B)
  val advance = !io.out.valid || io.out.ready
  io.coefficient.ready := outstanding === 0.U
  tile.io.rowCoefficient.get.valid := io.coefficient.fire
  tile.io.rowCoefficient.get.bits := io.coefficient.bits
  tile.io.coefficient.valid := false.B
  tile.io.coefficient.bits := 0.U.asTypeOf(tile.io.coefficient.bits)
  tile.io.clearAcc := false.B
  tile.io.publishAcc := false.B
  tile.io.outputStationary := false.B
  tile.io.advance := advance
  io.in.ready := advance && tile.io.coefficientsReady && !io.coefficient.valid && !preparing
  val launch = io.in.fire
  val tag = Wire(Valid(UInt(tagBits.W)))
  tag.valid := launch
  tag.bits := io.in.bits.tag
  val outputTag = StreamPipes.delay(tag, latency, advance)
  io.out.valid := outputTag.valid
  io.out.bits.tag := outputTag.bits
  for (r <- 0 until side) {
    val a = Wire(Valid(UInt(16.W)))
    val c = Wire(Valid(UInt(32.W)))
    a.valid := launch
    a.bits := io.in.bits.operand(r)
    c.valid := launch
    c.bits := io.in.bits.initial(r)
    tile.io.north(r) := StreamPipes.delay(a, r, advance)
    tile.io.west(r) := StreamPipes.delay(c, r, advance)
    val result = StreamPipes.delay(tile.io.east(r), side - 1 - r, advance)
    io.out.bits.values(r) := result.bits
    when(!preparing) {
      assert(result.valid === outputTag.valid, "elastic tile lane/tag misalignment")
    }
  }
  io.preparation.foreach { p =>
    p.accumulators := tile.io.accumulators
    when(p.active) {
      assert(outstanding === 0.U, "Gram preparation overlaps live panel data")
      tile.io.advance := true.B
      tile.io.outputStationary := p.gram
      tile.io.clearAcc := p.clear
      tile.io.north := p.north
      tile.io.west := p.west
      assert(!(p.gram && io.coefficient.valid))
    }
  }
  when(io.in.fire =/= io.out.fire) {
    when(io.in.fire) { outstanding := outstanding + 1.U }.otherwise {
      outstanding := outstanding - 1.U
    }
  }
  when(io.out.fire) { assert(outstanding =/= 0.U) }
  io.idle := outstanding === 0.U
  io.configured := tile.io.coefficientsReady
  io.stalled := outstanding =/= 0.U && !advance
}

/** Bounded multicast: atomic enqueue to enabled destinations, independent dequeue. A slow consumer
  * eventually backpressures the source; faster consumers never consume a frame twice while waiting
  * for it. Mask changes only while drained.
  */
class PanelBroadcast(side: Int, consumers: Int = 4, depth: Int = 2, tagBits: Int = 9)
    extends Module {
  require(consumers >= 1 && consumers <= 4 && depth >= 1 && depth <= 8)
  val io = IO(new Bundle {
    val mask = Input(UInt(consumers.W))
    val in = Flipped(Decoupled(new Panel(side, 16, tagBits)))
    val out = Vec(consumers, Decoupled(new Panel(side, 16, tagBits)))
    val idle = Output(Bool())
  })
  // Registered occupancy breaks ready loops across the two mode-dependent routes.
  val queues =
    Seq.fill(consumers)(Module(new Queue(new Panel(side, 16, tagBits), depth, pipe = false)))
  val allReady = (0 until consumers).map(i => !io.mask(i) || queues(i).io.enq.ready).reduce(_ && _)
  io.in.ready := allReady && io.mask.orR
  for (i <- 0 until consumers) {
    queues(i).io.enq.valid := io.in.valid && io.in.ready && io.mask(i)
    queues(i).io.enq.bits := io.in.bits
    io.out(i) <> queues(i).io.deq
    when(!io.mask(i)) { assert(queues(i).io.count === 0.U) }
  }
  io.idle := queues.map(_.io.count === 0.U).reduce(_ && _)
}
