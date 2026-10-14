package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic.FloatArithmetic

class BlockCoefficient(side: Int) extends CoefficientWrite(side) {
  val block = UInt(2.W) // J0, -L10, J1
}
class RhsColumn(side: Int, tagBits: Int) extends Bundle {
  val r0 = Vec(side, UInt(32.W))
  val r1 = Vec(side, UInt(32.W))
  val tag = UInt(tagBits.W)
}
class SolvedColumn(side: Int, tagBits: Int) extends Bundle {
  val x0 = Vec(side, UInt(16.W))
  val x1 = Vec(side, UInt(16.W))
  val tag = UInt(tagBits.W)
}

/** Three physical microtiles. All in-flight registers share an enable. Output backpressure freezes
  * operands, partial sums, turn pipes and tags.
  */
class PreparationPort(side: Int) extends Bundle {
  val active = Input(Bool())
  val gram = Input(Bool())
  val clear = Input(Bool())
  val north = Input(Vec(3, Vec(side, Valid(UInt(16.W)))))
  val west = Input(Vec(3, Vec(side, Valid(UInt(32.W)))))
  val rows = Input(Vec(3, Valid(new RowCoefficient(side))))
  val accumulators = Output(Vec(3, Vec(side, Vec(side, UInt(32.W)))))
}

class TwoBlockCore(val side: Int, val tagBits: Int = 9, val withPreparation: Boolean = false)
    extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  require(tagBits > 0 && tagBits <= 16)
  val latency = 4 * side + 2
  val io = IO(new Bundle {
    val coefficient = Flipped(Decoupled(new BlockCoefficient(side)))
    val in = Flipped(Decoupled(new RhsColumn(side, tagBits)))
    val out = Decoupled(new SolvedColumn(side, tagBits))
    val idle = Output(Bool())
    val coefficientsReady = Output(Bool())
    val preparation = if (withPreparation) Some(new PreparationPort(side)) else None
  })
  val tiles = Seq.fill(3)(Module(new MicroTile(side, withPreparation)))
  val turn0 = Module(new DiagonalTurn(side))
  val turn1 = Module(new DiagonalTurn(side))
  val turnOut = Module(new DiagonalTurn(side))
  val outstanding = RegInit(0.U(log2Ceil(latency + 2).W))
  val configured = tiles.map(_.io.coefficientsReady).reduce(_ && _)
  val preparing = io.preparation.map(_.active).getOrElse(false.B)
  val advance = !io.out.valid || io.out.ready
  io.coefficient.ready := outstanding === 0.U && io.coefficient.bits.block < 3.U && !preparing
  io.in.ready := advance && configured && !io.coefficient.valid && !preparing
  val launch = io.in.fire
  io.idle := outstanding === 0.U
  io.coefficientsReady := configured
  when(io.in.fire =/= io.out.fire) {
    when(io.in.fire) { outstanding := outstanding + 1.U }
      .otherwise { outstanding := outstanding - 1.U }
  }
  when(io.out.fire) { assert(outstanding =/= 0.U, "output without an input") }

  for (i <- 0 until 3) {
    tiles(i).io.advance := advance
    tiles(i).io.outputStationary := false.B
    tiles(i).io.coefficient.valid := io.coefficient.fire && io.coefficient.bits.block === i.U
    tiles(i).io.coefficient.bits.row := io.coefficient.bits.row
    tiles(i).io.coefficient.bits.col := io.coefficient.bits.col
    tiles(i).io.coefficient.bits.value := io.coefficient.bits.value
    tiles(i).io.clearAcc := false.B
    tiles(i).io.publishAcc := false.B
  }
  turn0.io.advance := advance
  turn1.io.advance := advance
  turnOut.io.advance := advance
  turn0.io.in := tiles(0).io.east
  turn1.io.in := tiles(1).io.east
  turnOut.io.in := tiles(2).io.east
  for (r <- 0 until side) {
    val a = Wire(Valid(UInt(16.W)))
    a.valid := launch
    a.bits := FloatArithmetic.narrow(io.in.bits.r0(r))
    val z = Wire(Valid(UInt(32.W)))
    z.valid := launch
    z.bits := 0.U
    val residual = Wire(Valid(UInt(32.W)))
    residual.valid := launch
    residual.bits := io.in.bits.r1(r)
    tiles(0).io.north(r) := StreamPipes.delay(a, r, advance)
    tiles(0).io.west(r) := StreamPipes.delay(z, r, advance)
    tiles(1).io.north(r) := turn0.io.out(r)
    tiles(1).io.west(r) := StreamPipes.delay(residual, side + 1 + r, advance)
    tiles(2).io.north(r) := turn1.io.out(r)
    tiles(2).io.west(r) := StreamPipes.delay(z, 2 * side + 2 + r, advance)
    val x0 = StreamPipes.delay(turn0.io.out(r), 3 * side + 1 - r, advance)
    val x1 = StreamPipes.delay(turnOut.io.out(r), side - 1 - r, advance)
    io.out.bits.x0(r) := x0.bits
    io.out.bits.x1(r) := x1.bits
    assert(x0.valid === x1.valid, "two-block output phase mismatch")
    when(io.out.valid) { assert(x1.valid, "missing output lane") }
  }
  io.preparation.foreach { p =>
    when(p.active) { assert(outstanding === 0.U, "preparation during solve") }
    for (i <- 0 until 3) {
      tiles(i).io.rowCoefficient.get := p.rows(i)
      when(p.active) {
        tiles(i).io.outputStationary := p.gram
        tiles(i).io.clearAcc := p.clear
        tiles(i).io.north := p.north(i)
        tiles(i).io.west := p.west(i)
      }
      p.accumulators(i) := tiles(i).io.accumulators
      assert(!p.rows(i).valid || p.active)
    }
    when(p.active) {
      for (r <- 0 until side) {
        turn0.io.in(r).valid := false.B
        turn1.io.in(r).valid := false.B
        turnOut.io.in(r).valid := false.B
      }
    }
  }
  val tag = Wire(Valid(UInt(tagBits.W)))
  tag.valid := launch
  tag.bits := io.in.bits.tag
  val delayedTag = StreamPipes.delay(tag, latency, advance)
  io.out.valid := delayedTag.valid
  io.out.bits.tag := delayedTag.bits
}
