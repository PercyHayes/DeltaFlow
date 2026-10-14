package deltaflow.sa

import chisel3._
import chisel3.util._

class CoefficientWrite(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val col = UInt(log2Ceil(side).W)
  val value = UInt(16.W)
}

class RowCoefficient(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val values = Vec(side, UInt(16.W))
}

class MicroTile(val side: Int, val withRows: Boolean = false) extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val outputStationary = Input(Bool())
    val coefficient = Flipped(Valid(new CoefficientWrite(side)))
    val rowCoefficient = if (withRows) Some(Input(Valid(new RowCoefficient(side)))) else None
    val clearAcc = Input(Bool())
    val publishAcc = Input(Bool())
    val north = Flipped(Vec(side, Valid(UInt(16.W))))
    val west = Flipped(Vec(side, Valid(UInt(32.W))))
    val south = Vec(side, Valid(UInt(16.W)))
    val east = Vec(side, Valid(UInt(32.W)))
    val accumulators = Output(Vec(side, Vec(side, UInt(32.W))))
    val coefficientsReady = Output(Bool())
  })
  val pe = Seq.fill(side, side)(Module(new NativePE))
  for (r <- 0 until side; k <- 0 until side) {
    val p = pe(r)(k)
    p.io.advance := io.advance
    p.io.outputStationary := io.outputStationary
    p.io.coefficient.valid :=
      io.coefficient.valid && io.coefficient.bits.row === r.U && io.coefficient.bits.col === k.U
    p.io.coefficient.bits := io.coefficient.bits.value
    io.rowCoefficient.foreach { row =>
      assert(!(row.valid && io.coefficient.valid), "scalar/row coefficient collision")
      when(row.valid && row.bits.row === r.U) {
        p.io.coefficient.valid := true.B
        p.io.coefficient.bits := row.bits.values(k)
      }
    }
    p.io.clearAcc := io.clearAcc
    p.io.publishAcc := io.publishAcc
    p.io.north := (if (r == 0) io.north(k) else pe(r - 1)(k).io.south)
    p.io.west := (if (k == 0) io.west(r) else pe(r)(k - 1).io.east)
    io.accumulators(r)(k) := p.io.accumulator
  }
  for (i <- 0 until side) {
    io.east(i) := pe(i)(side - 1).io.east
    io.south(i) := pe(side - 1)(i).io.south
  }
  io.coefficientsReady := pe.flatten.map(_.io.weightLoaded).reduce(_ && _)
}
