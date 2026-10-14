package deltaflow.arithmetic

import chisel3._
import chisel3.util._

class FloatRow(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val values = Vec(side, UInt(32.W))
}
class FmaOperands extends Bundle {
  val a = UInt(32.W)
  val b = UInt(32.W)
  val c = UInt(32.W)
}

/** Row-driven unit-triangular inverse. Arithmetic is borrowed from external lanes. Columns j
  * advance in parallel, but each element preserves k=j+1..r-1 order. Only the current L row and
  * completed FP32 J rows are stored here.
  */
class RowInverseController(val side: Int) extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(Bool()))
    val load = Flipped(Decoupled(new FloatRow(side)))
    val result = Decoupled(new FloatRow(side))
    val operands = Output(Vec(side, new FmaOperands))
    val fmaResult = Input(Vec(side, UInt(32.W)))
    val usingFma = Output(Bool())
    val busy = Output(Bool())
  })
  val idle :: receive :: compute :: publish :: Nil = Enum(4)
  val state = RegInit(idle)
  val row = RegInit(0.U(log2Ceil(side).W))
  val k = Reg(UInt(log2Ceil(side).W))
  val currentL = Reg(Vec(side, UInt(32.W)))
  val sums = Reg(Vec(side, UInt(32.W)))
  val completed = Reg(Vec(side, Vec(side, UInt(32.W))))
  io.start.ready := state === idle && !io.load.valid
  io.load.ready := state === receive && io.load.bits.row === row
  io.result.valid := state === publish
  io.result.bits.row := row
  io.result.bits.values := sums
  io.usingFma := state === compute
  io.busy := state =/= idle
  for (j <- 0 until side) {
    io.operands(j).a := currentL(k) ^ "h80000000".U
    io.operands(j).b := completed(k)(j)
    io.operands(j).c := sums(j)
  }
  when(io.start.fire) {
    row := 0.U
    state := receive
  }
  when(io.load.fire) {
    currentL := io.load.bits.values
    for (j <- 0 until side) {
      sums(j) := Mux(
        j.U < row,
        io.load.bits.values(j) ^ "h80000000".U,
        Mux(j.U === row, "h3f800000".U, 0.U)
      )
    }
    k := 1.U
    state := Mux(row <= 1.U, publish, compute)
  }
  when(state === compute) {
    assert(k > 0.U && k < row)
    for (j <- 0 until side) when(j.U < k) { sums(j) := io.fmaResult(j) }
    when(k === row - 1.U) { state := publish }.otherwise { k := k + 1.U }
  }
  when(io.result.fire) {
    completed(row) := sums
    when(row === (side - 1).U) { state := idle }
      .otherwise {
        row := row + 1.U
        state := receive
      }
  }
}
