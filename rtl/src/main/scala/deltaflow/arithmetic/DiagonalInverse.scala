package deltaflow.arithmetic

import chisel3._
import chisel3.util._
import easyfloat.FMA

class DiagonalEntry(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val col = UInt(log2Ceil(side).W)
  val value = UInt(32.W)
}

/** A bounded scalar FP32 preparation engine. Does not claim 128-lane throughput. J(r,j) = -L(r,j) -
  * sum(k=j+1..r-1) L(r,k) J(k,j). All three iteration variables are hardware counters, bounded by
  * side<=16.
  */
class DiagonalInverse(val side: Int) extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  val io = IO(new Bundle {
    val load = Flipped(Decoupled(new DiagonalEntry(side)))
    val start = Flipped(Decoupled(Bool()))
    val busy = Output(Bool())
    val done = Output(Bool())
    val result = Output(Vec(side, Vec(side, UInt(32.W))))
  })
  val l = RegInit(VecInit(Seq.fill(side * side)(0.U(32.W))))
  val j = RegInit(VecInit(Seq.fill(side * side)(0.U(32.W))))
  val idle :: seed :: multiply :: save :: Nil = Enum(4)
  val state = RegInit(idle)
  val row = Reg(UInt(log2Ceil(side).W))
  val col = Reg(UInt(log2Ceil(side).W))
  val k = Reg(UInt(log2Ceil(side).W))
  val sum = Reg(UInt(32.W))
  def index(r: UInt, c: UInt): UInt = Cat(r, c)
  def negative(x: UInt): UInt = x ^ "h80000000".U(32.W)
  val fma = Module(new FMA(8, 23, 8, 23))
  fma.io.a := negative(l(index(row, k)))
  fma.io.b := j(index(k, col))
  fma.io.c := sum
  io.load.ready := state === idle && !io.start.valid && io.load.bits.row > io.load.bits.col
  when(io.load.fire) { l(index(io.load.bits.row, io.load.bits.col)) := io.load.bits.value }
  io.start.ready := state === idle && !io.load.valid
  val done = RegInit(false.B)
  done := false.B
  when(io.start.fire) {
    for (r <- 0 until side; c <- 0 until side) {
      j(r * side + c) := (if (r == c) "h3f800000".U else 0.U)
    }
    row := 1.U
    col := 0.U
    state := seed
  }
  switch(state) {
    is(seed) {
      sum := negative(l(index(row, col)))
      k := col + 1.U
      state := Mux(col + 1.U < row, multiply, save)
    }
    is(multiply) {
      sum := fma.io.out
      when(k === row - 1.U) { state := save }.otherwise { k := k + 1.U }
    }
    is(save) {
      j(index(row, col)) := sum
      when(col === row - 1.U) {
        when(row === (side - 1).U) {
          state := idle
          done := true.B
        }
          .otherwise {
            row := row + 1.U
            col := 0.U
            state := seed
          }
      }.otherwise {
        col := col + 1.U
        state := seed
      }
    }
  }
  for (r <- 0 until side; c <- 0 until side) { io.result(r)(c) := j(r * side + c) }
  io.busy := state =/= idle
  io.done := done
}
