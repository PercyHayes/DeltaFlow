package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic.{FloatArithmetic, MixedFma}

/** One registered hop, coherently stalled with its containing wavefront. Configuration and mode
  * changes are legal only after the array is drained.
  */
class NativePE extends Module {
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val outputStationary = Input(Bool())
    val coefficient = Flipped(Valid(UInt(16.W)))
    val clearAcc = Input(Bool())
    val publishAcc = Input(Bool())
    val north = Flipped(Valid(UInt(16.W)))
    val west = Flipped(Valid(UInt(32.W)))
    val south = Valid(UInt(16.W))
    val east = Valid(UInt(32.W))
    val accumulator = Output(UInt(32.W))
    val weight = Output(UInt(16.W))
    val weightLoaded = Output(Bool())
  })
  val r16 = RegInit(0.U(16.W))
  val c32 = RegInit(0.U(32.W))
  val loaded = RegInit(false.B)
  val south = RegInit(0.U.asTypeOf(Valid(UInt(16.W))))
  val east = RegInit(0.U.asTypeOf(Valid(UInt(32.W))))
  val mac = Module(new MixedFma)
  mac.io.a := Mux(io.outputStationary, io.west.bits(15, 0), r16)
  mac.io.b := io.north.bits
  mac.io.c := Mux(io.outputStationary, c32, io.west.bits)
  val multiply = io.north.valid && io.west.valid

  when(io.coefficient.valid) {
    r16 := io.coefficient.bits
    loaded := true.B
  }.elsewhen(io.publishAcc) {
    r16 := FloatArithmetic.narrow(c32)
    loaded := true.B
  }
  when(io.clearAcc) { c32 := 0.U }
  when(io.advance) {
    south := io.north
    east.valid := Mux(io.outputStationary, io.west.valid, multiply)
    east.bits := Mux(io.outputStationary, Cat(0.U(16.W), io.west.bits(15, 0)), mac.io.result)
    when(multiply && io.outputStationary && !io.clearAcc) { c32 := mac.io.result }
    when(multiply && !io.outputStationary) {
      assert(loaded, "uninitialized stationary coefficient")
    }
  }
  assert(!(io.coefficient.valid && io.publishAcc), "two coefficient writers")
  assert(!(io.clearAcc && io.publishAcc), "clear/publish collision")
  when(io.coefficient.valid || io.publishAcc || io.clearAcc) {
    assert(!io.north.valid && !io.west.valid, "configuration during an operand beat")
  }
  io.south := south
  io.east := east
  io.accumulator := c32
  io.weight := r16
  io.weightLoaded := loaded
}
