package deltaflow.arithmetic

import chisel3._
import easyfloat.{FMA, RawFloat, Rounding, RoundingMode}

/** The same elem/acc separation as FSA, without its attention-only SFU. */
object FloatArithmetic {
  val elemWidth = 16
  val accWidth = 32
  def narrow(x: UInt): UInt = {
    require(x.getWidth == accWidth)
    val raw = Wire(new RawFloat(9, 24))
    raw.fromIEEE(x, 8, 23)
    Rounding.round(raw, RoundingMode.RNE, 8, 7)
  }
}

class MixedFma extends Module {
  val io = IO(new Bundle {
    val a = Input(UInt(16.W))
    val b = Input(UInt(16.W))
    val c = Input(UInt(32.W))
    val result = Output(UInt(32.W))
    val narrowed = Output(UInt(16.W))
  })
  val fma = Module(new FMA(8, 7, 8, 23))
  fma.io.a := io.a
  fma.io.b := io.b
  fma.io.c := io.c
  io.result := fma.io.out
  // Explicit C32 -> BF16 rounding, not a separately rounded raw FMA result.
  io.narrowed := FloatArithmetic.narrow(fma.io.out)
}
