package deltaflow.arithmetic

import chisel3._
import chisel3.util._
import easyfloat.{RawFloat, ShiftRightJam}

class NarrowFmaFlags extends Bundle {
  val exponent = SInt(11.W)
  val sign = Bool()
  val cancellationSign = Bool()
  val bothZero = Bool()
  val inf = Bool()
  val nan = Bool()
}

class NarrowFmaAligned extends Bundle {
  val large = UInt(24.W)
  val small = UInt(27.W)
  val subtract = Bool()
  val flags = new NarrowFmaFlags
}

class NarrowFmaAdded extends Bundle {
  val magnitude = UInt(28.W)
  val flags = new NarrowFmaFlags
}

/** Normalize the exact 16-bit BF16 product without converting it to FP32. Its extended exponent
  * remains live until the existing final rounding. Both operands have at most 24 significant bits.
  * Three low alignment bits preserve guard, round and sticky information, including cancellation.
  */
class NarrowFmaAlignStage extends Module {
  val io = IO(new Bundle {
    val p = Input(new ExactProduct)
    val c = Input(UInt(32.W))
    val out = Output(new NarrowFmaAligned)
  })
  val c = Wire(new RawFloat(9, 24))
  c.fromIEEE(io.c, 8, 23)
  val product = io.p.mantissa(26, 11)
  val high = product(15)
  // BF16 exponents and the normalized product fit in signed ten bits.
  val productExponent = io.p.exp + high.asUInt.zext
  val productMantissa =
    Mux(io.p.zero, 0.U(24.W), Mux(high, Cat(product, 0.U(8.W)), Cat(product(14, 0), 0.U(9.W))))
  val cMantissa = Mux(c.isZero, 0.U(24.W), c.mantissa)
  val exponentDelta = c.exp.pad(10) - productExponent
  val productLargerExponent = exponentDelta < 0.S
  val cLargest = !c.isZero && (io.p.zero || !productLargerExponent)
  val largeExponent = Mux(cLargest, c.exp.pad(11), productExponent.pad(11))
  val small = Cat(Mux(cLargest, productMantissa, cMantissa), 0.U(3.W))
  // Zero operands may select an exponent independently of delta; their
  // aligned payload is zero for every shift. Other operands use |delta|.
  val distance = Mux(productLargerExponent, (-exponentDelta).asUInt, exponentDelta.asUInt)
  val shift = Mux(distance >= 27.U, 27.U(5.W), distance(4, 0))
  val (shifted, sticky) = ShiftRightJam(small, shift)
  io.out.large := Mux(cLargest, cMantissa, productMantissa)
  io.out.small := shifted | sticky.asUInt
  io.out.subtract := c.sign ^ io.p.sign

  val bothZero = c.isZero && io.p.zero
  val nan = io.p.nan || c.isNaN || (c.isInf && io.p.inf && (c.sign ^ io.p.sign)) ||
    (io.p.zero && io.p.inf)
  val flags = io.out.flags
  flags.exponent := largeExponent
  flags.sign := Mux(
    c.isInf,
    c.sign,
    Mux(io.p.inf, io.p.sign, Mux(bothZero, c.sign && io.p.sign, Mux(cLargest, c.sign, io.p.sign)))
  )
  flags.cancellationSign := !(c.isInf || io.p.inf || bothZero)
  flags.bothZero := bothZero
  flags.inf := !nan && (c.isInf || io.p.inf)
  flags.nan := nan
}

/** One carry prefix handles addition and both subtraction directions. A 28-bit unsigned magnitude
  * retains the addition carry; its signed prefix needs 29 bits. The largest possible sum is
  * strictly below 2^28.
  */
class NarrowFmaAddStage extends Module {
  val io = IO(new Bundle {
    val in = Input(new NarrowFmaAligned)
    val out = Output(new NarrowFmaAdded)
  })
  val large = Cat(0.U(1.W), io.in.large, 0.U(3.W))
  val small = Cat(0.U(1.W), io.in.small)
  val (magnitude, negative) = FmaSharedMagnitude(small, large, io.in.subtract)
  io.out.magnitude := magnitude
  io.out.flags := io.in.flags
  io.out.flags.sign :=
    io.in.flags.sign ^ (io.in.flags.cancellationSign && io.in.subtract && negative)
}

class NarrowFmaNormalizeStage extends Module {
  val io = IO(new Bundle {
    val in = Input(new NarrowFmaAdded)
    val out = Output(new RawFloat(11, 26))
  })
  val magnitude = io.in.magnitude
  val leadingZeros = PriorityEncoder(magnitude.asBools.reverse)
  val normalized = (magnitude << leadingZeros)(27, 0)
  val zero = magnitude === 0.U
  val flags = io.in.flags
  io.out.exp := flags.exponent + 1.S - leadingZeros.zext
  io.out.mantissa := Cat(normalized(27, 3), normalized(2, 0).orR)
  io.out.sign := Mux(flags.cancellationSign && zero, false.B, flags.sign)
  io.out.isZero := flags.bothZero || zero
  io.out.isInf := flags.inf
  io.out.isNaN := flags.nan
}

/** Four arithmetic payload registers and combinational rounding. Product, alignment, magnitude,
  * normalization and the caller's commit form the same five stages as RetimedWavePE. C is sampled
  * one enabled edge after A/B, so OS retains the existing four-cycle C32 feedback interval.
  */
class NarrowFmaDatapath extends Module {
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val a = Input(UInt(16.W))
    val b = Input(UInt(16.W))
    val lateC = Input(UInt(32.W))
    val result = Output(UInt(32.W))
  })
  val product = Module(new FmaProductStage)
  val align = Module(new NarrowFmaAlignStage)
  val add = Module(new NarrowFmaAddStage)
  val normalize = Module(new NarrowFmaNormalizeStage)
  val pr = Reg(new ExactProduct)
  val ar = Reg(new NarrowFmaAligned)
  val sr = Reg(new NarrowFmaAdded)
  val nr = Reg(new RawFloat(11, 26))
  product.io.a := io.a
  product.io.b := io.b
  align.io.p := pr
  align.io.c := io.lateC
  add.io.in := ar
  normalize.io.in := sr
  io.result := FastFp32Round(nr)
  when(io.advance) {
    pr := product.io.out
    ar := align.io.out
    sr := add.io.out
    nr := normalize.io.out
  }
}
