package deltaflow.arithmetic
import chisel3._
import chisel3.util._
import easyfloat.{RawFloat, ShiftRightJam}

/** Same DAZ/FTZ and RNE as EasyFloat. Predict the rounding carry in parallel with the fraction
  * increment; exponent bias/limits do not follow that adder.
  */
object FastFp32Round {
  def apply(raw: RawFloat): UInt = {
    require(raw.mantissa.getWidth == 26 && raw.exp.getWidth == 11)
    val fraction = raw.mantissa(24, 2)
    val up = raw.mantissa(1) && (raw.mantissa(0) || fraction(0))
    val carry = fraction.andR && up
    val mantissa = (fraction + up)(22, 0)
    val overflow = raw.exp >= 128.S || (raw.exp === 127.S && carry)
    val underflow = raw.exp < (-127).S || (raw.exp === (-127).S && !carry)
    val base = (raw.exp.asUInt + 127.U)(7, 0)
    val bumped = (raw.exp.asUInt + 128.U)(7, 0)
    val exp = Mux(
      raw.isInf || raw.isNaN || overflow,
      255.U(8.W),
      Mux(raw.isZero || underflow, 0.U(8.W), Mux(carry, bumped, base))
    )
    val frac = Mux(
      raw.isNaN,
      "h400000".U(23.W),
      Mux(raw.isZero || raw.isInf || underflow || overflow, 0.U(23.W), mantissa)
    )
    Cat(raw.sign && !raw.isNaN, exp, frac)
  }
}
class Fp32ProductMeta extends Bundle {
  val c = UInt(32.W)
  val exp = SInt(10.W)
  val sign = Bool()
  val zero = Bool()
  val inf = Bool()
  val nan = Bool()
}
class Fp32Partials extends Bundle {
  val parts = Vec(4, UInt(24.W))
  val meta = new Fp32ProductMeta
}
class Fp32Middle extends Bundle {
  val outer = UInt(48.W)
  val middle = UInt(25.W)
  val meta = new Fp32ProductMeta
}
class Fp32Product extends Bundle {
  val value = UInt(48.W)
  val meta = new Fp32ProductMeta
}
class Fp32Aligned extends Bundle {
  val c = UInt(75.W)
  val p = UInt(75.W)
  val subtract = Bool()
  val shift = UInt(11.W)
  val anchored = Bool()
  val meta = new Fp32ProductMeta
  val addend = new RawFloat(9, 24)
}
class Fp32Added extends Bundle {
  val value = UInt(75.W)
  val cGreater = Bool()
  val aligned = new Fp32Aligned
}
class Fp32Norm extends Bundle {
  val cMant = UInt(76.W)
  val pMant = UInt(51.W)
  val lzc = UInt(6.W)
  val cExp = SInt(9.W)
  val pExp = SInt(11.W)
  val anchored = Bool()
  val sign = Bool()
  val zero = Bool()
  val inf = Bool()
  val nan = Bool()
}

/** Eight registered stages, fixed latency/II=1, no output backpressure. Caller reserves the result
  * destination before asserting in.valid. Reset cancels all in-flight operations; it must also
  * reset the caller's tags. Exact 24x24 product: four 12x12 multipliers, middle sum, final product
  * sum. No intermediate product rounding. One logical FP32 FMA lane.
  */
class PipelinedFp32(val earlyProduct: Boolean = false) extends Module {
  val latency = 8
  val io = IO(new Bundle {
    val in = Flipped(Valid(new FmaOperands))
    val out = Valid(UInt(32.W))
    val product = if (earlyProduct) Some(Valid(UInt(32.W))) else None
  })
  val valid = RegInit(VecInit(Seq.fill(latency)(false.B)))
  valid(0) := io.in.valid
  for (i <- 1 until latency) valid(i) := valid(i - 1)
  val partial = Reg(new Fp32Partials)
  val middle = Reg(new Fp32Middle)
  val product = Reg(new Fp32Product)
  val aligned = Reg(new Fp32Aligned)
  val added = Reg(new Fp32Added)
  val norm = Reg(new Fp32Norm)
  val raw = Reg(new RawFloat(11, 26))
  val result = Reg(UInt(32.W))
  val a = Wire(new RawFloat(9, 24))
  val b = Wire(new RawFloat(9, 24))
  a.fromIEEE(io.in.bits.a, 8, 23)
  b.fromIEEE(io.in.bits.b, 8, 23)
  when(io.in.valid) {
    for (i <- 0 until 2; j <- 0 until 2)
      partial.parts(2 * i + j) := a.mantissa(12 * i + 11, 12 * i) * b.mantissa(12 * j + 11, 12 * j)
    partial.meta.c := io.in.bits.c
    partial.meta.exp := a.exp +& b.exp
    partial.meta.sign := a.sign ^ b.sign
    partial.meta.zero := a.isZero || b.isZero
    partial.meta.inf := a.isInf || b.isInf
    partial.meta.nan := a.isNaN || b.isNaN
  }
  when(valid(0)) {
    middle.outer := Cat(partial.parts(3), partial.parts(0))
    middle.middle := partial.parts(1) +& partial.parts(2)
    middle.meta := partial.meta
  }
  when(valid(1)) {
    product.value := FmaPrefixAdd(
      middle.outer,
      Cat(0.U(11.W), middle.middle, 0.U(12.W)),
      false.B,
      48
    )
    product.meta := middle.meta
  }
  // Reuse the exact 48-bit product, never the BF16 or rounded FMA operand.
  // This endpoint computes fma(a,b,+0), with the same RNE/DAZ/FTZ contract,
  // regardless of the full FMA's c. The ordinary eight-stage result is intact.
  io.product.foreach { port =>
    val pr = Wire(new RawFloat(11, 26))
    val high = product.value(47)
    pr.exp := product.meta.exp.pad(11) + high.asUInt.zext
    pr.mantissa := Mux(
      high,
      Cat(product.value(47, 23), product.value(22, 0).orR),
      Cat(product.value(46, 22), product.value(21, 0).orR)
    )
    pr.isNaN := product.meta.nan || (product.meta.zero && product.meta.inf)
    pr.isInf := product.meta.inf && !pr.isNaN
    pr.isZero := product.meta.zero
    pr.sign := product.meta.sign && !product.meta.zero
    port.valid := valid(3)
    port.bits := RegEnable(FastFp32Round(pr), valid(2))
  }
  val c = Wire(new RawFloat(9, 24))
  c.fromIEEE(product.meta.c, 8, 23)
  val diff = product.meta.exp +& 27.S -& c.exp
  val shift = Mux(diff < 0.S || product.meta.zero, 0.U, diff.tail(1))
  val (shifted, sticky) = ShiftRightJam(Cat(Mux(c.isZero, 0.U(24.W), c.mantissa), 0.U(50.W)), shift)
  when(valid(2)) {
    aligned.c := Cat(shifted, sticky)
    aligned.p := Mux(product.meta.zero, 0.U, Cat(product.value, 0.U(1.W)))
    aligned.subtract := product.meta.sign ^ c.sign
    aligned.shift := shift
    aligned.anchored := (!c.isZero && diff <= 24.S) || product.meta.zero
    aligned.meta := product.meta
    aligned.addend := c
  }
  val ap = Cat(0.U(1.W), aligned.p)
  val ac = Cat(0.U(1.W), aligned.c)
  val pmc = FmaPrefixAdd(ap, ac, aligned.subtract, 76)
  val cmp = FmaPrefixAdd(ac, ap, true.B, 76)
  when(valid(3)) {
    added.value := Mux(pmc(75), cmp(74, 0), pmc(74, 0))
    added.cGreater := pmc(75)
    added.aligned := aligned
  }
  val m = added.aligned
  val pm = added.value(50, 0)
  val pzero = !pm.orR
  when(valid(4)) {
    norm.cMant := (added.value << m.shift)(75, 0)
    norm.cExp := m.addend.exp
    norm.pMant := pm
    norm.lzc := PriorityEncoder(pm.asBools.reverse)
    norm.pExp := m.meta.exp +& 3.S
    norm.anchored := m.anchored
    norm.sign := Mux(
      m.addend.isInf,
      m.addend.sign,
      Mux(
        m.meta.inf,
        m.meta.sign,
        Mux(
          m.addend.isZero && m.meta.zero,
          m.addend.sign && m.meta.sign,
          Mux(!m.anchored && pzero, false.B, Mux(added.cGreater, m.addend.sign, m.meta.sign))
        )
      )
    )
    norm.zero := (m.addend.isZero && m.meta.zero) || (!m.anchored && pzero)
    val nan =
      m.meta.nan || m.addend.isNaN ||
        (m.addend.isInf && m.meta.inf && (m.addend.sign ^ m.meta.sign)) ||
        (m.meta.zero && m.meta.inf)
    norm.nan := nan
    norm.inf := !nan && (m.addend.isInf || m.meta.inf)
  }
  val cn = Mux(
    norm.cMant(75),
    norm.cMant,
    Mux(norm.cMant(74), Cat(norm.cMant(74, 0), 0.U(1.W)), Cat(norm.cMant(73, 0), 0.U(2.W)))
  )
  val pn = (norm.pMant << norm.lzc)(50, 0)
  when(valid(5)) {
    raw.exp := Mux(
      norm.anchored,
      Mux(norm.cMant(75), norm.cExp + 1.S, Mux(norm.cMant(74), norm.cExp, norm.cExp - 1.S)),
      norm.pExp - norm.lzc.zext
    )
    raw.mantissa := Mux(
      norm.anchored,
      Cat(cn(75, 51), cn(50, 0).orR),
      Cat(pn(50, 26), pn(25, 0).orR)
    )
    raw.sign := norm.sign
    raw.isZero := norm.zero
    raw.isInf := norm.inf
    raw.isNaN := norm.nan
  }
  when(valid(6)) { result := FastFp32Round(raw) }
  io.out.valid := valid(7)
  io.out.bits := result
}

/** Explicit operand boundary for a shared-lane scheduler: the arbitration and wide source mux
  * cannot be concatenated with the first multiplier stage.
  */
class RegisteredFp32(val earlyProduct: Boolean = false) extends Module {
  val latency = 9
  val io = IO(new Bundle {
    val in = Flipped(Valid(new FmaOperands))
    val out = Valid(UInt(32.W))
    val product = if (earlyProduct) Some(Valid(UInt(32.W))) else None
  })
  val core = Module(new PipelinedFp32(earlyProduct))
  core.io.in.valid := RegNext(io.in.valid, false.B)
  core.io.in.bits := RegEnable(io.in.bits, io.in.valid)
  io.out <> core.io.out
  io.product.foreach(_ <> core.io.product.get)
}
