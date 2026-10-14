package deltaflow.arithmetic

import chisel3._
import chisel3.util._
import easyfloat.{RawFloat, Rounding, RoundingMode, ShiftRightJam}

/** Exact specialization of EasyFloat RawFloat_FMA(9,8,9,8,9,24). Keeps its addend/product anchored
  * normalization and DAZ/FTZ/RNE semantics. Reference source stays unmodified. No rounding at the
  * product boundary.
  */
class ExactProduct extends Bundle {
  val mantissa = UInt(27.W)
  val exp = SInt(10.W)
  val sign = Bool()
  val zero = Bool()
  val inf = Bool()
  val nan = Bool()
}

/** Classification and the selected normalization exponent. Operand mantissas already live in the
  * arithmetic payload; operand flags need not be retained separately once align has computed these
  * exact combined properties.
  */
class FmaClassification extends Bundle {
  val exponent = SInt(10.W)
  val sign = Bool()
  val cancellationSign = Bool()
  val bothZero = Bool()
  val inf = Bool()
  val nan = Bool()
}

class FmaMeta(val compactClassification: Boolean = false) extends Bundle {
  val c = new RawFloat(9, 24)
  val product = new ExactProduct
  val shift = UInt(11.W)
  val anchored = Bool()
  val classification =
    if (compactClassification)
      Some(new FmaClassification)
    else None
}
class AlignedFma(
    val compactClassification: Boolean = false,
    val splitAlignment: Boolean = false
) extends Bundle {
  val c = UInt((if (splitAlignment) 31 else 54).W)
  val coarseShift = if (splitAlignment) Some(UInt(3.W)) else None
  val p = UInt(54.W)
  val subtract = Bool()
  val meta = new FmaMeta(compactClassification)
}
class AddedFma(val compactClassification: Boolean = false) extends Bundle {
  val magnitude = UInt(54.W)
  val cGreater = Bool()
  val meta = new FmaMeta(compactClassification)
}

/** Exact two-stage alignment of a 24-bit addend followed by 23 zeros. A fine shift of 0..7 leaves
  * at least 16 low zeros. Store only the other 31 bits; add reconstructs the zeros and applies the
  * coarse 0/8/..56 shift. Sticky is formed once, after coarse alignment. No arithmetic rounding.
  */
object FmaSplitAlignment {
  def encode(initial: UInt, shift: UInt): (UInt, UInt) = {
    require(initial.getWidth == 47 && shift.getWidth == 6)
    val fine = initial >> shift(2, 0)
    (fine(46, 16), shift(5, 3))
  }

  def restore(partial: UInt, coarse: UInt): UInt = {
    require(partial.getWidth == 31 && coarse.getWidth == 3)
    val initial = Cat(partial, 0.U(16.W))
    val (shifted, sticky) = ShiftRightJam(initial, Cat(coarse, 0.U(3.W)))
    Cat(shifted, sticky)
  }
}

/** Block propagate/generate with a logarithmic prefix, bounded at elaboration. Each 8-bit local sum
  * carries only across its block; no 55-bit ripple chain.
  */
object FmaPrefixAdd {
  def apply(a: UInt, b: UInt, subtract: Bool, width: Int = 55): UInt = {
    require(width >= 1 && width <= 128)
    val bx = b ^ Fill(width, subtract)
    val block = 8
    val groups = (width + block - 1) / block
    val local = (0 until groups).map { i =>
      val hi = math.min(width, block * i + block) - 1
      val lo = block * i
      val av = a(hi, lo)
      val bv = bx(hi, lo)
      (av +& bv, (av ^ bv).andR, hi - lo + 1)
    }
    var g = local.map(x => x._1(x._3))
    var p = local.map(_._2)
    for (level <- 0 until log2Ceil(groups)) {
      val distance = 1 << level
      val oldG = g
      val oldP = p
      g = (0 until groups).map(i =>
        if (i < distance) oldG(i) else oldG(i) || (oldP(i) && oldG(i - distance))
      )
      p = (0 until groups).map(i => if (i < distance) oldP(i) else oldP(i) && oldP(i - distance))
    }
    val sum = (0 until groups).map { i =>
      val carry = if (i == 0) subtract else g(i - 1) || (p(i - 1) && subtract)
      (local(i)._1 + carry)(local(i)._3 - 1, 0)
    }
    Cat(sum.reverse)
  }
}

/** Two subtraction directions share one Brent-Kung carry prefix. For subtraction, prefix G means
  * b_low > a_low and prefix P means equality. Thus carries for b-a and a-b are G|P and !G
  * respectively. Addition uses the ordinary prefix. Aligned BF16 products cannot overflow the
  * 54-bit magnitude in addition: c <= 2^54-2^30 at shift zero, p < 2^28; positive shifts reduce c.
  */
object FmaSharedMagnitude {
  def apply(c: UInt, p: UInt, subtract: Bool): (UInt, Bool) = {
    val magnitudeWidth = c.getWidth
    require(Set(28, 48, 54).contains(magnitudeWidth) && p.getWidth == magnitudeWidth)
    val width = magnitudeWidth + 1
    val a = Cat(0.U(1.W), c)
    val b = Cat(0.U(1.W), p)
    val ax = a ^ Fill(width, subtract)
    val propagate = (b ^ ax).asBools.toVector
    val generate = (b & ax).asBools.toVector
    val levels = log2Ceil(width)
    val count = 1 << levels
    var g = Vector.tabulate(count)(i => if (i < width) generate(i) else false.B)
    var q = Vector.tabulate(count)(i => if (i < width) propagate(i) else true.B)
    for (level <- 0 until levels) {
      val distance = 1 << level
      val oldG = g
      val oldQ = q
      g = Vector.tabulate(count)(i =>
        if ((i + 1) % (2 * distance) == 0)
          oldG(i) || (oldQ(i) && oldG(i - distance))
        else oldG(i)
      )
      q = Vector.tabulate(count)(i =>
        if ((i + 1) % (2 * distance) == 0)
          oldQ(i) && oldQ(i - distance)
        else oldQ(i)
      )
    }
    for (level <- (0 until levels - 1).reverse) {
      val distance = 1 << level
      val oldG = g
      val oldQ = q
      def merge(i: Int): Boolean =
        i >= 3 * distance - 1 && (i - (3 * distance - 1)) % (2 * distance) == 0
      g = Vector.tabulate(count)(i =>
        if (merge(i)) oldG(i) || (oldQ(i) && oldG(i - distance)) else oldG(i)
      )
      q = Vector.tabulate(count)(i => if (merge(i)) oldQ(i) && oldQ(i - distance) else oldQ(i))
    }
    val primary = Cat((0 until width).reverse.map { i =>
      val carry = if (i == 0) subtract else g(i - 1) || (q(i - 1) && subtract)
      propagate(i) ^ carry
    })
    val opposite = Cat((0 until width - 1).reverse.map { i =>
      val carry = if (i == 0) true.B else !g(i - 1)
      propagate(i) ^ carry
    })
    (
      Mux(primary(magnitudeWidth), opposite, primary(magnitudeWidth - 1, 0)),
      primary(magnitudeWidth)
    )
  }
}
class FmaProductStage extends Module {
  val io = IO(new Bundle {
    val a = Input(UInt(16.W))
    val b = Input(UInt(16.W))
    val out = Output(new ExactProduct)
  })
  val a = Wire(new RawFloat(9, 8))
  val b = Wire(new RawFloat(9, 8))
  a.fromIEEE(io.a, 8, 7)
  b.fromIEEE(io.b, 8, 7)
  io.out.mantissa := Cat(a.mantissa * b.mantissa, 0.U(11.W))
  io.out.exp := a.exp +& b.exp
  io.out.sign := a.sign ^ b.sign
  io.out.zero := a.isZero || b.isZero
  io.out.inf := a.isInf || b.isInf
  io.out.nan := a.isNaN || b.isNaN
}
class FmaAlignStage(
    val packedAligned: Boolean = false,
    val compactClassification: Boolean = false,
    val splitAlignment: Boolean = false
) extends Module {
  require(!splitAlignment || packedAligned)
  val io = IO(new Bundle {
    val p = Input(new ExactProduct)
    val c = Input(UInt(32.W))
    val out = Output(new AlignedFma(compactClassification, splitAlignment))
  })
  val c = Wire(new RawFloat(9, 24))
  c.fromIEEE(io.c, 8, 23)
  val diff = io.p.exp +& 27.S -& c.exp
  val shift = Mux(diff < 0.S || io.p.zero, 0.U, diff.tail(1))
  val anchored = (!c.isZero && diff <= 24.S) || io.p.zero
  // Anchored alignment retains six exact low zeros. Otherwise the nonzero
  // addend is shifted by at least 25 and fits below bit 30. The existing
  // anchored flag chooses the scale of this exact 48-bit encoding.
  if (packedAligned) {
    // Use one narrow shift rather than multiplexing two 54-bit positions.
    // For anchored shift<=24, this 47-bit source discards at most one nonzero
    // bit; sticky retains that bit exactly. Unanchored scale compensates six
    // source zero bits. Saturating large shifts bounds the control to 6 bits.
    val initial = Cat(Mux(c.isZero, 0.U(24.W), c.mantissa), 0.U(23.W))
    val encodedShift =
      Mux(shift >= 53.U, 47.U(6.W), Mux(anchored, shift(5, 0), (shift(5, 0) - 6.U)(5, 0)))
    if (splitAlignment) {
      val (partial, coarse) = FmaSplitAlignment.encode(initial, encodedShift)
      io.out.c := partial
      io.out.coarseShift.get := coarse
    } else {
      val (shifted, sticky) = ShiftRightJam(initial, encodedShift)
      io.out.c := Cat(shifted, sticky)
    }
  } else {
    val initial = Cat(Mux(c.isZero, 0.U(24.W), c.mantissa), 0.U(29.W))
    val (shifted, sticky) = ShiftRightJam(initial, shift)
    io.out.c := Cat(shifted, sticky)
  }
  io.out.p := Mux(io.p.zero, 0.U(54.W), Cat(io.p.mantissa, 0.U(1.W)))
  io.out.subtract := io.p.sign ^ c.sign
  if (compactClassification) {
    // The original operand metadata is dead in this specialization. Tie it
    // off explicitly so no unobserved copy becomes a pipeline register.
    io.out.meta.c := 0.U.asTypeOf(io.out.meta.c)
    io.out.meta.product := 0.U.asTypeOf(io.out.meta.product)
    val flags = io.out.meta.classification.get
    val bothZero = c.isZero && io.p.zero
    val nan = io.p.nan || c.isNaN || (c.isInf && io.p.inf && (c.sign ^ io.p.sign)) ||
      (io.p.zero && io.p.inf)
    flags.exponent := Mux(anchored, c.exp.pad(10), io.p.exp)
    flags.sign := Mux(
      c.isInf,
      c.sign,
      Mux(io.p.inf, io.p.sign, Mux(bothZero, c.sign && io.p.sign, io.p.sign))
    )
    flags.cancellationSign := !(c.isInf || io.p.inf || bothZero)
    flags.bothZero := bothZero
    flags.nan := nan
    flags.inf := !nan && (c.isInf || io.p.inf)
  } else {
    io.out.meta.c := c
    io.out.meta.product := io.p
  }
  io.out.meta.shift := shift
  io.out.meta.anchored := anchored
}
class FmaAddStage(
    val sharedMagnitude: Boolean = false,
    val packedAligned: Boolean = false,
    val compactClassification: Boolean = false,
    val splitAlignment: Boolean = false
) extends Module {
  require(!packedAligned || sharedMagnitude)
  require(!splitAlignment || packedAligned)
  val io = IO(new Bundle {
    val in = Input(new AlignedFma(compactClassification, splitAlignment))
    val out = Output(new AddedFma(compactClassification))
  })
  val cGreater = Wire(Bool())
  if (sharedMagnitude) {
    val c =
      if (splitAlignment)
        FmaSplitAlignment.restore(io.in.c, io.in.coarseShift.get)
      else if (packedAligned) io.in.c(47, 0)
      else io.in.c
    // Keep the original product payload in registers: its 12 low zero bits
    // are constant. Storing both scaled positions would waste six DFFs.
    val p =
      if (packedAligned)
        Mux(io.in.meta.anchored, io.in.p(53, 6), io.in.p(47, 0))
      else io.in.p
    val (magnitude, greater) = FmaSharedMagnitude(c, p, io.in.subtract)
    io.out.magnitude := magnitude
    cGreater := greater
  } else {
    val a = Cat(0.U(1.W), io.in.c)
    val b = Cat(0.U(1.W), io.in.p)
    val cMinusP = FmaPrefixAdd(a, b, true.B)
    val pMinusC = FmaPrefixAdd(b, a, io.in.subtract)
    cGreater := pMinusC(54)
    io.out.magnitude := Mux(pMinusC(54), cMinusP(53, 0), pMinusC(53, 0))
  }
  io.out.meta := io.in.meta
  if (compactClassification) {
    val incoming = io.in.meta.classification.get
    // A larger opposite-sign addend flips the finite result sign. Resolve
    // that one-bit choice here, where the magnitude comparison already lives.
    io.out.meta.classification.get.sign :=
      incoming.sign ^ (incoming.cancellationSign && io.in.subtract && cGreater)
    io.out.cGreater := false.B
  } else {
    io.out.cGreater := cGreater
  }
}
class FmaNormalizeStage(
    val boundedShift: Boolean = false,
    val packedAligned: Boolean = false,
    val compactClassification: Boolean = false
) extends Module {
  val io = IO(new Bundle {
    val in = Input(new AddedFma(compactClassification))
    val out = Output(new RawFloat(11, 26))
  })
  val m = io.in.meta
  val p = m.product
  val c = m.c
  // Align sets shift=0 for a negative exponent difference or a zero product.
  // Otherwise anchored implies diff<=24, so only five shift bits are live.
  // The unanchored path does not consume cMant/cExp. Keep the original path
  // available for comparison; this changes neither rounding nor stage count.
  val leftShift = if (boundedShift) m.shift(4, 0) else m.shift
  // The anchored path restores its six zero bits by wiring. The unanchored
  // path already uses the original low-bit scale. No variable scale mux or
  // extra shift adder enters either normalization path.
  val cMagnitude =
    if (packedAligned)
      Cat(io.in.magnitude(47, 0), 0.U(6.W))
    else io.in.magnitude
  val cMant = (cMagnitude << leftShift)(54, 0)
  val msbs = cMant(54, 53)
  val cNorm =
    Mux(msbs(1), cMant, Mux(msbs(0), Cat(cMant(53, 0), 0.U(1.W)), Cat(cMant(52, 0), 0.U(2.W))))
  val cExponent =
    if (compactClassification)
      m.classification.get.exponent.asUInt(8, 0).asSInt
    else c.exp
  val cExp = Mux(msbs(1), cExponent + 1.S, Mux(msbs(0), cExponent, cExponent - 1.S))
  val pMant = io.in.magnitude(29, 0)
  val pZero = !pMant.orR
  val lzc = PriorityEncoder(pMant.asBools.reverse)
  val pExponent =
    if (compactClassification)
      m.classification.get.exponent
    else p.exp
  val pExp = (pExponent +& 3.S) - lzc.zext
  val pNorm = (pMant << lzc)(29, 0)
  if (compactClassification) {
    val flags = m.classification.get
    io.out.sign := Mux(flags.cancellationSign && !m.anchored && pZero, false.B, flags.sign)
    io.out.isZero := flags.bothZero || (!m.anchored && pZero)
    io.out.isNaN := flags.nan
    io.out.isInf := flags.inf
  } else {
    io.out.sign := Mux(
      c.isInf,
      c.sign,
      Mux(
        p.inf,
        p.sign,
        Mux(
          c.isZero && p.zero,
          c.sign && p.sign,
          Mux(!m.anchored && pZero, false.B, Mux(io.in.cGreater, c.sign, p.sign))
        )
      )
    )
    io.out.isZero := (c.isZero && p.zero) || (!m.anchored && pZero)
    io.out.isNaN := p.nan || c.isNaN || (c.isInf && p.inf && (c.sign ^ p.sign)) || (p.zero && p.inf)
    io.out.isInf := !io.out.isNaN && (c.isInf || p.inf)
  }
  io.out.exp := Mux(m.anchored, cExp, pExp)
  io.out.mantissa := Mux(
    m.anchored,
    Cat(cNorm(54, 30), cNorm(29, 0).orR),
    Cat(pNorm(29, 5), pNorm(4, 0).orR)
  )
}
class FmaRoundStage extends Module {
  val io = IO(new Bundle {
    val in = Input(new RawFloat(11, 26))
    val out = Output(UInt(32.W))
  })
  io.out := FastFp32Round(io.in)
}

class PipelineMacRequest(tagBits: Int = 2) extends Bundle {
  val a = UInt(16.W)
  val b = UInt(16.W)
  val c = UInt(32.W)
  val os = Bool()
  val context = Bool()
  val tag = UInt(tagBits.W)
}
class PipelineMacResult(tagBits: Int = 2) extends Bundle {
  val value = UInt(32.W)
  val os = Bool()
  val context = Bool()
  val tag = UInt(tagBits.W)
}
class FmaTag(tagBits: Int) extends Bundle {
  val os = Bool()
  val context = Bool()
  val tag = UInt(tagBits.W)
}

/** Four/five registered stages, two architectural C32, late accumulator read. OS admission reserves
  * its context until normalization/round commit. A new product can enter on that commit cycle and
  * read C32 on the following cycle. CS fills independent issue slots. Output backpressure freezes
  * all stages.
  */
class PipelinedMac(val fusedRound: Boolean = false, val tagBits: Int = 2) extends Module {
  require(tagBits >= 1 && tagBits <= 16)
  val stages = if (fusedRound) 4 else 5
  val commitIndex = stages - 2
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new PipelineMacRequest(tagBits)))
    val out = Decoupled(new PipelineMacResult(tagBits))
    val accumulators = Output(Vec(2, UInt(32.W)))
    val contextReady = Output(Vec(2, Bool()))
    val busy = Output(Bool())
    val advance = Output(Bool())
    val seed = Flipped(Decoupled(Vec(2, UInt(32.W))))
  })
  val product = Module(new FmaProductStage)
  val align = Module(new FmaAlignStage)
  val add = Module(new FmaAddStage)
  val normalize = Module(new FmaNormalizeStage)
  val round = Module(new FmaRoundStage)
  val valid = RegInit(VecInit(Seq.fill(stages)(false.B)))
  val tags = Reg(Vec(stages - 1, new FmaTag(tagBits)))
  val c32 = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))
  val owners = RegInit(VecInit(Seq.fill(2)(false.B)))
  val preg = Reg(new ExactProduct)
  val csAddend = Reg(UInt(32.W))
  val areg = Reg(new AlignedFma)
  val sumreg = Reg(new AddedFma)
  val nreg = if (fusedRound) Wire(new RawFloat(11, 26)) else Reg(new RawFloat(11, 26))
  val result = Reg(new PipelineMacResult(tagBits))
  io.out.valid := valid(stages - 1)
  io.out.bits := result
  val advance = !valid(stages - 1) || io.out.ready
  val commit = valid(commitIndex) && advance && tags(commitIndex).os
  for (i <- 0 until 2)
    io.contextReady(i) := !owners(i) || (commit && tags(commitIndex).context === i.U)
  io.in.ready := advance &&
    (!io.in.bits.os || io.contextReady(
      io.in.bits.context
    )) && !io.seed.valid
  io.seed.ready := !valid.asUInt.orR && !io.in.valid
  io.busy := valid.asUInt.orR
  io.advance := advance
  io.accumulators := c32
  product.io.a := io.in.bits.a
  product.io.b := io.in.bits.b
  align.io.p := preg
  align.io.c := Mux(tags(0).os, c32(tags(0).context), csAddend)
  add.io.in := areg
  normalize.io.in := sumreg
  round.io.in := nreg
  if (fusedRound) { nreg := normalize.io.out }
  when(advance) {
    valid(0) := io.in.fire
    for (i <- 1 until stages) valid(i) := valid(i - 1)
    when(io.in.fire) {
      preg := product.io.out
      csAddend := io.in.bits.c
      tags(0).os := io.in.bits.os
      tags(0).context := io.in.bits.context
      tags(0).tag := io.in.bits.tag
    }
    for (i <- 1 until stages - 1) when(valid(i - 1)) { tags(i) := tags(i - 1) }
    when(valid(0)) { areg := align.io.out }
    when(valid(1)) { sumreg := add.io.out }
    if (!fusedRound) { when(valid(2)) { nreg := normalize.io.out } }
    when(valid(commitIndex)) {
      result.value := round.io.out
      result.os := tags(commitIndex).os
      result.context := tags(commitIndex).context
      result.tag := tags(commitIndex).tag
      when(tags(commitIndex).os) { c32(tags(commitIndex).context) := round.io.out }
    }
  }
  for (i <- 0 until 2) {
    val take = io.in.fire && io.in.bits.os && io.in.bits.context === i.U
    val release = commit && tags(commitIndex).context === i.U
    when(take || release) { owners(i) := take }
  }
  when(io.seed.fire) { c32 := io.seed.bits }
}
