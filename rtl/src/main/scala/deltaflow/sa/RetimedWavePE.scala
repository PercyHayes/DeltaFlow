package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic._

class RetimedMeta(tagBits: Int) extends Bundle {
  val os = Bool()
  val context = UInt(2.W)
  val tag = UInt(tagBits.W)
}

/** Product starts with north. CS partial sum arrives one enabled cycle later, directly at align; OS
  * reads its selected C32 at align. Five arithmetic stages, four-cycle recurrence and horizontal
  * propagation stay fixed. Separate operand and partial-sum wires permit adjacent OS and CS tokens.
  */
class RetimedWavePE(
    tagBits: Int = 2,
    val coefficientBanks: Int = 1,
    val compactWestMetadata: Boolean = false,
    val sharedMagnitude: Boolean = false,
    val freeInvalidPayload: Boolean = false,
    val packedAligned: Boolean = false,
    val compactClassification: Boolean = false,
    val splitAlignment: Boolean = false,
    val narrowAddition: Boolean = false,
    val freeForwardPayload: Boolean = false
) extends Module {
  require(tagBits >= 1 && tagBits <= 16)
  require(Set(1, 2).contains(coefficientBanks))
  require(!packedAligned || sharedMagnitude)
  require(!splitAlignment || packedAligned)
  require(!narrowAddition || (sharedMagnitude && freeInvalidPayload))
  require(!freeForwardPayload || freeInvalidPayload)

  val io = IO(new Bundle {
    val advance = Input(Bool())
    val north = Flipped(Valid(new CapacityToken(16, 4, tagBits)))
    val westOperand = Flipped(Valid(new CapacityToken(16, 4, tagBits)))
    val westSum = Flipped(Valid(new CapacityToken(32, 4, tagBits)))
    val south = Valid(new CapacityToken(16, 4, tagBits))
    val eastOperand = Valid(new CapacityToken(16, 4, tagBits))
    val eastSum = Valid(new CapacityToken(32, 4, tagBits))
    val seed = Flipped(Valid(new CapacityValue(4)))
    val clear = Flipped(Valid(UInt(2.W)))
    val coefficient = Flipped(Valid(UInt(16.W)))
    val coefficientBank = if (coefficientBanks == 2) Some(Input(Bool())) else None
    val accumulators = Output(Vec(4, UInt(32.W)))
  })

  val weights = Reg(Vec(coefficientBanks, UInt(16.W)))
  val loaded = RegInit(VecInit(Seq.fill(coefficientBanks)(false.B)))
  val c32 = RegInit(VecInit(Seq.fill(4)(0.U(32.W))))
  val valid = RegInit(VecInit(Seq.fill(4)(false.B)))
  val meta = Reg(Vec(4, new RetimedMeta(tagBits)))
  val os = io.north.bits.os
  val ctx = io.north.bits.context
  val fire = io.north.valid

  // CS uses context[0] as its coefficient version; it never addresses C32.
  val readBank = if (coefficientBanks == 2) ctx(0) else 0.U
  val writeBank = io.coefficientBank.getOrElse(false.B)
  val operandA = Mux(os, io.westOperand.bits.value, weights(readBank))
  val lateC = Mux(meta(0).os, c32(meta(0).context), io.westSum.bits.value)
  val arithmeticResult = Wire(UInt(32.W))

  if (narrowAddition) {
    val arithmetic = Module(new NarrowFmaDatapath)
    arithmetic.io.advance := io.advance
    arithmetic.io.a := operandA
    arithmetic.io.b := io.north.bits.value
    arithmetic.io.lateC := lateC
    arithmeticResult := arithmetic.io.result
  } else {
    val product = Module(new FmaProductStage)
    val align = Module(new FmaAlignStage(packedAligned, compactClassification, splitAlignment))
    val add = Module(
      new FmaAddStage(sharedMagnitude, packedAligned, compactClassification, splitAlignment)
    )
    val normalize = Module(
      new FmaNormalizeStage(
        boundedShift = sharedMagnitude,
        packedAligned = packedAligned,
        compactClassification = compactClassification
      )
    )
    val round = Module(new FmaRoundStage)
    val pr = Reg(new ExactProduct)
    val ar = Reg(new AlignedFma(compactClassification, splitAlignment))
    val sr = Reg(new AddedFma(compactClassification))
    val nr = Reg(new easyfloat.RawFloat(11, 26))
    product.io.a := operandA
    product.io.b := io.north.bits.value
    align.io.p := pr
    align.io.c := lateC
    add.io.in := ar
    normalize.io.in := sr
    round.io.in := nr
    arithmeticResult := round.io.out

    when(io.advance) {
      if (freeInvalidPayload) {
        pr := product.io.out
        ar := align.io.out
        sr := add.io.out
        nr := normalize.io.out
      } else {
        when(fire) { pr := product.io.out }
        when(valid(0)) { ar := align.io.out }
        when(valid(1)) { sr := add.io.out }
        when(valid(2)) { nr := normalize.io.out }
      }
      if (sharedMagnitude) {
        when(valid(1) && !ar.subtract) {
          val productValue =
            if (packedAligned)
              Mux(ar.meta.anchored, ar.p(53, 6), ar.p(47, 0))
            else ar.p
          val width = if (packedAligned) 48 else 54
          val addend =
            if (splitAlignment)
              FmaSplitAlignment.restore(ar.c, ar.coarseShift.get)
            else ar.c
          assert(
            (addend +& productValue) < (BigInt(1) << width).U,
            "aligned addition magnitude overflow"
          )
        }
        when(valid(2)) {
          assert(!sr.meta.anchored || sr.meta.shift <= 24.U, "anchored normalization shift bound")
        }
      }
    }
  }

  io.south := StreamPipes.delay(io.north, 1, io.advance, freeForwardPayload)
  if (compactWestMetadata) {
    // A west OS token is aligned with north at each PE. Its metadata already
    // lives in the arithmetic pipe; only its operand needs a separate link.
    val operand = Wire(Valid(UInt(16.W)))
    operand.valid := io.westOperand.valid
    operand.bits := io.westOperand.bits.value
    val delayed = StreamPipes.delay(operand, 4, io.advance, freeForwardPayload)
    io.eastOperand.valid := delayed.valid
    io.eastOperand.bits.value := delayed.bits
    io.eastOperand.bits.os := true.B
    io.eastOperand.bits.context := meta(3).context
    io.eastOperand.bits.tag := meta(3).tag

    when(io.advance) {
      assert(io.westOperand.valid === (fire && os), "west/north OS wave alignment")
      when(io.westOperand.valid) {
        assert(io.westOperand.bits.os)
        assert(io.westOperand.bits.context === ctx)
        assert(io.westOperand.bits.tag === io.north.bits.tag)
      }
      assert(delayed.valid === (valid(3) && meta(3).os), "shared west metadata retirement")
    }
  } else {
    io.eastOperand := StreamPipes.delay(io.westOperand, 4, io.advance, freeForwardPayload)
  }

  val east = Wire(Valid(new CapacityToken(32, 4, tagBits)))
  val eastPayload =
    if (freeForwardPayload) Reg(new CapacityToken(32, 4, tagBits))
    else RegInit(0.U.asTypeOf(new CapacityToken(32, 4, tagBits)))
  val eastValid = RegInit(false.B)
  east.bits := eastPayload
  east.valid := eastValid
  io.eastSum := east
  val cooldown = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))

  when(io.coefficient.valid) {
    weights(writeBank) := io.coefficient.bits
    loaded(writeBank) := true.B
  }

  when(io.advance) {
    // Payloads are observed only under delayed valid. On an enabled edge a
    // valid token overwrites its stage before that stage can be consumed.
    // Invalid payload captures remove token-valid feedback muxes. The common
    // advance still freezes every stage under CS backpressure.
    if (freeInvalidPayload) {
      meta(0).os := os
      meta(0).context := ctx
      meta(0).tag := io.north.bits.tag
    }

    valid(0) := fire
    for (i <- 1 until 4) {
      valid(i) := valid(i - 1)
      if (freeInvalidPayload) {
        meta(i) := meta(i - 1)
      } else {
        when(valid(i - 1)) { meta(i) := meta(i - 1) }
      }
    }

    when(fire) {
      if (!freeInvalidPayload) {
        meta(0).os := os
        meta(0).context := ctx
        meta(0).tag := io.north.bits.tag
      }
      when(os) {
        assert(io.westOperand.valid && io.westOperand.bits.os)
        assert(io.westOperand.bits.context === ctx)
        assert(io.westOperand.bits.tag === io.north.bits.tag)
      }.otherwise {
        assert(loaded(readBank), "CS product before coefficient load")
      }
    }

    when(valid(0)) {
      when(!meta(0).os) {
        assert(io.westSum.valid && !io.westSum.bits.os, "late CS sum missing")
        assert(io.westSum.bits.context === meta(0).context, "late CS context mismatch")
        assert(io.westSum.bits.tag === meta(0).tag, "late CS tag mismatch")
      }
    }

    eastValid := valid(3) && !meta(3).os
    eastPayload.value := arithmeticResult
    eastPayload.context := meta(3).context
    eastPayload.tag := meta(3).tag
    eastPayload.os := false.B

    // Architectural state updates remain valid-qualified. Neither invalid
    // tokens nor free-running arithmetic payloads may alter C32.
    when(valid(3) && meta(3).os) {
      c32(meta(3).context) := arithmeticResult
    }
    for (i <- 0 until 4) {
      when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
      when(fire && os && ctx === i.U) {
        assert(cooldown(i) === 0.U, "C32 RAW")
        cooldown(i) := 3.U
      }
    }
  }

  // Seed/clear are permitted only after all tokens for that context retire.
  // The panel enforces ownership; these checks guard the physical PE boundary.
  for (i <- 0 until 4) {
    val busy = (0 until 4)
      .map { k =>
        valid(k) && meta(k).os && meta(k).context === i.U
      }
      .reduce(_ || _)
    when(io.seed.valid && io.seed.bits.context === i.U) {
      assert(!busy && !(fire && os && ctx === i.U))
      c32(i) := io.seed.bits.value
    }
    when(io.clear.valid && io.clear.bits === i.U) {
      assert(!busy && !(fire && os && ctx === i.U))
      c32(i) := 0.U
    }
  }
  io.accumulators := c32
}
