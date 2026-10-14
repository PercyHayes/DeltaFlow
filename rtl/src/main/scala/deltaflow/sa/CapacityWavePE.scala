package deltaflow.sa
import chisel3._
import chisel3.util._
import deltaflow.arithmetic._
class CapacityToken(width: Int, contexts: Int, tagBits: Int = 2) extends Bundle {
  val value = UInt(width.W)
  val context = UInt(math.max(1, log2Ceil(math.max(1, contexts))).W)
  val tag = UInt(tagBits.W)
  val os = Bool()
}
class CapacityValue(contexts: Int) extends Bundle {
  val value = UInt(32.W)
  val context = UInt(math.max(1, log2Ceil(math.max(1, contexts))).W)
}

/** Five-stage PE variants use identical arithmetic. 0/1 preserve command modes; 2/4 permit native
  * per-wave modes. No array controller is implied here.
  */
class CapacityWavePE(val contexts: Int = 4, tagBits: Int = 2) extends Module {
  require(Set(0, 1, 2, 4).contains(contexts) && tagBits >= 1 && tagBits <= 16)
  val pipelineStages = 5
  val perWave = contexts >= 2
  val slots = math.max(1, contexts)
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val commandOS = Input(Bool())
    val north = Flipped(Valid(new CapacityToken(16, contexts, tagBits)))
    val west = Flipped(Valid(new CapacityToken(32, contexts, tagBits)))
    val south = Valid(new CapacityToken(16, contexts, tagBits))
    val east = Valid(new CapacityToken(32, contexts, tagBits))
    val seed = Flipped(Valid(new CapacityValue(contexts)))
    val clear = Flipped(Valid(UInt(math.max(1, log2Ceil(slots)).W)))
    val coefficient = Flipped(Valid(UInt(16.W)))
    val accumulators = Output(Vec(slots, UInt(32.W)))
    val osBusy = Output(Vec(slots, Bool()))
    val csBusy = Output(Bool())
    val loaded = Output(Bool())
  })
  val c32 = if (contexts > 0) Some(RegInit(VecInit(Seq.fill(contexts)(0.U(32.W))))) else None
  val weight = RegInit(0.U(16.W))
  val loaded = RegInit(false.B)
  val south = RegInit(0.U.asTypeOf(Valid(new CapacityToken(16, contexts, tagBits))))
  val east = RegInit(0.U.asTypeOf(Valid(new CapacityToken(32, contexts, tagBits))))
  val os = if (contexts == 0) false.B else if (perWave) io.west.bits.os else io.commandOS
  val ctx = if (contexts >= 2) io.west.bits.context else 0.U
  val multiply = io.north.valid && io.west.valid
  val acc = c32.map(x => if (contexts >= 2) x(ctx) else x(0)).getOrElse(0.U(32.W))
  val pipelineBusy = Wire(Vec(slots, Bool()))
  pipelineBusy := VecInit(Seq.fill(slots)(false.B))
  val pipelineCS = WireDefault(false.B)
  when(io.coefficient.valid) {
    weight := io.coefficient.bits
    loaded := true.B
  }
  when(io.advance && multiply) {
    assert(io.north.bits.os === io.west.bits.os, "wave mode mismatch")
    assert(io.north.bits.context === io.west.bits.context, "wave context mismatch")
    assert(io.north.bits.tag === io.west.bits.tag, "wave tag mismatch")
    when(!os) { assert(loaded, "CS before coefficient load") }
  }
  if (pipelineStages == 1) {
    val mac = Module(new MixedFma)
    mac.io.a := Mux(os, io.west.bits.value(15, 0), weight)
    mac.io.b := io.north.bits.value
    mac.io.c := Mux(os, acc, io.west.bits.value)
    when(io.advance) {
      south := io.north
      east.bits := io.west.bits
      east.valid := Mux(os, io.west.valid, multiply)
      east.bits.value := Mux(os, Cat(0.U(16.W), io.west.bits.value(15, 0)), mac.io.result)
      c32.foreach { x =>
        when(multiply && os) {
          if (contexts >= 2) x(ctx) := mac.io.result else x(0) := mac.io.result
          assert(!(io.seed.valid && io.seed.bits.context === ctx), "OS/seed collision")
          assert(!(io.clear.valid && io.clear.bits === ctx), "OS/clear collision")
        }
      }
    }
  } else {
    // One registered vertical hop, five horizontal hops for both modes.
    // Boundary admission enforces four enabled cycles between same-C32 waves.
    val valid = RegInit(VecInit(Seq.fill(4)(false.B)))
    val tokens = Reg(Vec(4, new CapacityToken(32, contexts, tagBits)))
    val product = Module(new FmaProductStage)
    val align = Module(new FmaAlignStage)
    val add = Module(new FmaAddStage)
    val normalize = Module(new FmaNormalizeStage)
    val round = Module(new FmaRoundStage)
    val pr = Reg(new ExactProduct)
    val ar = Reg(new AlignedFma)
    val sr = Reg(new AddedFma)
    val nr = Reg(new easyfloat.RawFloat(11, 26))
    product.io.a := Mux(os, io.west.bits.value(15, 0), weight)
    product.io.b := io.north.bits.value
    align.io.p := pr
    align.io.c := Mux(
      tokens(0).os,
      c32.map(x => if (contexts >= 2) x(tokens(0).context) else x(0)).getOrElse(0.U(32.W)),
      tokens(0).value
    )
    add.io.in := ar
    normalize.io.in := sr
    round.io.in := nr
    val cooldown = RegInit(VecInit(Seq.fill(slots)(0.U(2.W))))
    when(io.advance) {
      south := io.north
      valid(0) := multiply
      for (i <- 1 until 4) {
        valid(i) := valid(i - 1)
        when(valid(i - 1)) { tokens(i) := tokens(i - 1) }
      }
      when(multiply) {
        pr := product.io.out
        tokens(0) := io.west.bits
        tokens(0).os := os
        tokens(0).context := ctx
      }
      when(valid(0)) { ar := align.io.out }
      when(valid(1)) { sr := add.io.out }
      when(valid(2)) { nr := normalize.io.out }
      east.valid := valid(3)
      east.bits := tokens(3)
      east.bits.value := Mux(tokens(3).os, Cat(0.U(16.W), tokens(3).value(15, 0)), round.io.out)
      c32.foreach(x =>
        when(valid(3) && tokens(3).os) {
          if (contexts >= 2) x(tokens(3).context) := round.io.out else x(0) := round.io.out
        }
      )
      for (i <- 0 until slots) {
        when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
        when(multiply && os && ctx === i.U) {
          assert(cooldown(i) === 0.U, "pipeline C32 RAW")
          cooldown(i) := 3.U
        }
      }
    }
    for (i <- 0 until slots) {
      pipelineBusy(i) := (0 until 4)
        .map(k => valid(k) && tokens(k).os && tokens(k).context === i.U)
        .reduce(_ || _)
      when(io.seed.valid && io.seed.bits.context === i.U) {
        assert(!pipelineBusy(i), "pipeline seed collision")
      }
      when(io.clear.valid && io.clear.bits === i.U) {
        assert(!pipelineBusy(i), "pipeline clear collision")
      }
    }
    pipelineCS := (0 until 4).map(k => valid(k) && !tokens(k).os).reduce(_ || _)
  }
  c32.foreach { x =>
    when(io.seed.valid) {
      if (contexts >= 2) x(io.seed.bits.context) := io.seed.bits.value
      else x(0) := io.seed.bits.value
    }
    when(io.clear.valid) { if (contexts >= 2) x(io.clear.bits) := 0.U else x(0) := 0.U }
  }
  val southMode = if (perWave) south.bits.os else if (contexts > 0) io.commandOS else false.B
  val eastMode = if (perWave) east.bits.os else if (contexts > 0) io.commandOS else false.B
  for (i <- 0 until slots) {
    io.accumulators(i) := (if (i < contexts) c32.get(i) else 0.U)
    io.osBusy(i) := (south.valid && southMode && south.bits.context === i.U) ||
      (east.valid && eastMode && east.bits.context === i.U) || pipelineBusy(i)
  }
  io.csBusy := (south.valid && !southMode) || (east.valid && !eastMode) || pipelineCS
  io.south := south
  io.east := east
  io.loaded := loaded
  if (!perWave) {
    io.south.bits.os := southMode
    io.east.bits.os := eastMode
  }
}
