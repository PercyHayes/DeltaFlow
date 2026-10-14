package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic._

class WaveToken(width: Int, tagBits: Int) extends ContextToken(width, tagBits) {
  val os = Bool()
}
class WaveCoefficient(side: Int) extends Bundle {
  val column = Bool()
  val index = UInt(log2Ceil(side).W)
  val values = Vec(side, UInt(16.W))
}

/** One MAC, optional local C32, mode carried on both registered wave paths. contexts=0/1 are frozen
  * CS / command-mode CS+OS comparison implementations.
  */
class MixedWavePE(
    contexts: Int = 2,
    tagBits: Int = 2,
    perWave: Boolean = true,
    val pipelineStages: Int = 1
) extends Module {
  require(contexts >= 0 && contexts <= 2 && tagBits >= 1 && tagBits <= 16)
  require(pipelineStages == 1 || pipelineStages == 5)
  require(pipelineStages == 1 || (contexts == 2 && perWave))
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val commandOS = Input(Bool())
    val north = Flipped(Valid(new WaveToken(16, tagBits)))
    val west = Flipped(Valid(new WaveToken(32, tagBits)))
    val south = Valid(new WaveToken(16, tagBits))
    val east = Valid(new WaveToken(32, tagBits))
    val seed = Flipped(Valid(new ContextValue))
    val clear = Flipped(Valid(Bool()))
    val coefficient = Flipped(Valid(UInt(16.W)))
    val accumulators = Output(Vec(2, UInt(32.W)))
    val osBusy = Output(Vec(2, Bool()))
    val csBusy = Output(Bool())
    val loaded = Output(Bool())
  })
  val c32 = if (contexts > 0) Some(RegInit(VecInit(Seq.fill(contexts)(0.U(32.W))))) else None
  val weight = RegInit(0.U(16.W))
  val loaded = RegInit(false.B)
  val south = RegInit(0.U.asTypeOf(Valid(new WaveToken(16, tagBits))))
  val east = RegInit(0.U.asTypeOf(Valid(new WaveToken(32, tagBits))))
  val os = if (contexts == 0) false.B else if (perWave) io.west.bits.os else io.commandOS
  val ctx = if (contexts == 2) io.west.bits.context else false.B
  val multiply = io.north.valid && io.west.valid
  val acc = c32.map(x => if (contexts == 2) x(ctx) else x(0)).getOrElse(0.U(32.W))
  val pipelineBusy = Wire(Vec(2, Bool()))
  pipelineBusy := VecInit(Seq.fill(2)(false.B))
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
          if (contexts == 2) x(ctx) := mac.io.result else x(0) := mac.io.result
          assert(!(io.seed.valid && io.seed.bits.context === ctx), "OS/seed collision")
          assert(!(io.clear.valid && io.clear.bits === ctx), "OS/clear collision")
        }
      }
    }
  } else {
    // One registered vertical hop, five horizontal hops for both modes.
    // Boundary admission enforces four enabled cycles between same-C32 waves.
    val valid = RegInit(VecInit(Seq.fill(4)(false.B)))
    val tokens = Reg(Vec(4, new WaveToken(32, tagBits)))
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
    align.io.c := Mux(tokens(0).os, c32.get(tokens(0).context), tokens(0).value)
    add.io.in := ar
    normalize.io.in := sr
    round.io.in := nr
    val cooldown = RegInit(VecInit(Seq.fill(2)(0.U(2.W))))
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
      }
      when(valid(0)) { ar := align.io.out }
      when(valid(1)) { sr := add.io.out }
      when(valid(2)) { nr := normalize.io.out }
      east.valid := valid(3)
      east.bits := tokens(3)
      east.bits.value := Mux(tokens(3).os, Cat(0.U(16.W), tokens(3).value(15, 0)), round.io.out)
      when(valid(3) && tokens(3).os) { c32.get(tokens(3).context) := round.io.out }
      for (i <- 0 until 2) {
        when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
        when(multiply && os && ctx === i.U) {
          assert(cooldown(i) === 0.U, "pipeline C32 RAW")
          cooldown(i) := 3.U
        }
      }
    }
    for (i <- 0 until 2) {
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
      if (contexts == 2) x(io.seed.bits.context) := io.seed.bits.value
      else x(0) := io.seed.bits.value
    }
    when(io.clear.valid) { if (contexts == 2) x(io.clear.bits) := 0.U else x(0) := 0.U }
  }
  val southMode = if (perWave) south.bits.os else if (contexts > 0) io.commandOS else false.B
  val eastMode = if (perWave) east.bits.os else if (contexts > 0) io.commandOS else false.B
  for (i <- 0 until 2) {
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

/** Raw skewed boundary, one enable freezes mode/value/context/tag coherently. CS coefficient writes
  * ignore OS readers; C32 access ignores CS traffic. Response queue is two rows of flip-flops, not
  * an SRAM blackbox.
  */
class MixedWaveMicroTile(
    val side: Int,
    val contexts: Int = 2,
    val perWave: Boolean = true,
    tagBits: Int = 2,
    val pipelineStages: Int = 1
) extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  require(contexts >= 0 && contexts <= 2 && (!perWave || contexts == 2))
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val commandOS = Input(Bool())
    val north = Flipped(Vec(side, Valid(new WaveToken(16, tagBits))))
    val west = Flipped(Vec(side, Valid(new WaveToken(32, tagBits))))
    val south = Vec(side, Valid(new WaveToken(16, tagBits)))
    val east = Vec(side, Valid(new WaveToken(32, tagBits)))
    val coefficient = Flipped(Decoupled(new WaveCoefficient(side)))
    val seed = Flipped(Decoupled(new ContextRowData(side)))
    val clear = Flipped(Decoupled(Bool()))
    val read = Flipped(Decoupled(new ContextRow(side)))
    val response = Decoupled(new ContextRowData(side))
    val contextIdle = Output(Vec(2, Bool()))
    val csIdle = Output(Bool())
    val idle = Output(Bool())
    val coefficientsReady = Output(Bool())
  })
  val pe = Seq.fill(side, side)(Module(new MixedWavePE(contexts, tagBits, perWave, pipelineStages)))
  val north = Wire(chiselTypeOf(io.north))
  val west = Wire(chiselTypeOf(io.west))
  north := io.north
  west := io.west
  for (i <- 0 until side) {
    if (!perWave) {
      north(i).bits.os := (if (contexts > 0) io.commandOS else false.B)
      west(i).bits.os := (if (contexts > 0) io.commandOS else false.B)
    }
    if (contexts < 2) {
      north(i).bits.context := false.B
      west(i).bits.context := false.B
    }
  }
  for (i <- 0 until 2) {
    val incoming =
      (north.map(x => x.valid && x.bits.os && x.bits.context === i.U) ++
        west.map(x => x.valid && x.bits.os && x.bits.context === i.U)).reduce(_ || _)
    io.contextIdle(i) := !incoming && !pe.flatten.map(_.io.osBusy(i)).reduce(_ || _)
  }
  val csIncoming = (north.map(x => x.valid && !x.bits.os) ++ west.map(x => x.valid && !x.bits.os))
    .reduce(_ || _)
  io.csIdle := !csIncoming && !pe.flatten.map(_.io.csBusy).reduce(_ || _)
  io.idle := io.csIdle && io.contextIdle.asUInt.andR
  if (!perWave && contexts > 0) {
    val previous = RegNext(io.commandOS, false.B)
    when(previous =/= io.commandOS) { assert(io.idle, "baseline command mode changed while busy") }
  }
  io.coefficient.ready := (if (perWave) io.csIdle else io.idle)
  if (!perWave) {
    when(io.coefficient.valid) {
      assert(!io.coefficient.bits.column, "baseline has row coefficient load only")
    }
  }
  val seedFire = WireDefault(false.B)
  val clearFire = WireDefault(false.B)
  if (contexts > 0) {
    val queue = Module(new Queue(new ContextRowData(side), 2, pipe = false, flow = false))
    val readContext = if (contexts == 2) io.read.bits.context else false.B
    val seedContext = if (contexts == 2) io.seed.bits.context else false.B
    val clearContext = if (contexts == 2) io.clear.bits else false.B
    io.read.ready := io.contextIdle(readContext) && queue.io.enq.ready
    queue.io.enq.valid := io.read.valid && io.contextIdle(readContext)
    queue.io.enq.bits.context := readContext
    queue.io.enq.bits.row := io.read.bits.row
    for (col <- 0 until side)
      queue.io.enq.bits.values(col) := VecInit(
        (0 until side).map(r => pe(r)(col).io.accumulators(readContext))
      )(io.read.bits.row)
    io.response <> queue.io.deq
    io.seed.ready := io.contextIdle(seedContext) && !io.read.valid && !io.clear.valid
    io.clear.ready := io.contextIdle(clearContext) && !io.read.valid
    seedFire := io.seed.fire
    clearFire := io.clear.fire
    when(io.read.fire) { assert(io.read.bits.row < side.U) }
    when(io.seed.fire) { assert(io.seed.bits.row < side.U) }
    if (contexts == 1) {
      when(io.read.valid) { assert(!io.read.bits.context) }
      when(io.seed.valid) { assert(!io.seed.bits.context) }
      when(io.clear.valid) { assert(!io.clear.bits) }
    }
  } else {
    io.read.ready := false.B
    io.seed.ready := false.B
    io.clear.ready := false.B
    io.response.valid := false.B
    io.response.bits := 0.U.asTypeOf(io.response.bits)
  }
  val coefficientMask = RegInit(0.U(side.W))
  val loading = RegInit(false.B)
  val loadColumn = RegInit(false.B)
  when(io.coefficient.fire) {
    assert(io.coefficient.bits.index < side.U)
    val bit = (1.U(side.W) << io.coefficient.bits.index)(side - 1, 0)
    val next = Mux(loading, coefficientMask | bit, bit)
    when(loading) {
      assert(!(coefficientMask & bit).orR, "duplicate coefficient vector")
      assert(loadColumn === io.coefficient.bits.column, "coefficient layout changed within frame")
    }.otherwise { loadColumn := io.coefficient.bits.column }
    coefficientMask := next
    loading := !next.andR
  }
  for (r <- 0 until side; c <- 0 until side) {
    val p = pe(r)(c)
    p.io.advance := io.advance
    p.io.commandOS := io.commandOS
    p.io.north := (if (r == 0) north(c) else pe(r - 1)(c).io.south)
    p.io.west := (if (c == 0) west(r) else pe(r)(c - 1).io.east)
    p.io.seed.valid := seedFire && io.seed.bits.row === r.U
    p.io.seed.bits.context := (if (contexts == 2) io.seed.bits.context else false.B)
    p.io.seed.bits.value := io.seed.bits.values(c)
    p.io.clear.valid := clearFire
    p.io.clear.bits := (if (contexts == 2) io.clear.bits else false.B)
    val column = if (perWave) io.coefficient.bits.column else false.B
    p.io.coefficient.valid := io.coefficient.fire && Mux(
      column,
      io.coefficient.bits.index === c.U,
      io.coefficient.bits.index === r.U
    )
    p.io.coefficient.bits := Mux(
      column,
      io.coefficient.bits.values(r),
      io.coefficient.bits.values(c)
    )
  }
  for (i <- 0 until side) {
    io.south(i) := pe(side - 1)(i).io.south
    io.east(i) := pe(i)(side - 1).io.east
  }
  io.coefficientsReady := coefficientMask.andR && !loading &&
    pe.flatten
      .map(_.io.loaded)
      .reduce(_ && _)
}
