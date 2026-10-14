package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic.MixedFma

class ContextToken(width: Int, tagBits: Int) extends Bundle {
  val value = UInt(width.W)
  val context = Bool()
  val tag = UInt(tagBits.W)
}
class ContextValue extends Bundle {
  val context = Bool()
  val value = UInt(32.W)
}
class ContextRow(side: Int) extends Bundle {
  val context = Bool()
  val row = UInt(log2Ceil(side).W)
}
class ContextRowData(side: Int) extends ContextRow(side) {
  val values = Vec(side, UInt(32.W))
}

/** One shared MAC, two stationary FP32 values, separately tagged link state. Default tags are
  * two-bit alignment checks; full panel IDs stay at boundaries.
  */
class DualContextPE(tagBits: Int = 2) extends Module {
  require(tagBits >= 1 && tagBits <= 32)
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val outputStationary = Input(Bool())
    val north = Flipped(Valid(new ContextToken(16, tagBits)))
    val west = Flipped(Valid(new ContextToken(32, tagBits)))
    val south = Valid(new ContextToken(16, tagBits))
    val east = Valid(new ContextToken(32, tagBits))
    val seed = Flipped(Valid(new ContextValue))
    val clear = Flipped(Valid(Bool()))
    val coefficient = Flipped(Valid(UInt(16.W)))
    val accumulators = Output(Vec(2, UInt(32.W)))
    val inFlight = Output(Vec(2, Bool()))
    val loaded = Output(Bool())
  })
  val accumulators = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))
  val weight = RegInit(0.U(16.W))
  val loaded = RegInit(false.B)
  val south = RegInit(0.U.asTypeOf(Valid(new ContextToken(16, tagBits))))
  val east = RegInit(0.U.asTypeOf(Valid(new ContextToken(32, tagBits))))
  val mac = Module(new MixedFma)
  val multiply = io.north.valid && io.west.valid
  val context = io.west.bits.context
  mac.io.a := Mux(io.outputStationary, io.west.bits.value(15, 0), weight)
  mac.io.b := io.north.bits.value
  mac.io.c := Mux(io.outputStationary, accumulators(context), io.west.bits.value)
  when(io.coefficient.valid) {
    weight := io.coefficient.bits
    loaded := true.B
  }
  when(io.advance) {
    south := io.north
    east.valid := Mux(io.outputStationary, io.west.valid, multiply)
    east.bits := io.west.bits
    east.bits.value := Mux(
      io.outputStationary,
      Cat(0.U(16.W), io.west.bits.value(15, 0)),
      mac.io.result
    )
    when(multiply) {
      assert(io.north.bits.context === context, "context misalignment")
      assert(io.north.bits.tag === io.west.bits.tag, "reduction tag misalignment")
      when(io.outputStationary) {
        accumulators(context) := mac.io.result
        assert(!(io.seed.valid && io.seed.bits.context === context), "seed/MAC collision")
        assert(!(io.clear.valid && io.clear.bits === context), "clear/MAC collision")
      }.otherwise { assert(loaded, "uninitialized CS coefficient") }
    }
  }
  when(io.seed.valid) { accumulators(io.seed.bits.context) := io.seed.bits.value }
  when(io.clear.valid) { accumulators(io.clear.bits) := 0.U }
  assert(!(io.seed.valid && io.clear.valid), "seed/clear collision")
  for (i <- 0 until 2) {
    io.inFlight(i) :=
      (south.valid && south.bits.context === i.U) || (east.valid && east.bits.context === i.U)
  }
  io.accumulators := accumulators
  io.south := south
  io.east := east
  io.loaded := loaded
}

/** Independently owned C0/C1, coherent wave stall, bounded snapshot response. Seed/clear/read
  * require the selected context to drain. The other context can keep computing. FP32 readout feeds
  * shared conversion units downstream.
  */
class DualContextMicroTile(val side: Int, tagBits: Int = 2) extends Module {
  require(side >= 2 && side <= 16 && isPow2(side))
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val outputStationary = Input(Bool())
    val north = Flipped(Vec(side, Valid(new ContextToken(16, tagBits))))
    val west = Flipped(Vec(side, Valid(new ContextToken(32, tagBits))))
    val south = Vec(side, Valid(new ContextToken(16, tagBits)))
    val east = Vec(side, Valid(new ContextToken(32, tagBits)))
    val seed = Flipped(Decoupled(new ContextRowData(side)))
    val clear = Flipped(Decoupled(Bool()))
    val read = Flipped(Decoupled(new ContextRow(side)))
    val response = Decoupled(new ContextRowData(side))
    val coefficient = Flipped(Decoupled(new CoefficientWrite(side)))
    val contextIdle = Output(Vec(2, Bool()))
    val idle = Output(Bool())
    val coefficientsReady = Output(Bool())
  })
  val pe = Seq.fill(side, side)(Module(new DualContextPE(tagBits)))
  val queue = Module(new Queue(new ContextRowData(side), 2, pipe = false, flow = false))
  for (i <- 0 until 2) {
    val internal = pe.flatten.map(_.io.inFlight(i)).reduce(_ || _)
    val incoming =
      (io.north.map(x => x.valid && x.bits.context === i.U) ++
        io.west.map(x => x.valid && x.bits.context === i.U)).reduce(_ || _)
    io.contextIdle(i) := !internal && !incoming
  }
  io.idle := io.contextIdle.asUInt.andR
  val previousMode = RegNext(io.outputStationary, true.B)
  when(previousMode =/= io.outputStationary) { assert(io.idle, "dataflow mode change in flight") }
  io.read.ready := io.contextIdle(io.read.bits.context) && queue.io.enq.ready
  queue.io.enq.valid := io.read.valid && io.contextIdle(io.read.bits.context)
  queue.io.enq.bits.context := io.read.bits.context
  queue.io.enq.bits.row := io.read.bits.row
  for (col <- 0 until side) {
    queue.io.enq.bits.values(col) := VecInit(
      (0 until side).map(row => pe(row)(col).io.accumulators(io.read.bits.context))
    )(io.read.bits.row)
  }
  io.response <> queue.io.deq
  io.clear.ready := io.contextIdle(io.clear.bits) && !io.read.valid
  io.seed.ready := io.contextIdle(io.seed.bits.context) && !io.read.valid && !io.clear.valid
  io.coefficient.ready := io.idle
  when(io.seed.fire) { assert(io.seed.bits.row < side.U) }
  when(io.read.fire) { assert(io.read.bits.row < side.U) }
  when(io.coefficient.fire) {
    assert(io.coefficient.bits.row < side.U && io.coefficient.bits.col < side.U)
  }
  for (row <- 0 until side; col <- 0 until side) {
    val p = pe(row)(col)
    p.io.advance := io.advance
    p.io.outputStationary := io.outputStationary
    p.io.north := (if (row == 0) io.north(col) else pe(row - 1)(col).io.south)
    p.io.west := (if (col == 0) io.west(row) else pe(row)(col - 1).io.east)
    p.io.seed.valid := io.seed.fire && io.seed.bits.row === row.U
    p.io.seed.bits.context := io.seed.bits.context
    p.io.seed.bits.value := io.seed.bits.values(col)
    p.io.clear.valid := io.clear.fire
    p.io.clear.bits := io.clear.bits
    p.io.coefficient.valid :=
      io.coefficient.fire && io.coefficient.bits.row === row.U && io.coefficient.bits.col === col.U
    p.io.coefficient.bits := io.coefficient.bits.value
  }
  for (i <- 0 until side) {
    io.south(i) := pe(side - 1)(i).io.south
    io.east(i) := pe(i)(side - 1).io.east
  }
  io.coefficientsReady := pe.flatten.map(_.io.loaded).reduce(_ && _)
}
