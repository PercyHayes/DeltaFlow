package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.sa.PublishedVector
import deltaflow.arithmetic.FloatArithmetic

class QueryVector(side: Int) extends Bundle {
  val column = UInt(8.W)
  val values = Vec(4 * side, UInt(16.W))
}
class QueryCoefficientWrite(side: Int) extends ProjectionColumn(side) {
  val group = UInt(2.W)
}

/** Q -> borrowed preparation multipliers -> BF16 Qbar in Operand SRAM. Reserve a complete raw
  * vector before any bank grant. The fetch register holds its address until every bank accepts; no
  * descriptor or data FIFO is needed here. Each bank's valid is independent of other banks'
  * readiness. Two raw vectors and the original shared scale/rounding lanes are retained.
  */
class QueryCoefficientController(
    val side: Int = 16,
    val reserveBeforeRead: Boolean = true,
    val registerFetchEnd: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  val words = (side + 7) / 8
  val ports = 4 * words until 8 * words
  val io = IO(new Bundle {
    val start = Input(Valid(UInt(8.W)))
    val available = Input(UInt(16.W))
    val bankEnable = Input(UInt(16.W))
    val read = Vec(16, Decoupled(UInt(9.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val scale = Vec(4, Decoupled(new ProjectionColumn(side)))
    val scaled = Flipped(Vec(4, Decoupled(new PublishedVector(side))))
    val write = Decoupled(new QueryCoefficientWrite(side))
    val idle = Output(Bool())
    val allocated = Output(SInt(10.W))
    val captured = Output(SInt(10.W))
  })

  val active = RegInit(false.B)
  val limit = Reg(UInt(8.W))
  val fetch = RegInit(0.U(8.W))
  // Terminal state changes on the same edge as the final column increment.
  // It removes a variable-width comparison from every bank's admission path.
  val fetchEnded = if (registerFetchEnd) Some(RegInit(false.B)) else None
  val reserved = RegInit(0.U(2.W))
  val written = RegInit(0.U(10.W))
  val group = RegInit(0.U(2.W))
  val granted = RegInit(0.U(ports.size.W))
  val tags = Module(new Queue(UInt(8.W), 2, pipe = false, flow = false))
  val raw = Module(new Queue(new QueryVector(side), 2, pipe = false, flow = false))
  val out = Module(new Queue(new QueryCoefficientWrite(side), 2, pipe = false, flow = false))
  io.write <> out.io.deq
  io.idle := !active

  when(io.start.valid) {
    assert(!active && reserved === 0.U && raw.io.count === 0.U && out.io.count === 0.U)
    assert(!granted.orR && io.start.bits > 0.U && io.start.bits <= 128.U)
    active := true.B
    limit := io.start.bits
    fetch := 0.U
    fetchEnded.foreach(_ := false.B)
    written := 0.U
    group := 0.U
  }

  val partial = if (reserveBeforeRead) RegInit(false.B) else granted.orR
  val room = (reserved +& raw.io.count) < 2.U && tags.io.enq.ready
  val fetchAvailable = fetchEnded.map(x => !x).getOrElse(fetch < limit)
  val offer = active && fetchAvailable && (partial || room)
  fetchEnded.foreach { ended =>
    when(active) {
      assert(ended === (fetch >= limit), "query terminal state must match its cursor")
    }
  }
  for (b <- 0 until 16) {
    io.read(b).valid := false.B
    io.read(b).bits := fetch
    io.response(b).ready := false.B
  }
  for ((b, i) <- ports.zipWithIndex) {
    io.read(b).valid := offer && !granted(i) && io.available(b) && io.bankEnable(b)
  }
  val acceptedBanks = VecInit(ports.map(b => io.read(b).fire)).asUInt
  // Original two-vector capacity and tag FIFO own the entire return before
  // reading any bank. No SRAM-ready/collision path feeds the tag write enable.
  val accept = if (reserveBeforeRead) offer && !partial else !partial && acceptedBanks.orR
  val complete = offer && (granted | acceptedBanks).andR
  if (reserveBeforeRead) {
    when(offer) { partial := !complete }
  }
  when(acceptedBanks.orR) {
    granted := Mux(complete, 0.U, granted | acceptedBanks)
  }
  when(complete) {
    fetch := fetch + 1.U
    fetchEnded.foreach(_ := (fetch +& 1.U) === limit)
  }
  when(partial) { assert(reserved =/= 0.U && active, "partial query lost its reservation") }
  assert(!(granted & acceptedBanks).orR, "query bank accepted twice")

  tags.io.enq.valid := accept
  tags.io.enq.bits := fetch
  val returned = ports.map(b => io.response(b).valid).reduce(_ && _)
  raw.io.enq.valid := tags.io.deq.valid && returned
  raw.io.enq.bits.column := tags.io.deq.bits
  for (g <- 0 until 4; r <- 0 until side) {
    val bank = 4 * words + g * words + r / 8
    raw.io.enq.bits.values(g * side + r) := io.response(bank).bits(16 * (r % 8) + 15, 16 * (r % 8))
  }
  tags.io.deq.ready := returned && raw.io.enq.ready
  for (b <- ports) {
    io.response(b).ready := tags.io.deq.valid && returned && raw.io.enq.ready
  }
  when(accept =/= raw.io.enq.fire) {
    reserved := Mux(accept, reserved + 1.U, reserved - 1.U)
  }
  when(accept) { assert(tags.io.enq.ready) }
  when(raw.io.enq.valid) { assert(raw.io.enq.ready && reserved =/= 0.U) }
  assert((reserved +& raw.io.count) <= 2.U, "query vector capacity exceeded")
  io.allocated := Mux(accept, fetch.zext, (-1).S)
  io.captured := Mux(raw.io.enq.fire, tags.io.deq.bits.zext, (-1).S)

  for (g <- 0 until 4) {
    io.scale(g).valid := active && raw.io.deq.valid && group === g.U
    io.scale(g).bits.column := raw.io.deq.bits.column
    for (r <- 0 until side) io.scale(g).bits.values(r) := raw.io.deq.bits.values(g * side + r)
  }
  val sent = io.scale.map(_.fire).reduce(_ || _)
  raw.io.deq.ready := sent && group === 3.U
  when(sent) { group := group + 1.U }

  val arb = Module(new RRArbiter(new PublishedVector(side), 4))
  for (g <- 0 until 4) arb.io.in(g) <> io.scaled(g)
  out.io.enq.valid := arb.io.out.valid
  arb.io.out.ready := out.io.enq.ready
  out.io.enq.bits.column := arb.io.out.bits.key
  out.io.enq.bits.group := arb.io.chosen
  for (r <- 0 until side)
    out.io.enq.bits.values(r) := FloatArithmetic.narrow(arb.io.out.bits.values(r))
  when(io.write.fire) { written := written + 1.U }
  when(active && written === (limit << 2)) {
    assert(reserved === 0.U && raw.io.count === 0.U && !partial)
    active := false.B
  }
}
