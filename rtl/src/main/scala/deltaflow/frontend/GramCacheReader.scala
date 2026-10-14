package deltaflow.frontend
import chisel3._
import chisel3.util._

/** Sixteen future Gram blocks live in Operand addresses 640..703 at b16. Four independent bank
  * groups, two reserved rows each. No arithmetic and no full-matrix register copy. Each output
  * carries the original Gram job tag.
  */
class GramCacheReader(val side: Int, val full: Boolean = false) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val start = Input(Bool())
    val bankEnable = Input(UInt(16.W))
    val read = Vec(16, Decoupled(UInt(10.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val result = Vec(4, Decoupled(new GramRow(side)))
    val idle = Output(Bool())
    val released = Output(UInt(16.W))
  })
  val active = RegInit(false.B)
  val blocks = if (full) 5 else 4
  val phase = if (full) Some(RegInit(0.U(2.W))) else None
  val next = RegInit(VecInit(Seq.fill(4)(0.U(log2Ceil(blocks * side + 1).W))))
  val received = RegInit(VecInit(Seq.fill(4)(0.U(log2Ceil(blocks * side + 1).W))))
  val finished = Wire(Vec(4, Bool()))
  when(io.start) {
    assert(!active)
    active := true.B
    for (g <- 0 until 4) {
      next(g) := 0.U
      received(g) := 0.U
    }
  }
  if (full) {
    when(io.start) { phase.get := 0.U }
    // Drain normal rows before reusing their physical bank response paths.
    when(active && phase.get === 0.U && received.map(_ === (4 * side).U).reduce(_ && _)) {
      phase.get := 1.U
    }
    when(active && phase.get === 1.U && received.take(2).map(_ === (5 * side).U).reduce(_ && _)) {
      phase.get := 2.U
    }
  }
  for (b <- 0 until 16) {
    io.read(b).valid := false.B
    io.read(b).bits := 0.U
    io.response(b).ready := false.B
  }
  for (g <- 0 until 4) {
    val tags = Module(new Queue(UInt(log2Ceil(blocks * side).W), 2, pipe = false, flow = false))
    val rows = Module(new Queue(new GramRow(side), 2, pipe = false, flow = false))
    val reserved = RegInit(0.U(2.W))
    io.result(g) <> rows.io.deq
    val tail = phase.map(_ =/= 0.U).getOrElse(false.B)
    val enabled = phase.map(p => p === 0.U || p === (if (g < 2) 1 else 2).U).getOrElse(true.B)
    val banks = (0 until side / 4).map(i => 4 * g + i)
    val tailBanks = (0 until side / 4).map(i => 8 + 4 * (g % 2) + i)
    val selectedReady = Mux(
      tail,
      tailBanks.map(b => io.read(b).ready && io.bankEnable(b)).reduce(_ && _),
      banks.map(b => io.read(b).ready && io.bankEnable(b)).reduce(_ && _)
    )
    val limit = if (full) Mux(tail, (5 * side).U, (4 * side).U) else (4 * side).U
    val request = active && enabled && next(
      g
    ) < limit && tags.io.enq.ready && reserved +& rows.io.count < 2.U && selectedReady
    tags.io.enq.valid := request
    tags.io.enq.bits := next(g)
    for (b <- banks) when(enabled && !tail) {
      io.read(b).valid := request
      io.read(b).bits := 640.U + next(g)
    }
    if (full) for (b <- tailBanks) when(enabled && tail) {
      io.read(b).valid := request
      io.read(b).bits := (896 + (g / 2) * side).U + next(g)(log2Ceil(side) - 1, 0)
    }
    when(request) { next(g) := next(g) + 1.U }
    val returned = Mux(
      tail,
      tailBanks.map(b => io.response(b).valid).reduce(_ && _),
      banks.map(b => io.response(b).valid).reduce(_ && _)
    )
    rows.io.enq.valid := tags.io.deq.valid && returned
    rows.io.enq.bits.job := Cat(
      tags.io.deq.bits(log2Ceil(blocks * side) - 1, log2Ceil(side)),
      g.U(2.W)
    )
    rows.io.enq.bits.row := tags.io.deq.bits(log2Ceil(side) - 1, 0)
    for (i <- 0 until side)
      rows.io.enq.bits.values(i) := Mux(
        tail,
        io.response(8 + 4 * (g % 2) + i / 4).bits,
        io.response(4 * g + i / 4).bits
      )(32 * (i % 4) + 31, 32 * (i % 4))
    tags.io.deq.ready := returned && rows.io.enq.ready
    for (b <- banks) when(enabled && !tail) {
      io.response(b).ready := tags.io.deq.valid && returned && rows.io.enq.ready
    }
    if (full) for (b <- tailBanks) when(enabled && tail) {
      io.response(b).ready := tags.io.deq.valid && returned && rows.io.enq.ready
    }
    when(request =/= rows.io.enq.fire) { reserved := Mux(request, reserved + 1.U, reserved - 1.U) }
    when(rows.io.enq.valid) { assert(rows.io.enq.ready && reserved =/= 0.U) }
    when(io.result(g).fire) { received(g) := received(g) + 1.U }
    finished(g) := received(g) === (blocks * side).U
  }
  when(active && finished.asUInt.andR) { active := false.B }
  // A consumed final normal row proves all SRAM responses for that bank
  // group have retired. Full-cache tails use high banks only; low banks can
  // pass to a future RHS reader independently of the other groups.
  io.released := VecInit(
    (0 until 16).map(b =>
      !active ||
        (if (b < 8) {
           if (b % 4 < side / 4) received(b / 4) >= (4 * side).U else true.B
         } else false.B)
    )
  ).asUInt
  io.idle := !active
}
