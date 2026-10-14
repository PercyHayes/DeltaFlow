package deltaflow.sa
import chisel3._
import chisel3.util._
import deltaflow.arithmetic.FloatArithmetic

class CompletionTicket(depth: Int) extends Bundle {
  val slot = UInt(log2Ceil(depth).W)
  val key = UInt(16.W)
}
class PublishedVector(side: Int) extends Bundle {
  val key = UInt(16.W)
  val values = Vec(side, UInt(32.W))
}

/** Ordered reservations/returns, one shared payload read, registered multicast. Tickets are
  * (physical slot, monotonic 16-bit key). Keys may wrap only when all reservations and captured
  * outputs drain. Payload registers are not reset. onReturn uses the same port as an atomic return
  * allocation, with no admission reservation; its caller must provide an elastic arithmetic
  * pipeline. Compact all-BF16 payloads narrow exactly once at return, retaining the original RNE.
  * Consumers 0..2 receive BF16 in the low bits; consumer 3 receives FP32. All queues/register
  * payloads are logic resources, not SRAM blackboxes.
  */
class CompletionPublisher(
    val side: Int = 16,
    val depth: Int = 65,
    val reuseConsumedCredit: Boolean = true,
    val bf16Mask: Int = 7,
    val onReturn: Boolean = false,
    val compactBF16: Boolean = false,
    val eagerReturn: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(depth >= 2 && depth <= 65)
  require(bf16Mask >= 0 && bf16Mask < 16)
  require(!compactBF16 || (onReturn && bf16Mask == 15))
  require(!eagerReturn || onReturn)
  val io = IO(new Bundle {
    val reserve = Flipped(Decoupled(UInt(4.W)))
    val ticket = Output(new CompletionTicket(depth))
    val returned = Flipped(Decoupled(Vec(side, UInt(32.W))))
    val expectedKey = Output(UInt(16.W))
    val request = Flipped(Vec(4, Decoupled(new CompletionTicket(depth))))
    val response = Vec(4, Decoupled(new PublishedVector(side)))
    val occupied = Output(UInt(log2Ceil((if (eagerReturn) 4 * depth else depth) + 1).W))
    val idle = Output(Bool())
    val captured = Output(UInt(4.W))
    val selected = Valid(new CompletionTicket(depth))
  })
  if (eagerReturn) {
    // Each fixed graph edge owns a small FIFO. Atomic fanout on actual return
    // avoids revisiting a shared payload at different successor readiness times.
    // Ready uses only registered FIFO counts; no consumer-ready propagation.
    val bits = if (compactBF16) 16 else 32
    class Stored extends Bundle {
      val ticket = new CompletionTicket(depth)
      val values = Vec(side, UInt(bits.W))
    }
    val queues = Seq.fill(4)(Module(new Queue(new Stored, depth, pipe = false, flow = false)))
    val responses =
      Seq.fill(4)(Module(new Queue(new PublishedVector(side), 2, pipe = false, flow = false)))
    val tail = RegInit(0.U(log2Ceil(depth).W))
    val nextKey = RegInit(0.U(16.W))
    val wrap = RegInit(false.B)
    val empty = queues.map(_.io.count === 0.U).reduce(_ && _) &&
      responses
        .map(_.io.count === 0.U)
        .reduce(_ && _)
    val capacity = (0 until 4)
      .map(c => !io.reserve.bits(c) || queues(c).io.enq.ready)
      .reduce(_ && _)
    io.reserve.ready := capacity && io.reserve.bits.orR && (!wrap || empty)
    io.returned.ready := io.reserve.valid && io.reserve.ready
    io.ticket.slot := tail
    io.ticket.key := nextKey
    io.expectedKey := nextKey
    io.idle := empty
    // Occupancy means copies across edge FIFOs in this mode, not unique vectors.
    io.occupied := queues.map(_.io.count).reduce(_ +& _)
    val captured = Wire(Vec(4, Bool()))
    val stored =
      if (compactBF16) VecInit(io.returned.bits.map(FloatArithmetic.narrow(_)))
      else io.returned.bits
    when(io.returned.fire) {
      tail := Mux(tail === (depth - 1).U, 0.U, tail + 1.U)
      nextKey := nextKey + 1.U
      wrap := nextKey.andR
    }
    when(io.reserve.fire) { assert(io.returned.fire) }
    for (c <- 0 until 4) {
      val q = queues(c)
      val response = responses(c)
      val request = io.request(c)
      q.io.enq.valid := io.returned.fire && io.reserve.bits(c)
      q.io.enq.bits.ticket := io.ticket
      q.io.enq.bits.values := stored
      val hit = q.io.deq.valid && q.io.deq.bits.ticket.asUInt === request.bits.asUInt
      request.ready := hit && response.io.enq.ready
      q.io.deq.ready := request.valid && hit && response.io.enq.ready
      response.io.enq.valid := request.valid && hit
      response.io.enq.bits.key := q.io.deq.bits.ticket.key
      for (k <- 0 until side)
        response.io.enq.bits.values(k) :=
          (if (compactBF16 || (bf16Mask & (1 << c)) == 0)
             q.io.deq.bits.values(k)
           else FloatArithmetic.narrow(q.io.deq.bits.values(k)))
      io.response(c) <> response.io.deq
      captured(c) := request.fire
    }
    io.captured := captured.asUInt
    // This legacy observation reports one of potentially several parallel reads.
    io.selected.valid := captured.asUInt.orR
    io.selected.bits := Mux1H(PriorityEncoderOH(captured.asUInt), io.request.map(_.bits))
  } else {
    val live = RegInit(VecInit(Seq.fill(depth)(false.B)))
    val available = RegInit(VecInit(Seq.fill(depth)(false.B)))
    val pending = Reg(Vec(depth, UInt(4.W)))
    val keys = Reg(Vec(depth, UInt(16.W)))
    val data = Reg(Vec(depth, Vec(side, UInt((if (compactBF16) 16 else 32).W))))
    val tail = RegInit(0.U(log2Ceil(depth).W))
    val returning = RegInit(0.U(log2Ceil(depth).W))
    val nextKey = RegInit(0.U(16.W))
    val wrap = RegInit(false.B)
    def increment(x: UInt): UInt = Mux(x === (depth - 1).U, 0.U, x + 1.U)
    // Selection is registered before the wide payload mux/converters. Two
    // response credits per consumer cover this in-flight read and queued copies.
    val responses =
      Seq.fill(4)(Module(new Queue(new PublishedVector(side), 2, pipe = true, flow = false)))
    val readValid = RegInit(false.B)
    val readTicket = Reg(new CompletionTicket(depth))
    val readMask = Reg(UInt(4.W))
    val empty = !live.asUInt.orR && !readValid && responses.map(_.io.count === 0.U).reduce(_ && _)
    io.idle := empty
    io.occupied := PopCount(live)
    io.reserve.ready := !live(tail) && io.reserve.bits.orR && (!wrap || empty)
    io.ticket.slot := tail
    io.ticket.key := nextKey
    io.returned.ready :=
      (if (onReturn) io.reserve.valid && io.reserve.ready
       else live(returning) && !available(returning))
    io.expectedKey := (if (onReturn) nextKey else keys(returning))
    when(io.reserve.fire) {
      live(tail) := true.B
      available(tail) := onReturn.B
      pending(tail) := io.reserve.bits
      keys(tail) := nextKey
      if (onReturn) {
        assert(io.returned.fire, "return queue may only allocate an actual returned wave")
        for (k <- 0 until side)
          data(tail)(k) :=
            (if (compactBF16) FloatArithmetic.narrow(io.returned.bits(k))
             else io.returned.bits(k))
      }
      tail := increment(tail)
      nextKey := nextKey + 1.U
      wrap := nextKey.andR
    }
    if (!onReturn) when(io.returned.fire) {
      data(returning) := io.returned.bits
      available(returning) := true.B
      returning := increment(returning)
    }
    // Arbitration sees only four metadata lookups. One selected row feeds all
    // matching consumers; the 65-row payload mux and converters are shared.
    val arb = Module(new RRArbiter(new CompletionTicket(depth), 4))
    for (c <- 0 until 4) {
      val q = io.request(c)
      val slot = q.bits.slot
      // Match version alongside address decoding, before reducing the match.
      // This avoids serializing a 65:1 key mux and a wide equality comparator.
      val hit = (0 until depth)
        .map(i =>
          slot === i.U && live(i) && available(i) && pending(i)(c) && keys(i) === q.bits.key
        )
        .reduce(_ || _)
      val occupancy = responses(c).io.count +& (readValid && readMask(c)).asUInt
      arb.io.in(c).valid := q.valid && hit &&
        (occupancy < 2.U ||
          (if (reuseConsumedCredit)
             responses(c).io.deq.fire
           else false.B))
      arb.io.in(c).bits := q.bits
      io.response(c) <> responses(c).io.deq
    }
    arb.io.out.ready := true.B
    val selected = arb.io.out.bits.slot
    val take = Wire(Vec(4, Bool()))
    // Decode once for the wide shared read instead of a cascaded binary mux.
    val row = Mux1H((0 until depth).map(i => (readTicket.slot === i.U) -> data(i)))
    val narrowed = if (compactBF16) row else VecInit(row.map(FloatArithmetic.narrow(_)))
    for (c <- 0 until 4) {
      take(c) := arb.io.out.valid &&
        arb.io
          .in(c)
          .valid && io.request(c).bits.asUInt === arb.io.out.bits.asUInt
      io.request(c).ready := take(c)
      responses(c).io.enq.valid := readValid && readMask(c)
      responses(c).io.enq.bits.key := readTicket.key
      for (k <- 0 until side)
        responses(c).io.enq.bits.values(k) :=
          (if ((bf16Mask & (1 << c)) != 0) narrowed(k)
           else row(k))
      when(readValid && readMask(c)) {
        assert(responses(c).io.enq.ready, "unreserved publication response")
      }
    }
    readValid := arb.io.out.valid
    when(arb.io.out.valid) {
      readTicket := arb.io.out.bits
      readMask := take.asUInt
      pending(selected) := pending(selected) & ~take.asUInt
    }
    // Clearing pending reserves consumer copies, but the original data must stay
    // pinned until the registered read actually captures it in the response FIFOs.
    when(readValid && pending(readTicket.slot) === 0.U) {
      live(readTicket.slot) := false.B
      available(readTicket.slot) := false.B
    }
    io.captured := take.asUInt
    io.selected.valid := arb.io.out.valid
    io.selected.bits := arb.io.out.bits
    // Full slots cannot be reclaimed on the capture edge; reuse is next cycle.
    when(io.reserve.fire && arb.io.out.valid) { assert(tail =/= selected) }
  }
}
