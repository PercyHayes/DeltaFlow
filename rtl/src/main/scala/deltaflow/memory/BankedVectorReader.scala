package deltaflow.memory
import chisel3._
import chisel3.util._

class VectorAddress extends Bundle {
  val word = UInt(13.W)
  val key = UInt(16.W)
}
class GatheredVector(words: Int) extends Bundle {
  val key = UInt(16.W)
  val data = Vec(words, UInt(128.W))
}

/** Four finite vector streams share sixteen 128-bit synchronous 1R1W banks. Each accepted vector
  * reserves a complete frame before issuing bank reads. Tags remain with bank responses;
  * independent words join into that frame. All frame data and macro response queues are logic, not
  * SRAM blackboxes.
  */
class BankedVectorReader(val side: Int = 16, val frames: Int = 4) extends Module {
  require(Set(4, 16).contains(side))
  require(Set(2, 4).contains(frames))
  val sizes =
    Seq((side * 6 + 15) / 16, (side * 4 + 15) / 16, (side * 4 + 15) / 16, (side * 4 + 15) / 16)
  val maxWords = sizes.max
  val slotBits = log2Ceil(frames)
  val io = IO(new Bundle {
    val request = Flipped(Vec(4, Decoupled(new VectorAddress)))
    val response = Vec(4, Decoupled(new GatheredVector(maxWords)))
    val write = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val idle = Output(Bool())
    val granted = Output(UInt(4.W))
    val bankReads = Output(UInt(16.W))
    val bankReturns = Output(UInt(16.W))
    val occupied = Output(Vec(4, UInt(log2Ceil(frames + 1).W)))
    val conflict = Output(UInt(4.W))
    val capacity = Output(UInt(4.W))
  })
  val memory = Module(new MacroScratchpad(16, 512, 16))
  memory.io.write <> io.write
  val tags = Seq.fill(16)(
    Module(
      new Queue(
        new Bundle {
          val port = UInt(2.W)
          val slot = UInt(slotBits.W)
          val lane = UInt(log2Ceil(maxWords).W)
        },
        2,
        pipe = true,
        flow = false
      )
    )
  )
  val phase = RegInit(0.U(2.W))
  phase := phase + 1.U
  val head = RegInit(VecInit(Seq.fill(4)(0.U(slotBits.W))))
  val tail = RegInit(VecInit(Seq.fill(4)(0.U(slotBits.W))))
  val count = RegInit(VecInit(Seq.fill(4)(0.U(log2Ceil(frames + 1).W))))
  val masks = Wire(Vec(4, UInt(16.W)))
  val available = VecInit(
    (0 until 16).map(b => (tags(b).io.count < 2.U || tags(b).io.deq.fire) && io.bankEnable(b))
  ).asUInt
  val grants = Wire(Vec(4, Bool()))
  for (p <- 0 until 4) {
    masks(p) := (0 until sizes(p))
      .map(w => UIntToOH((io.request(p).bits.word + w.U)(3, 0), 16))
      .reduce(_ | _)
    when(io.request(p).valid) { assert(io.request(p).bits.word +& sizes(p).U <= 8192.U) }
  }
  // Compute four fixed priority orders in parallel on pairwise conflict bits.
  // Avoid a serial cascade of dynamic sixteen-bit mask selectors.
  val eligible = VecInit(
    (0 until 4).map(p =>
      io.request(p).valid && count(p) < frames.U && (masks(p) & available) === masks(p)
    )
  )
  val orders = Wire(Vec(4, Vec(4, Bool())))
  for (start <- 0 until 4; k <- 0 until 4) {
    val p = (start + k) % 4
    val blocked = (0 until k).map(j => {
      val q = (start + j) % 4
      orders(start)(q) && (masks(p) & masks(q)).orR
    })
    orders(start)(p) := eligible(p) && !blocked.foldLeft(false.B)(_ || _)
  }
  for (p <- 0 until 4)
    grants(p) := Mux1H((0 until 4).map(s => phase === s.U), (0 until 4).map(s => orders(s)(p)))
  for (p <- 0 until 4) { io.request(p).ready := grants(p) }
  for (b <- 0 until 16) {
    val hits =
      for (p <- 0 until 4; w <- 0 until sizes(p))
        yield grants(p) && (io.request(p).bits.word + w.U)(3, 0) === b.U
    val entries = for (p <- 0 until 4; w <- 0 until sizes(p)) yield (p, w)
    val valid = hits.reduce(_ || _)
    memory.io.read(b).valid := valid
    memory.io.read(b).bits := Mux1H(
      hits,
      entries.map { case (p, w) => (io.request(p).bits.word + w.U)(12, 4) }
    )
    tags(b).io.enq.valid := valid
    tags(b).io.enq.bits.port := Mux1H(hits, entries.map { case (p, w) => p.U(2.W) })
    tags(b).io.enq.bits.slot := Mux1H(hits, entries.map { case (p, w) => tail(p) })
    tags(b).io.enq.bits.lane := Mux1H(
      hits,
      entries.map { case (p, w) => w.U(log2Ceil(maxWords).W) }
    )
    when(valid) {
      assert(memory.io.read(b).ready)
      assert(tags(b).io.enq.ready)
      assert(PopCount(VecInit(hits)) === 1.U)
    }
    memory.io.response(b).ready := tags(b).io.deq.valid
    tags(b).io.deq.ready := memory.io.response(b).valid
    when(memory.io.response(b).valid) { assert(tags(b).io.deq.valid) }
  }
  for (p <- 0 until 4) {
    val data = Reg(Vec(frames, Vec(sizes(p), UInt(128.W))))
    val complete = RegInit(VecInit(Seq.fill(frames)(0.U(sizes(p).W))))
    val keys = Reg(Vec(frames, UInt(16.W)))
    val live = RegInit(VecInit(Seq.fill(frames)(false.B)))
    io.response(p).valid := count(p) =/= 0.U && complete(head(p)).andR
    io.response(p).bits.key := keys(head(p))
    for (w <- 0 until maxWords)
      io.response(p).bits.data(w) := (if (w < sizes(p)) data(head(p))(w) else 0.U)
    // Atomic vector reads and always-drained bank responses permit one shared
    // payload mux per stream/lane, rather than duplicating it for every slot.
    val arrived = Wire(Vec(sizes(p), Bool()))
    val arrivalSlot = Wire(Vec(sizes(p), UInt(slotBits.W)))
    val payload = Wire(Vec(sizes(p), UInt(128.W)))
    for (w <- 0 until sizes(p)) {
      val hit = (0 until 16).map(b =>
        memory.io.response(b).fire && tags(b).io.deq.bits.port === p.U && tags(
          b
        ).io.deq.bits.lane === w.U
      )
      arrived(w) := hit.reduce(_ || _)
      arrivalSlot(w) := Mux1H(hit, tags.map(_.io.deq.bits.slot))
      payload(w) := Mux1H(hit, memory.io.response.map(_.bits))
      assert(PopCount(VecInit(hit)) <= 1.U)
    }
    for (s <- 0 until frames) {
      val got = Wire(Vec(sizes(p), Bool()))
      for (w <- 0 until sizes(p)) {
        got(w) := arrived(w) && arrivalSlot(w) === s.U
        when(got(w)) {
          assert(live(s))
          assert(!complete(s)(w))
          data(s)(w) := payload(w)
        }
      }
      when(got.asUInt.orR) { complete(s) := complete(s) | got.asUInt }
      when(io.response(p).fire && head(p) === s.U) {
        live(s) := false.B
        complete(s) := 0.U
      }
      when(io.request(p).fire && tail(p) === s.U) {
        assert(!live(s))
        live(s) := true.B
        complete(s) := 0.U
        keys(s) := io.request(p).bits.key
      }
    }
    when(io.request(p).fire) { tail(p) := tail(p) + 1.U }
    when(io.response(p).fire) { head(p) := head(p) + 1.U }
    when(io.request(p).fire =/= io.response(p).fire) {
      count(p) := Mux(io.request(p).fire, count(p) + 1.U, count(p) - 1.U)
    }
    io.occupied(p) := count(p)
  }
  io.idle := count.map(_ === 0.U).reduce(_ && _) && tags.map(_.io.count === 0.U).reduce(_ && _)
  io.granted := grants.asUInt
  io.bankReads := VecInit(memory.io.read.map(_.fire)).asUInt
  io.bankReturns := VecInit(memory.io.response.map(_.fire)).asUInt
  io.capacity := VecInit((0 until 4).map(p => io.request(p).valid && count(p) === frames.U)).asUInt
  io.conflict := VecInit(
    (0 until 4).map(p => io.request(p).valid && count(p) < frames.U && !grants(p))
  ).asUInt
}
