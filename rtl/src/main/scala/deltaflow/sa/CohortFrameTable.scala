package deltaflow.sa
import chisel3._
import chisel3.util._

class CohortKey extends Bundle {
  val panel = UInt(2.W)
  val index = UInt(7.W)
}
class CohortFrame(consumers: Int) extends Bundle {
  val key = new CohortKey
  val data = UInt(256.W)
  val mask = UInt(consumers.W)
}

/** Two frame slots per admitted value-panel cohort, two live cohorts. Payload registers count as
  * logic area. No ready bypass or same-cycle reuse. A consumer owns a copy once fire occurs; others
  * can read either tagged slot. The external admission controller retires an old panel before its
  * parity is reused. Both index and full panel are matched, never parity alone.
  */
class CohortFrameTable(val consumers: Int = 8) extends Module {
  require(consumers == 2 || consumers == 4 || consumers == 8)
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new CohortFrame(consumers)))
    val request = Flipped(Vec(consumers, Decoupled(new CohortKey)))
    val response = Vec(consumers, Decoupled(UInt(256.W)))
    val occupied = Output(UInt(3.W))
  })
  val valid = RegInit(VecInit(Seq.fill(4)(false.B)))
  val keys = Reg(Vec(4, new CohortKey))
  val data = Reg(Vec(4, UInt(256.W)))
  val pending = Reg(Vec(4, UInt(consumers.W)))
  val cohort = io.in.bits.key.panel(0)
  val free = VecInit((0 until 4).map(i => !valid(i) && cohort === ((i / 2) == 1).B))
  val compatible = (0 until 4)
    .map(i => !valid(i) || keys(i).panel(0) =/= cohort || keys(i).panel === io.in.bits.key.panel)
    .reduce(_ && _)
  io.in.ready := free.asUInt.orR && compatible
  val position = PriorityEncoder(free.asUInt)
  val take = Wire(Vec(4, Vec(consumers, Bool())))
  for (c <- 0 until consumers) {
    val hit = VecInit(
      (0 until 4).map(i =>
        valid(i) && pending(i)(c) && keys(i).asUInt === io.request(c).bits.asUInt
      )
    )
    assert(PopCount(hit) <= 1.U)
    io.response(c).valid := io.request(c).valid && hit.asUInt.orR
    io.response(c).bits := Mux1H(hit, data)
    io.request(c).ready := hit.asUInt.orR && io.response(c).ready
    for (i <- 0 until 4) take(i)(c) := hit(i) && io.request(c).fire
  }
  for (i <- 0 until 4) {
    when(valid(i)) {
      val next = pending(i) & ~take(i).asUInt
      pending(i) := next
      when(!next.orR) { valid(i) := false.B }
    }
  }
  when(io.in.fire) {
    assert(io.in.bits.mask.orR, "empty multicast frame")
    assert(
      !(0 until 4).map(i => valid(i) && keys(i).asUInt === io.in.bits.key.asUInt).reduce(_ || _),
      "duplicate multicast key"
    )
    valid(position) := true.B
    keys(position) := io.in.bits.key
    data(position) := io.in.bits.data
    pending(position) := io.in.bits.mask
  }
  io.occupied := PopCount(valid)
}
