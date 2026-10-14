package deltaflow.sa
import chisel3._
import chisel3.util._

class GramWordReturn extends Bundle {
  val slot = Bool()
  val data = UInt(128.W)
}

/** Two reserved synchronous-read words, retained until every Gram consumer captures them. Payload
  * is a register buffer, never an SRAM blackbox. A full chunk must drain before feature indices are
  * reused. The controller must place a register between response.fire and PE admission.
  */
class GramOperandBuffer(val consumers: Int) extends Module {
  require(consumers >= 1 && consumers <= 8)
  val io = IO(new Bundle {
    val allocate = Flipped(Decoupled(UInt(7.W)))
    val allocatedSlot = Output(Bool())
    val returned = Flipped(Valid(new GramWordReturn))
    val request = Flipped(Vec(consumers, Decoupled(UInt(7.W))))
    val response = Vec(consumers, Decoupled(UInt(128.W)))
    val occupied = Output(UInt(2.W))
  })
  val valid = RegInit(VecInit(Seq.fill(2)(false.B)))
  val available = RegInit(VecInit(Seq.fill(2)(false.B)))
  val key = Reg(Vec(2, UInt(7.W)))
  val data = Reg(Vec(2, UInt(128.W)))
  val pending = Reg(Vec(2, UInt(consumers.W)))
  val duplicate = (0 until 2).map(i => valid(i) && key(i) === io.allocate.bits).reduce(_ || _)
  io.allocate.ready := !valid.asUInt.andR && !duplicate
  io.allocatedSlot := valid(0)
  val take = Wire(Vec(2, Vec(consumers, Bool())))
  for (c <- 0 until consumers) {
    val hit = VecInit(
      (0 until 2).map(i =>
        valid(i) && available(i) && pending(i)(c) && key(i) === io.request(c).bits
      )
    )
    assert(PopCount(hit) <= 1.U)
    io.response(c).valid := io.request(c).valid && hit.asUInt.orR
    io.response(c).bits := Mux1H(hit, data)
    io.request(c).ready := hit.asUInt.orR && io.response(c).ready
    for (i <- 0 until 2) take(i)(c) := hit(i) && io.request(c).fire
  }
  for (i <- 0 until 2) {
    when(valid(i) && available(i)) {
      val remaining = pending(i) & ~take(i).asUInt
      pending(i) := remaining
      when(!remaining.orR) {
        valid(i) := false.B
        available(i) := false.B
      }
    }
  }
  when(io.allocate.fire) {
    val slot = io.allocatedSlot
    valid(slot) := true.B
    available(slot) := false.B
    key(slot) := io.allocate.bits
    pending(slot) := ((1 << consumers) - 1).U
  }
  when(io.returned.valid) {
    val slot = io.returned.bits.slot
    assert(valid(slot) && !available(slot), "unreserved or duplicate Gram return")
    data(slot) := io.returned.bits.data
    available(slot) := true.B
  }
  io.occupied := PopCount(valid)
}
