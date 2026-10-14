package deltaflow.memory

import chisel3._
import chisel3.util._

/** A bounded tag FIFO with a directly registered head. The head replaces one existing payload
  * entry; capacity is not increased. Acceptance matches Queue with pipe=false and flow=false,
  * including a full queue rejecting enqueue during simultaneous dequeue. Empty enqueue appears on
  * the next cycle. The tail has exactly entries-1 locations.
  */
class RegisteredTagQueue(val width: Int, val entries: Int, val registeredStatus: Boolean = false)
    extends Module {
  require(width > 0 && entries >= 2)
  val io = IO(new QueueIO(UInt(width.W), entries))
  private val tailEntries = entries - 1
  private val pointerBits = math.max(1, log2Ceil(tailEntries))
  val head = Reg(UInt(width.W))
  val tail = Mem(tailEntries, UInt(width.W))
  val readPointer =
    if (tailEntries == 1) 0.U(pointerBits.W)
    else RegInit(0.U(pointerBits.W))
  val writePointer =
    if (tailEntries == 1) 0.U(pointerBits.W)
    else RegInit(0.U(pointerBits.W))
  val occupancy = RegInit(0.U(log2Ceil(entries + 1).W))

  val room = if (registeredStatus) Some(RegInit(true.B)) else None
  val present = if (registeredStatus) Some(RegInit(false.B)) else None
  io.enq.ready := room.getOrElse(occupancy < entries.U)
  io.deq.valid := present.getOrElse(occupancy =/= 0.U)
  io.deq.bits := head
  io.count := occupancy

  val put = io.enq.fire
  val take = io.deq.fire
  val directHead = put && (occupancy === 0.U || (take && occupancy === 1.U))
  val tailPut = put && !directHead
  val tailTake = take && occupancy > 1.U
  def next(pointer: UInt): UInt =
    Mux(pointer === (tailEntries - 1).U, 0.U, pointer + 1.U)

  when(directHead) {
    head := io.enq.bits
  }.elsewhen(tailTake) {
    head := tail(readPointer)
  }
  when(tailPut) {
    tail(writePointer) := io.enq.bits
    if (tailEntries > 1) { writePointer := next(writePointer) }
  }
  when(tailTake) {
    if (tailEntries > 1) { readPointer := next(readPointer) }
  }
  when(put =/= take) {
    occupancy := Mux(put, occupancy + 1.U, occupancy - 1.U)
    if (registeredStatus) {
      room.get := Mux(take, true.B, occupancy =/= (entries - 1).U)
      present.get := Mux(put, true.B, occupancy =/= 1.U)
    }
  }

  if (registeredStatus) {
    assert(room.get === (occupancy < entries.U))
    assert(present.get === (occupancy =/= 0.U))
  }

  assert(occupancy <= entries.U)
  assert(readPointer < tailEntries.U && writePointer < tailEntries.U)
}
