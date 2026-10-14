package deltaflow.memory

import chisel3._
import chisel3.util._

/** Exact registered availability around the original bounded Queue storage. Payload entries,
  * pointers and pipe=false/flow=false acceptance stay intact. The status flags describe the current
  * occupancy, without an extra stage.
  */
class RegisteredStatusQueue[T <: Data](
    gen: T,
    val entries: Int,
    val registeredStatus: Boolean = true
) extends Module {
  require(entries >= 2)
  val io = IO(new QueueIO(gen, entries))
  val storage = Module(new Queue(gen, entries, pipe = false, flow = false))
  val room = if (registeredStatus) Some(RegInit(true.B)) else None
  val present = if (registeredStatus) Some(RegInit(false.B)) else None

  storage.io.enq.valid := io.enq.valid
  storage.io.enq.bits := io.enq.bits
  storage.io.deq.ready := io.deq.ready
  io.enq.ready := room.getOrElse(storage.io.enq.ready)
  io.deq.valid := present.getOrElse(storage.io.deq.valid)
  io.deq.bits := storage.io.deq.bits
  io.count := storage.io.count

  if (registeredStatus) {
    val put = storage.io.enq.fire
    val take = storage.io.deq.fire
    when(put =/= take) {
      room.get := take || storage.io.count =/= (entries - 1).U
      present.get := put || storage.io.count =/= 1.U
    }
    assert(room.get === storage.io.enq.ready, "queue room changed an acceptance cycle")
    assert(present.get === storage.io.deq.valid, "queue present changed a visibility cycle")
  }
}
