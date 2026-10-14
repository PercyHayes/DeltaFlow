package deltaflow.memory

import chisel3._
import chisel3.util._

/** Typed view of the bounded registered-head storage. Packing changes no payload bit or capacity.
  * The first existing entry is the head register; the remaining entries hold the tail. Acceptance
  * retains Queue(pipe=false, flow=false) semantics, including simultaneous operations.
  */
class RegisteredPayloadQueue[T <: Data](gen: T, val entries: Int) extends Module {
  private val payloadBits = gen.getWidth
  require(payloadBits > 0 && entries >= 2)
  val io = IO(new QueueIO(gen, entries))
  val storage = Module(new RegisteredTagQueue(payloadBits, entries))

  storage.io.enq.valid := io.enq.valid
  storage.io.enq.bits := io.enq.bits.asUInt
  io.enq.ready := storage.io.enq.ready
  storage.io.deq.ready := io.deq.ready
  io.deq.valid := storage.io.deq.valid
  io.deq.bits := storage.io.deq.bits.asTypeOf(gen)
  io.count := storage.io.count
}
