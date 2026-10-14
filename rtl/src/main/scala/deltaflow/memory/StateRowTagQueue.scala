package deltaflow.memory

import chisel3._
import chisel3.util._

class StateRowTagQueueIO(entries: Int) extends QueueIO(UInt(8.W), entries) {
  val keyColumns = Input(UInt(8.W))
  val withinRange = Output(Bool())
}

/** Same H row tags and capacity, optionally carrying their range qualification. The owning command
  * keeps keyColumns stable until all its tags retire. The extra bit is control metadata, not
  * another row or payload location.
  */
class StateRowTagQueue(val entries: Int, val captureRange: Boolean = false) extends Module {
  val io = IO(new StateRowTagQueueIO(entries))
  val queue = Module(
    new RegisteredTagQueue(if (captureRange) 9 else 8, entries, registeredStatus = captureRange)
  )
  queue.io.enq.valid := io.enq.valid
  queue.io.enq.bits :=
    (if (captureRange)
       Cat(io.enq.bits < io.keyColumns, io.enq.bits)
     else io.enq.bits)
  io.enq.ready := queue.io.enq.ready
  queue.io.deq.ready := io.deq.ready
  io.deq.valid := queue.io.deq.valid
  io.deq.bits := queue.io.deq.bits(7, 0)
  io.count := queue.io.count
  io.withinRange :=
    (if (captureRange) queue.io.deq.bits(8)
     else io.deq.bits < io.keyColumns)
  when(io.deq.valid) {
    assert(
      io.withinRange === (io.deq.bits < io.keyColumns),
      "H row range qualification must remain valid for its owning command"
    )
  }
}
