package deltaflow.memory

import chisel3._

/** Exact nonempty status for the existing non-bypassing return FIFO. The caller reserves capacity
  * before issue, so every put is accepted. This is one control bit; the original queue owns payload
  * and pointers.
  */
object BorrowedReturnStatus {
  def apply(queuedCount: UInt, put: Bool, downstreamReady: Bool): Bool = {
    val present = RegInit(false.B)
    val take = present && downstreamReady
    when(put =/= take) {
      present := put || queuedCount > 1.U
    }
    present
  }
}
