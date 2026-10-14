package deltaflow.memory

import chisel3._

/** An H tag reserves one borrowed-FP32 response during the H/O phase. N production starts with an
  * empty tag queue and cannot enqueue an H tag. Keep that invariant checked while using the tag as
  * the return qualifier.
  */
object HReturnPresent {
  def apply(producingN: Bool, tagPresent: Bool, responseValid: Bool): Bool = {
    when(producingN) {
      assert(!tagPresent, "N production cannot hold an H return tag")
    }
    tagPresent && responseValid
  }
}

/** Standalone combinational screen; only legal phase/tag combinations apply. */
class HReturnSelectionProbe(qualified: Boolean) extends Module {
  val io = IO(new Bundle {
    val producingN = Input(Bool())
    val tagPresent = Input(Bool())
    val responseValid = Input(Bool())
    val hasH = Output(Bool())
  })
  io.hasH :=
    (if (qualified)
       HReturnPresent(io.producingN, io.tagPresent, io.responseValid)
     else !io.producingN && io.tagPresent && io.responseValid)
}
