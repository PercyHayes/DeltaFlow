package deltaflow.system

import chisel3._

/** Confirm legality once for the current GDN queue head. Resource readiness stays live in the
  * owner; this stage stores no payload. Mode changes and consumption invalidate the confirmation
  * before reuse.
  */
class GdnCommandCheck extends Module {
  val io = IO(new Bundle {
    val eligible = Input(Bool())
    val invalidHead = Input(Bool())
    val consumed = Input(Bool())
    val modeChanged = Input(Bool())
    val bad = Input(Bool())
    val checked = Output(Bool())
    val checkedBad = Output(Bool())
  })

  val checked = RegInit(false.B)
  val checkedBad = Reg(Bool())
  when(io.invalidHead || io.consumed || io.modeChanged) {
    checked := false.B
  }.elsewhen(io.eligible && !checked) {
    checked := true.B
    checkedBad := io.bad
  }
  io.checked := checked
  io.checkedBad := checkedBad

  when(io.eligible && checked) {
    assert(
      checkedBad === io.bad,
      "GDN legality must remain stable while its confirmed head is resident"
    )
  }
}
