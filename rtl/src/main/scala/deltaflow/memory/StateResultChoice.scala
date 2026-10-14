package deltaflow.memory

import chisel3._

/** H/O choice retained through partial writes and source-valid gaps. The encoded version replaces
  * the original two control registers. States are free=(allowH=1,forceH=0), H=(1,1), O=(0,0).
  */
class StateResultChoice(val encoded: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val hasH = Input(Bool())
    val hasO = Input(Bool())
    val ready = Input(Bool())
    val chooseH = Output(Bool())
    val validH = Output(Bool())
    val validO = Output(Bool())
    val valid = Output(Bool())
  })

  if (encoded) {
    val allowH = RegInit(true.B)
    val forceH = RegInit(false.B)
    io.chooseH := forceH || (allowH && io.hasH)
    io.validH := allowH && io.hasH
    io.validO := !forceH && !io.validH && io.hasO
    when(io.valid && !io.ready) {
      allowH := io.chooseH
      forceH := io.chooseH
    }
    when(io.valid && io.ready) {
      allowH := true.B
      forceH := false.B
    }
    assert(!forceH || allowH, "invalid H/O held-choice encoding")
    assert(io.validH === (io.chooseH && io.hasH))
    assert(io.validO === (!io.chooseH && io.hasO))
  } else {
    val locked = RegInit(false.B)
    val heldH = Reg(Bool())
    io.chooseH := Mux(locked, heldH, io.hasH)
    io.validH := io.chooseH && io.hasH
    io.validO := !io.chooseH && io.hasO
    when(io.valid && !io.ready) {
      locked := true.B
      heldH := io.chooseH
    }
    when(io.valid && io.ready) { locked := false.B }
  }
  io.valid := io.validH || io.validO
}
