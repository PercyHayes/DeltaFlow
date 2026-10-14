package deltaflow.memory

import chisel3._

/** Command dimensions remain fixed for an entire H/O epoch. Capture this one-bit phase limit with
  * the command instead of decoding the dimensions again on the completion and shared-memory
  * admission path.
  */
object SecondValueHalf {
  def apply(start: Bool, valueColumns: UInt, tokens: Int): Bool = {
    require(Set(16, 64).contains(tokens))
    val second = RegInit(false.B)
    when(start) { second := valueColumns > tokens.U }
    second
  }
}

/** Local timing probe retains the command register in both versions. The production backend already
  * has this register for its other dimension uses.
  */
class ValueHalfCompletionProbe(val tokens: Int, val cached: Boolean) extends Module {
  require(Set(16, 64).contains(tokens))
  val io = IO(new Bundle {
    val start = Input(Bool())
    val valueColumns = Input(UInt(8.W))
    val half = Input(Bool())
    val finish = Input(Bool())
    val columns = Output(UInt(8.W))
    val lastHalf = Output(Bool())
    val done = Output(Bool())
  })
  val configured = RegInit(false.B)
  val columns = Reg(UInt(8.W))
  when(io.start) {
    columns := io.valueColumns
    configured := true.B
  }
  val second =
    if (cached) SecondValueHalf(io.start, io.valueColumns, tokens)
    else columns > tokens.U
  when(configured) {
    assert(second === (columns > tokens.U), "value-half cache differs from command")
  }
  io.columns := columns
  io.lastHalf := io.half || !second
  io.done := configured && io.finish && io.lastHalf
}
