package deltaflow.memory

import chisel3._
import chisel3.util._

/** Track one bounded, ordered gather without broadcasting its completion back to every bank
  * register. Each accepted bank records the current epoch. The epoch changes only when the original
  * gather completes, so the next gather begins with no grants and no additional cycle.
  */
class LocalBankGrants(val banks: Int) extends Module {
  require(banks > 0 && banks <= 32)
  val io = IO(new Bundle {
    val inRequest = Input(Bool())
    val offer = Input(Bool())
    val needed = Input(UInt(banks.W))
    val accepted = Input(UInt(banks.W))
    val granted = Output(UInt(banks.W))
    val complete = Output(Bool())
  })

  val epoch = RegInit(false.B)
  val acceptedEpoch = RegInit(VecInit(Seq.fill(banks)(true.B)))
  val granted = VecInit((0 until banks).map { bank =>
    io.inRequest && io.needed(bank) && (acceptedEpoch(bank) === epoch)
  }).asUInt
  io.granted := granted
  io.complete := io.offer && (((granted | io.accepted) & io.needed) === io.needed)

  when(io.complete) { epoch := !epoch }
  for (bank <- 0 until banks) {
    when(!io.inRequest) {
      // Between frames the controller may change its needed-bank mask.
      acceptedEpoch(bank) := !epoch
    }.elsewhen(io.accepted(bank) || !io.needed(bank)) {
      // Unneeded banks are also armed for the next epoch. This matters when
      // W/Q and H gathers use different bank masks without an idle cycle.
      acceptedEpoch(bank) := epoch
    }
  }
  assert(!io.offer || io.inRequest, "bank offer requires a live gather")
  assert(!(io.accepted & ~io.needed).orR, "an unneeded bank was accepted")
  assert(!(io.accepted & granted).orR, "bank accepted the same gather twice")
}
