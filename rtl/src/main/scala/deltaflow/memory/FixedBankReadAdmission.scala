package deltaflow.memory

import chisel3._
import chisel3.util._

/** Reserve the caller's existing tag before issuing a fixed group of reads. No bank ready controls
  * another bank valid. The caller holds the admitted address/index while pending is true; this
  * module stores only a bank mask.
  */
class FixedBankReadAdmission(
    val banks: Int,
    val registeredPending: Boolean = false
) extends Module {
  require(banks >= 1 && banks <= 16)
  val io = IO(new Bundle {
    val offer = Input(Bool())
    val room = Input(Bool())
    val bankReady = Input(Vec(banks, Bool()))
    val bankValid = Output(Vec(banks, Bool()))
    val admit = Output(Bool())
    val pending = Output(Bool())
  })
  val remaining = RegInit(0.U(banks.W))
  val pending = if (registeredPending) Some(RegInit(false.B)) else None
  io.pending := pending.getOrElse(remaining.orR)
  pending.foreach { status =>
    assert(status === remaining.orR, "cached pending differs from remaining banks")
  }
  io.admit := io.offer && io.room && !io.pending
  val needed = Mux(io.pending, remaining, Fill(banks, 1.U(1.W)))
  val issue = io.pending || io.admit
  val accepted = Wire(Vec(banks, Bool()))
  for (bank <- 0 until banks) {
    io.bankValid(bank) := issue && needed(bank)
    accepted(bank) := io.bankValid(bank) && io.bankReady(bank)
  }
  val left = needed & ~accepted.asUInt
  when(issue) {
    remaining := left
    pending.foreach { status => status := left.orR }
  }
  when(io.pending) { assert(!io.admit, "cannot admit a row before all its bank reads issue") }
}
