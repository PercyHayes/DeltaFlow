package deltaflow.memory

import chisel3._
import chisel3.util._

/** Atomic two-word DMA write with target-local permission. Each available bit represents both banks
  * of one region/group. Accepted never depends on another target's availability; ready retains the
  * same selected atomic transaction. No payload storage or pipeline is added.
  */
class DmaLocalWriteAdmission extends Module {
  val io = IO(new Bundle {
    val valid = Input(Bool())
    val permit = Input(Bool())
    val region = Input(Bool())
    val group = Input(Bool())
    val available = Input(Vec(4, Bool()))
    val ready = Output(Bool())
    val accepted = Output(Vec(4, Bool()))
  })
  io.ready := io.permit && io.available(Cat(io.region, io.group))
  for (region <- 0 until 2; group <- 0 until 2) {
    val index = 2 * region + group
    io.accepted(index) := io.valid && io.permit && io.region === (region == 1).B &&
      io.group === (group == 1).B && io.available(index)
  }
}
