package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.memory.{BankWrite, MacroScratchpad}
import deltaflow.arithmetic.FloatArithmetic

class RetimedMemoryRequest(depth: Int) extends Bundle {
  val source = UInt(log2Ceil(depth).W)
  val rhs = UInt(log2Ceil(depth).W)
  val destination = UInt(log2Ceil(depth).W)
  val feedback = Bool()
  val os = Bool()
  val context = UInt(2.W)
  val tag = UInt(2.W)
}

/** A four-C32 retimed panel with synchronous banked operands and result feedback. Every CS request
  * reserves a completion slot BEFORE memory reads. Results are write-once per reset; feedback
  * cannot read a merely reserved destination.
  */
class RetimedMemoryEngine(val side: Int, val depth: Int = 256, val completionDepth: Int = 65)
    extends Module {
  require(Set(4, 16).contains(side))
  require(depth >= 64 && depth <= 256 && isPow2(depth))
  require(completionDepth >= 2 && completionDepth <= 128)
  val nb = (side + 7) / 8
  val wb = side / 4
  val io = IO(new Bundle {
    val request = Flipped(Decoupled(new RetimedMemoryRequest(depth)))
    val operandWrite = Flipped(Vec(nb, Decoupled(new BankWrite(depth, 16))))
    val rhsWrite = Flipped(Vec(wb, Decoupled(new BankWrite(depth, 16))))
    // Models grant of the result banks' write port; no unbounded output sink.
    val commitEnable = Input(Bool())
    val hostRead = Flipped(Decoupled(UInt(log2Ceil(depth).W)))
    val hostResponse = Decoupled(Vec(side, UInt(32.W)))
    val coefficient = Flipped(Decoupled(new WaveCoefficient(side)))
    val seed = Flipped(Decoupled(new RetimedRowData(side)))
    val clear = Flipped(Decoupled(UInt(2.W)))
    val read = Flipped(Decoupled(new RetimedRow(side)))
    val response = Decoupled(new RetimedRowData(side))
    val idle = Output(Bool())
    val accepted = Valid(new RetimedMemoryRequest(depth))
    val launched = Valid(new RetimedMemoryRequest(depth))
    val returned = Valid(UInt(log2Ceil(depth).W))
    val committed = Valid(UInt(log2Ceil(depth).W))
    val reserved = Output(UInt(log2Ceil(completionDepth + 1).W))
    val queued = Output(UInt(log2Ceil(completionDepth + 1).W))
    val advance = Output(Bool())
  })
  val panel = Module(new RetimedWavePanel(side))
  val operand = Module(new MacroScratchpad(nb, depth, 16))
  val rhs = Module(new MacroScratchpad(wb, depth, 16))
  val result = Module(new MacroScratchpad(wb, depth, 16))
  val reads = Module(new Queue(new RetimedMemoryRequest(depth), 4, pipe = false, flow = false))
  // Registered conversion/operand boundary: do not concatenate narrow with product.
  val staged = Module(
    new Queue(
      new Bundle {
        val request = new RetimedMemoryRequest(depth)
        val input = new RetimedPanelInput(side, 2)
      },
      2,
      pipe = true,
      flow = false
    )
  )
  val destinations = Module(
    new Queue(UInt(log2Ceil(depth).W), completionDepth, pipe = false, flow = false)
  )
  val completed = Module(
    new Queue(new CompletedWave(side, depth), completionDepth, pipe = false, flow = false)
  )
  val reserved = RegInit(0.U(log2Ceil(completionDepth + 1).W))
  val allocated = RegInit(VecInit(Seq.fill(depth)(false.B)))
  val written = RegInit(VecInit(Seq.fill(depth)(false.B)))
  val hostPending = RegInit(false.B)
  val idle =
    reads.io.count === 0.U && staged.io.count === 0.U && reserved === 0.U && panel.io.idle &&
      !hostPending
  io.idle := idle
  // Ownership starts at accepted memory requests, before panel launch.
  val controlSafe = reads.io.count === 0.U && staged.io.count === 0.U && !io.request.valid
  val coefficientSafe = controlSafe && destinations.io.count === 0.U
  panel.io.coefficient.valid := io.coefficient.valid && coefficientSafe
  panel.io.coefficient.bits := io.coefficient.bits
  io.coefficient.ready := panel.io.coefficient.ready && coefficientSafe
  panel.io.seed.valid := io.seed.valid && controlSafe
  panel.io.seed.bits := io.seed.bits
  io.seed.ready := panel.io.seed.ready && controlSafe
  panel.io.clear.valid := io.clear.valid && controlSafe
  panel.io.clear.bits := io.clear.bits
  io.clear.ready := panel.io.clear.ready && controlSafe
  panel.io.read.valid := io.read.valid && controlSafe
  panel.io.read.bits := io.read.bits
  io.read.ready := panel.io.read.ready && controlSafe
  io.response <> panel.io.response

  // Host initialization cannot modify a bank while a computation is active.
  for (b <- 0 until nb) {
    operand.io.write(b).bits := io.operandWrite(b).bits
    operand.io.write(b).valid := io.operandWrite(b).valid && idle && !io.request.valid
    io.operandWrite(b).ready := operand.io.write(b).ready && idle && !io.request.valid
  }
  for (b <- 0 until wb) {
    rhs.io.write(b).bits := io.rhsWrite(b).bits
    rhs.io.write(b).valid := io.rhsWrite(b).valid && idle && !io.request.valid
    io.rhsWrite(b).ready := rhs.io.write(b).ready && idle && !io.request.valid
  }
  val resultWriteReady = result.io.write.map(_.ready).reduce(_ && _)
  val commit = completed.io.deq.valid && io.commitEnable && resultWriteReady
  completed.io.deq.ready := io.commitEnable && resultWriteReady
  for (b <- 0 until wb) {
    result.io.write(b).valid := completed.io.deq.valid && io.commitEnable && resultWriteReady
    result.io.write(b).bits.address := completed.io.deq.bits.destination
    result.io.write(b).bits.data := Cat(
      (0 until 4).reverse.map(i => completed.io.deq.bits.values(b * 4 + i))
    )
    result.io.write(b).bits.mask := "hffff".U
  }
  when(commit) { written(completed.io.deq.bits.destination) := true.B }

  val requestBanksReady = rhs.io.read.map(_.ready).reduce(_ && _) && Mux(
    io.request.bits.feedback,
    result.io.read.map(_.ready).reduce(_ && _),
    operand.io.read.map(_.ready).reduce(_ && _)
  )
  val sourceReady = !io.request.bits.feedback || written(io.request.bits.source)
  val destinationReady = io.request.bits.os ||
    (!allocated(
      io.request.bits.destination
    ) && reserved < completionDepth.U && destinations.io.enq.ready)
  io.request.ready := reads.io.enq.ready && requestBanksReady && sourceReady && destinationReady &&
    !hostPending && !io.hostRead.valid
  val accept = io.request.fire
  reads.io.enq.valid := accept
  reads.io.enq.bits := io.request.bits
  destinations.io.enq.valid := accept && !io.request.bits.os
  destinations.io.enq.bits := io.request.bits.destination
  when(accept && !io.request.bits.os) { allocated(io.request.bits.destination) := true.B }

  val hostAllowed = idle && !io.request.valid && written(io.hostRead.bits)
  io.hostRead.ready := hostAllowed && result.io.read.map(_.ready).reduce(_ && _)
  when(io.hostRead.fire) { hostPending := true.B }
  for (b <- 0 until nb) {
    operand.io.read(b).valid := accept && !io.request.bits.feedback
    operand.io.read(b).bits := io.request.bits.source
  }
  for (b <- 0 until wb) {
    rhs.io.read(b).valid := accept
    rhs.io.read(b).bits := io.request.bits.rhs
    result.io.read(b).valid := (accept && io.request.bits.feedback) || io.hostRead.fire
    result.io.read(b).bits := Mux(
      idle && io.hostRead.valid && !io.request.valid,
      io.hostRead.bits,
      io.request.bits.source
    )
  }
  val sourceValid = Mux(
    reads.io.deq.bits.feedback,
    result.io.response.map(_.valid).reduce(_ && _),
    operand.io.response.map(_.valid).reduce(_ && _)
  )
  val joinValid = reads.io.deq.valid && sourceValid && rhs.io.response.map(_.valid).reduce(_ && _)
  staged.io.enq.valid := joinValid
  staged.io.enq.bits.request := reads.io.deq.bits
  panel.io.in.valid := staged.io.deq.valid
  panel.io.in.bits := staged.io.deq.bits.input
  staged.io.deq.ready := panel.io.in.ready
  staged.io.enq.bits.input.os := reads.io.deq.bits.os
  staged.io.enq.bits.input.context := reads.io.deq.bits.context
  staged.io.enq.bits.input.tag := reads.io.deq.bits.tag
  for (i <- 0 until side) {
    val feedbackWord = result.io.response(i / 4).bits(32 * (i % 4) + 31, 32 * (i % 4))
    staged.io.enq.bits.input.north(i) := Mux(
      reads.io.deq.bits.feedback,
      FloatArithmetic.narrow(feedbackWord),
      operand.io.response(i / 8).bits(16 * (i % 8) + 15, 16 * (i % 8))
    )
    staged.io.enq.bits.input.west(i) := rhs.io.response(i / 4).bits(32 * (i % 4) + 31, 32 * (i % 4))
    io.hostResponse.bits(i) := feedbackWord
  }
  reads.io.deq.ready := staged.io.enq.ready && sourceValid &&
    rhs.io.response
      .map(_.valid)
      .reduce(_ && _)
  val join = staged.io.enq.fire
  val launch = panel.io.in.fire
  io.hostResponse.valid := hostPending && result.io.response.map(_.valid).reduce(_ && _)
  when(io.hostResponse.fire) { hostPending := false.B }
  for (b <- 0 until nb) operand.io.response(b).ready := join && !reads.io.deq.bits.feedback
  for (b <- 0 until wb) {
    rhs.io.response(b).ready := join
    result.io.response(b).ready := (join && reads.io.deq.bits.feedback) || io.hostResponse.fire
  }
  completed.io.enq.valid := panel.io.out.valid && destinations.io.deq.valid
  completed.io.enq.bits.destination := destinations.io.deq.bits
  completed.io.enq.bits.values := panel.io.out.bits.values
  panel.io.out.ready := completed.io.enq.ready && destinations.io.deq.valid
  destinations.io.deq.ready := panel.io.out.valid && completed.io.enq.ready
  when(panel.io.out.valid) {
    assert(destinations.io.deq.valid, "missing destination reservation")
    assert(completed.io.enq.ready, "completion reservation failed")
  }
  val reserve = accept && !io.request.bits.os
  when(reserve =/= commit) { reserved := Mux(reserve, reserved + 1.U, reserved - 1.U) }
  when(commit) { assert(reserved =/= 0.U) }
  assert(reserved <= completionDepth.U)
  io.accepted.valid := accept
  io.accepted.bits := io.request.bits
  io.launched.valid := launch
  io.launched.bits := staged.io.deq.bits.request
  io.returned.valid := completed.io.enq.fire
  io.returned.bits := destinations.io.deq.bits
  io.committed.valid := commit
  io.committed.bits := completed.io.deq.bits.destination
  io.reserved := reserved
  io.queued := completed.io.count
  io.advance := panel.io.advance
}
