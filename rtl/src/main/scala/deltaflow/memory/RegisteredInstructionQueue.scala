package deltaflow.memory

import chisel3._
import chisel3.util._

/** Checks that depend only on instruction bits, never on live tile resources. */
object UnifiedInstructionCheck {
  def illegal(command: UInt, side: Int): Bool = {
    require(Set(4, 16).contains(side))
    val opcode = command(7, 0)
    val keys = command(39, 32)
    val values = command(47, 40)
    val dimensions = keys >= 1.U && keys <= (8 * side).U && values >= 1.U && values <= (8 * side).U
    val reserved = command(127, 51).orR || command(15, 12).orR
    val chunk = dimensions && !reserved
    val prepare = chunk && !command(50) && !command(49)
    val release = !reserved && !command(50) && !command(49) && !command(47, 32).orR
    val recurrent = dimensions && !command(127, 64).orR && !command(15, 12).orR &&
      command(54, 48) > 0.U && command(54, 48) <= 64.U && (command(63, 55) +& (keys << 1)) <= 512.U
    val legal = opcode <= 4.U ||
      (opcode === 33.U && chunk) ||
      (opcode === 34.U && prepare) ||
      (opcode === 35.U && release) ||
      ((opcode === 36.U || opcode === 37.U) && recurrent)
    !legal
  }
}

class RegisteredInstructionQueueIO(predecode: Boolean) extends QueueIO(UInt(128.W), 4) {
  val staticIllegal = if (predecode) Some(Output(Bool())) else None
}

/** Four 128-bit instruction entries with a directly registered head.
  *
  * Three circular tail entries replace the other three payload locations. The total payload
  * capacity remains 512 bits; there is no extra data slot. Like Queue(..., pipe = false, flow =
  * false), a full queue rejects enqueue even if the head is consumed in that cycle. Empty enqueue
  * becomes visible on the next cycle. Simultaneous dequeue/enqueue of the sole entry replaces the
  * head directly, without touching the tail pointers.
  */
class RegisteredInstructionQueue(
    val independentTailWrite: Boolean = false,
    val predecodeSide: Option[Int] = None
) extends Module {
  val io = IO(new RegisteredInstructionQueueIO(predecodeSide.nonEmpty))
  val head = Reg(UInt(128.W))
  val tail = Mem(3, UInt(128.W))
  val readPointer = RegInit(0.U(2.W))
  val writePointer = RegInit(0.U(2.W))
  val occupancy = RegInit(0.U(3.W))

  io.enq.ready := occupancy < 4.U
  io.deq.valid := occupancy =/= 0.U
  io.deq.bits := head
  io.count := occupancy

  val put = io.enq.fire
  val take = io.deq.fire
  val directHead = put && (occupancy === 0.U || (take && occupancy === 1.U))
  val tailPut = put && !directHead
  val tailTake = take && occupancy > 1.U

  when(directHead) {
    head := io.enq.bits
  }.elsewhen(tailTake) {
    head := tail(readPointer)
  }
  // The write pointer selects a free slot whenever enqueue is accepted.
  // Empty enqueue or sole-entry replacement may overwrite that invalid slot,
  // removing consumer readiness from the wide tail write enable. The slot
  // becomes valid only under tailPut; pointer and occupancy updates are exact.
  when(if (independentTailWrite) put else tailPut) {
    tail(writePointer) := io.enq.bits
  }
  predecodeSide.foreach { side =>
    // One control flag per existing entry. Payload capacity and all queue
    // transitions stay unchanged; no extra decode stage or data slot.
    val headIllegal = Reg(Bool())
    val tailIllegal = Mem(3, Bool())
    val enqueuedIllegal = UnifiedInstructionCheck.illegal(io.enq.bits, side)
    when(directHead) {
      headIllegal := enqueuedIllegal
    }.elsewhen(tailTake) {
      headIllegal := tailIllegal(readPointer)
    }
    when(if (independentTailWrite) put else tailPut) {
      tailIllegal(writePointer) := enqueuedIllegal
    }
    io.staticIllegal.get := headIllegal
    when(io.deq.valid) {
      assert(
        headIllegal === UnifiedInstructionCheck.illegal(head, side),
        "instruction legality flag must describe the registered head"
      )
    }
  }
  when(tailPut) {
    writePointer := Mux(writePointer === 2.U, 0.U, writePointer + 1.U)
  }
  when(tailTake) {
    readPointer := Mux(readPointer === 2.U, 0.U, readPointer + 1.U)
  }
  when(put =/= take) {
    occupancy := Mux(put, occupancy + 1.U, occupancy - 1.U)
  }

  assert(occupancy <= 4.U)
  assert(readPointer < 3.U && writePointer < 3.U)
}
