package deltaflow.sa
import chisel3._
import chisel3.util._

/** Two-entry elastic input, with a physical head register (no read-index mux). No fall-through and
  * no same-cycle reuse of a full queue's retiring credit. All mode/context/tag/data bits move
  * together. Storage is logic, not SRAM.
  */
class RegisteredWaveInput(side: Int, tagBits: Int) extends Module {
  require(Set(4, 8, 16, 32).contains(side))
  require(tagBits > 0)
  val io = IO(new Bundle {
    val enq = Flipped(Decoupled(new RetimedPanelInput(side, tagBits)))
    val deq = Decoupled(new RetimedPanelInput(side, tagBits))
    val count = Output(UInt(2.W))
  })
  val count = RegInit(0.U(2.W))
  val head = Reg(new RetimedPanelInput(side, tagBits))
  val tail = Reg(new RetimedPanelInput(side, tagBits))
  io.enq.ready := count < 2.U
  io.deq.valid := count =/= 0.U
  io.deq.bits := head
  io.count := count
  val push = io.enq.fire
  val pop = io.deq.fire
  when(push =/= pop) { count := Mux(push, count + 1.U, count - 1.U) }
  when(pop && count === 2.U) { head := tail }
    .elsewhen(push && (count === 0.U || pop)) { head := io.enq.bits }
  when(push && count === 1.U && !pop) { tail := io.enq.bits }
  assert(count <= 2.U)
}
