package deltaflow.sa
import chisel3._
import chisel3.util._

/** Work-conserving CS/OS admission with four enabled-cycle C32 recurrence. A successful-issue phase
  * gives CS one opportunity in three when both streams are eligible; blocked credits/RAW never
  * force an otherwise usable bubble. Input streams must be registered queues. Lock selection during
  * backpressure.
  */
class ReadyWaveIssue(side: Int) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val cs = Flipped(Decoupled(new RetimedPanelInput(side, 2)))
    val os = Flipped(Decoupled(new RetimedPanelInput(side, 2)))
    val out = Decoupled(new RetimedPanelInput(side, 2))
    val allowCS = Input(Bool())
    val advance = Input(Bool())
  })
  val phase = RegInit(0.U(2.W))
  val cooldown = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val locked = RegInit(false.B)
  val lockedCS = Reg(Bool())
  val csOK = io.cs.valid && io.allowCS
  val osOK = io.os.valid && cooldown(io.os.bits.context) === 0.U
  val chooseCS = Mux(locked, lockedCS, csOK && (!osOK || phase === 0.U))
  val valid = Mux(chooseCS, csOK, osOK)
  io.out.valid := valid
  io.out.bits := Mux(chooseCS, io.cs.bits, io.os.bits)
  io.out.bits.os := !chooseCS
  io.cs.ready := chooseCS && valid && io.out.ready
  io.os.ready := !chooseCS && valid && io.out.ready
  when(io.out.valid && !io.out.ready) {
    locked := true.B
    lockedCS := chooseCS
  }
  when(io.out.fire) {
    locked := false.B
    phase := Mux(phase === 2.U, 0.U, phase + 1.U)
  }
  for (i <- 0 until 4) {
    when(io.advance) {
      when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
      when(io.out.fire && io.out.bits.os && io.out.bits.context === i.U) { cooldown(i) := 3.U }
    }
  }
  when(io.out.fire) { assert(io.advance) }
}
