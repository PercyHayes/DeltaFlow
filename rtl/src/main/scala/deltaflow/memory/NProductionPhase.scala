package deltaflow.memory

import chisel3._

/** Exact current phase, captured from the original state registers' next values. Retire overrides
  * start for active; pack events override start for their flags, matching the backend's original
  * assignment order even on coincident events. This control bit adds no data slot or visible
  * execution cycle.
  */
object NProductionPhase {
  def apply(
      active: Bool,
      started: Bool,
      finished: Bool,
      start: Bool,
      packStart: Bool,
      packDone: Bool,
      retire: Bool
  ): Bool = {
    val nextActive = !retire && (start || active)
    val nextStarted = packStart || (!start && started)
    val nextFinished = packDone || (!start && finished)
    val producing = RegInit(false.B)
    producing := nextActive && nextStarted && !nextFinished
    assert(
      producing === (active && started && !finished),
      "N production phase differs from the original backend state"
    )
    producing
  }
}

/** Uses the actual original update priority, also for adversarial event pairs. */
class NProductionPhaseProbe(val registered: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val packStart = Input(Bool())
    val packDone = Input(Bool())
    val retire = Input(Bool())
    val producing = Output(Bool())
  })
  val active = RegInit(false.B)
  val started = RegInit(false.B)
  val finished = RegInit(false.B)
  when(io.start) {
    active := true.B
    started := false.B
    finished := false.B
  }
  when(io.packStart) { started := true.B }
  when(io.packDone) { finished := true.B }
  when(io.retire) { active := false.B }
  io.producing :=
    (if (registered) {
       NProductionPhase(
         active,
         started,
         finished,
         io.start,
         io.packStart,
         io.packDone,
         io.retire
       )
     } else active && started && !finished)
}
