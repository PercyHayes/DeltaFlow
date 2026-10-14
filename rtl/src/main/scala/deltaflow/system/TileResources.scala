package deltaflow.system
import chisel3._
import deltaflow.sa.RetimedPanelPort
import deltaflow.memory.ScratchpadPort

/** Controller-side ports. External-resource controllers contain no main MACs or Operand/Work SRAM.
  * Gate SRAM remains inside the GDN preparation engine.
  */
class TileResources(side: Int) extends Bundle {
  val panels =
    Vec(16, Flipped(new RetimedPanelPort(side, columnSeed = true, osAdmissionStatus = true)))
  val operand = Flipped(new ScratchpadPort(16, 1024))
  val work = Flipped(new ScratchpadPort(16, 512))
}
class ResourceHandle[T <: Data](val io: T)
