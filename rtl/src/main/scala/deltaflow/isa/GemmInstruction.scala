package deltaflow.isa
import chisel3._
class GemmFields extends Bundle {
  val opcode = UInt(8.W)
  val tileMask = UInt(4.W)
  val contextMask = UInt(4.W)
  val tag = UInt(16.W)
  val kCount = UInt(16.W)
  val inputBase = UInt(16.W)
  val outputBase = UInt(16.W)
  val mValid = UInt(16.W)
  val nValid = UInt(16.W)
  val reserved = UInt(16.W)
}
object GemmFields {
  def decode(x: UInt): GemmFields = {
    val f = Wire(new GemmFields)
    f.opcode := x(7, 0)
    f.tileMask := x(11, 8)
    f.contextMask := x(15, 12)
    f.tag := x(31, 16)
    f.kCount := x(47, 32)
    f.inputBase := x(63, 48)
    f.outputBase := x(79, 64)
    f.mValid := x(95, 80)
    f.nValid := x(111, 96)
    f.reserved := x(127, 112)
    f
  }
}
class GemmCompletion extends Bundle {
  val tile = UInt(2.W)
  val tag = UInt(16.W)
  val opcode = UInt(8.W)
  val error = Bool()
}
class GemmLoad extends Bundle {
  val group = Bool()
  val address = UInt(9.W)
  val data = UInt(256.W)
}
class GemmHostRead extends Bundle {
  val bank = UInt(4.W)
  val address = UInt(9.W)
}
