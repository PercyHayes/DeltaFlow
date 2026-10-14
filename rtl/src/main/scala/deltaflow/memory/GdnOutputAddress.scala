package deltaflow.memory

import chisel3._
import chisel3.util.Cat

/** Packed O pages are 576..639 and 704..767. One BF16 value half occupies each bank half of a row;
  * the Work address low bit selects that value half. For chunk <=64 the row has six bits and cannot
  * carry into the page prefix.
  */
object GdnOutputAddress {
  def apply(workAddress: UInt, slot: Bool): UInt = {
    require(workAddress.getWidth == 9)
    Cat("b10".U(2.W), slot, 1.U(1.W), workAddress(6, 1))
  }
}

/** Local proof probes, not additional modules in the compute system. */
class GdnOutputAddressProbe(val encoded: Boolean) extends Module {
  override def desiredName = if (encoded) "GdnOutputAddressEncoded" else "GdnOutputAddressReference"
  val io = IO(new Bundle {
    val workAddress = Input(UInt(9.W))
    val slot = Input(Bool())
    val address = Output(UInt(10.W))
  })
  io.address :=
    (if (encoded) GdnOutputAddress(io.workAddress, io.slot)
     else Mux(io.slot, 704.U, 576.U) + (io.workAddress >> 1))
}
