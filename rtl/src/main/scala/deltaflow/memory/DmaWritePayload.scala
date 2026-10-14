package deltaflow.memory

import chisel3._
import chisel3.util._

/** Compute retains priority over DMA at each physical SRAM write port. DMA acceptance controls the
  * write enable. Payload selection needs only the compute producer's valid signal, avoiding DMA
  * grant logic before collision comparison. Payload on an invalid write is intentionally
  * unobserved.
  */
object DmaWritePayload {
  def apply(
      computeValid: Bool,
      compute: BankWrite,
      dma: BankWrite,
      dmaAccepted: Bool
  ): ValidIO[BankWrite] = {
    require(compute.getWidth == dma.getWidth)
    val result = Wire(Valid(chiselTypeOf(compute)))
    result.valid := computeValid || dmaAccepted
    result.bits := Mux(computeValid, compute, dma)
    assert(!(computeValid && dmaAccepted), "DMA accepted over a compute write")
    result
  }
}
