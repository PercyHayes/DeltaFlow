package deltaflow.memory

import chisel3._

/** Compare both unmuxed addresses before late compute/DMA permissions arrive. Compute still wins
  * when both valid signals are high. The actual SRAM write address and payload use the existing
  * priority mux; only read collision qualification is distributed. No mutual exclusion assumption
  * is needed.
  */
object DmaWriteConflict {
  def apply(
      computeValid: Bool,
      computeAddress: UInt,
      dmaAccepted: Bool,
      dmaAddress: UInt,
      readAddress: UInt
  ): Bool = {
    require(computeAddress.getWidth == dmaAddress.getWidth)
    require(computeAddress.getWidth == readAddress.getWidth)
    val computeMatch = computeAddress === readAddress
    val dmaMatch = dmaAddress === readAddress
    (computeValid && computeMatch) || (!computeValid && dmaAccepted && dmaMatch)
  }
}
