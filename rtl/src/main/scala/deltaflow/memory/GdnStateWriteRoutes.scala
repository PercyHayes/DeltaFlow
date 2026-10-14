package deltaflow.memory

import chisel3._
import chisel3.util._

/** FP32 H rows occupy Work 256..511 (256..319 at side 4). The caller supplies a valid H row; the
  * fixed page bit cannot depend on H/O arbitration.
  */
object GdnStateAddress {
  def apply(row: UInt, half: Bool, side: Int): UInt = {
    require(Set(4, 16).contains(side))
    val rowBits = log2Ceil(8 * side)
    require(row.getWidth >= rowBits)
    val prefix = 256 >> (rowBits + 1)
    Cat(prefix.U((8 - rowBits).W), row(rowBits - 1, 0), half)
  }
}

/** In epilogue mode a selected FP32 return is always H. The other selected result comes from the
  * output engine. No selected-row mux is needed here.
  */
object GdnWriteSelection {
  def output(engineRow: UInt, selectState: Bool, side: Int): Bool = {
    require(Set(4, 16).contains(side))
    !selectState && engineRow >= (8 * side).U
  }

  def bankValid(
      stateValid: Bool,
      outputValid: Bool,
      stateRow: UInt,
      keyColumns: UInt,
      bank: Int,
      side: Int
  ): Bool = {
    bankValid(stateValid, outputValid, stateRow < keyColumns, bank, side)
  }

  /** H range may have been captured with its existing return tag. */
  def bankValid(
      stateValid: Bool,
      outputValid: Bool,
      withinRange: Bool,
      bank: Int,
      side: Int
  ): Bool = {
    require(Set(4, 16).contains(side) && bank >= 0 && bank < 16)
    val state = if (bank < side) stateValid && withinRange else false.B
    val output = if (bank < side / 2) outputValid else false.B
    state || output
  }
}
