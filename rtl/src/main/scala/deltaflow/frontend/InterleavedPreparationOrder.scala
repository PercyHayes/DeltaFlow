package deltaflow.frontend

import chisel3._
import chisel3.util._

class PreparationRow(side: Int) extends Bundle {
  val blockRow = UInt(2.W)
  val blockCol = UInt(2.W)
  val row = UInt(log2Ceil(side).W)
}

/** Bounded ready-based producer order. Request remains stable across both scale steps and
  * backpressure; private solver queues preserve increasing row order. No arithmetic resources are
  * added by this controller.
  */
class InterleavedPreparationOrder(side: Int, columnOrder: Boolean = false) extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  val io = IO(new Bundle {
    val start = Input(Bool())
    val enable = Input(Bool())
    val diagonalReady = Input(Vec(4, Bool()))
    val request = Decoupled(new PreparationRow(side))
    val done = Output(Bool())
  })
  val nextRow = RegInit(VecInit(Seq.fill(4)(0.U(log2Ceil(side + 1).W))))
  val offIndex = RegInit(0.U(log2Ceil(6 * side + 1).W))
  val rr = RegInit(0.U(2.W))
  val active = RegInit(false.B)
  val held = Reg(new PreparationRow(side))
  val candidates = Wire(Vec(4, Bool()))
  val indices = Wire(Vec(4, UInt(2.W)))
  for (i <- 0 until 4) {
    indices(i) := rr + i.U
    candidates(i) := nextRow(indices(i)) < side.U && io.diagonalReady(indices(i))
  }
  val diagonal = candidates.asUInt.orR
  val selected = indices(PriorityEncoder(candidates.asUInt))
  val offBlocks =
    if (columnOrder) Seq((1, 0), (2, 0), (3, 0), (2, 1), (3, 1), (3, 2))
    else Seq((1, 0), (2, 0), (2, 1), (3, 0), (3, 1), (3, 2))
  val offRows = VecInit(offBlocks.map(x => x._1.U(2.W)))
  val offCols = VecInit(offBlocks.map(x => x._2.U(2.W)))
  val offBlock = Mux(offIndex < (6 * side).U, (offIndex >> log2Ceil(side))(2, 0), 0.U(3.W))
  val chosen = Wire(new PreparationRow(side))
  chosen.blockRow := Mux(diagonal, selected, offRows(offBlock))
  chosen.blockCol := Mux(diagonal, selected, offCols(offBlock))
  chosen.row := Mux(
    diagonal,
    nextRow(selected)(log2Ceil(side) - 1, 0),
    offIndex(log2Ceil(side) - 1, 0)
  )
  io.done := nextRow.map(_ === side.U).reduce(_ && _) && offIndex === (6 * side).U && !active
  io.request.valid := io.enable && (active || diagonal || offIndex < (6 * side).U)
  io.request.bits := Mux(active, held, chosen)
  when(io.enable && !active && io.request.valid) {
    active := true.B
    held := chosen
    when(diagonal) { rr := selected + 1.U }
  }
  when(io.request.fire) {
    val p = io.request.bits
    active := false.B
    when(p.blockRow === p.blockCol) { nextRow(p.blockRow) := nextRow(p.blockRow) + 1.U }
      .otherwise { offIndex := offIndex + 1.U }
  }
  when(io.start) {
    active := false.B
    rr := 0.U
    offIndex := 0.U
    for (i <- 0 until 4) nextRow(i) := 0.U
  }
  assert(offIndex <= (6 * side).U)
  for (i <- 0 until 4) assert(nextRow(i) <= side.U)
}
