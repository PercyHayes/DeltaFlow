package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.sa._

/** Read completed FP32 projection columns directly from the existing C32s. Four independent token
  * groups retain ordered value-column tags. Panel responses and two-entry output queues are the
  * only data storage: no R matrix SRAM, transpose buffer, arithmetic unit or additional PE context.
  * Callers give returned E seeds priority over new reads to avoid a circular dependency between CS
  * multicast retirement and residual input admission.
  */
class ResidualColumnBridge(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W)))
    val enable = Input(Bool())
    val completed = Input(Vec(16, UInt(2.W)))
    val read = Vec(16, Decoupled(new RetimedRow(side)))
    val response = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val result = Vec(4, Decoupled(new PublishedVector(side)))
    val idle = Output(Bool())
  })
  val active = RegInit(false.B)
  val columns = Reg(UInt(8.W))
  val next = RegInit(VecInit(Seq.fill(4)(0.U(8.W))))
  val tags = Seq.fill(4)(Module(new Queue(UInt(8.W), 4, pipe = false, flow = false)))
  val output =
    Seq.fill(4)(Module(new Queue(new PublishedVector(side), 2, pipe = false, flow = false)))
  val empty = tags.map(!_.io.deq.valid).reduce(_ && _) && output.map(!_.io.deq.valid).reduce(_ && _)
  io.start.ready := !active && empty
  io.idle := !active
  when(io.start.fire) {
    assert(io.start.bits > 0.U && io.start.bits <= (8 * side).U)
    active := true.B
    columns := io.start.bits
    for (g <- 0 until 4) next(g) := 0.U
  }
  for (g <- 0 until 4) {
    val tq = tags(g)
    val out = output(g)
    io.result(g) <> out.io.deq
    val panel = (next(g) >> log2Ceil(side))(1, 0)
    val want = active && io.enable && next(g) < columns && tq.io.enq.ready
    for (c <- 0 until 4) {
      val p = io.read(g * 4 + c)
      p.valid := want && panel === c.U && io.completed(g * 4 + c)(next(g)(log2Ceil(4 * side)))
      p.bits.context := Cat(next(g)(log2Ceil(4 * side)), 0.U(1.W))
      p.bits.row := next(g)(log2Ceil(side) - 1, 0)
    }
    val issued = (0 until 4).map(c => io.read(g * 4 + c).fire).reduce(_ || _)
    tq.io.enq.valid := issued
    tq.io.enq.bits := next(g)
    when(issued) { next(g) := next(g) + 1.U }
    val column = tq.io.deq.bits
    val source = (column >> log2Ceil(side))(1, 0)
    val response = io.response((g * 4).U(4.W) + source)
    out.io.enq.valid := active && tq.io.deq.valid && response.valid
    out.io.enq.bits.key := column
    out.io.enq.bits.values := response.bits.values
    tq.io.deq.ready := out.io.enq.fire
    for (c <- 0 until 4)
      io.response(g * 4 + c).ready :=
        active && tq.io.deq.valid && source === c.U && out.io.enq.ready
    when(out.io.enq.fire) {
      assert(response.bits.context === Cat(column(log2Ceil(4 * side)), 0.U(1.W)))
      assert(response.bits.row === column(log2Ceil(side) - 1, 0))
    }
  }
  when(active && next.map(_ === columns).reduce(_ && _) && empty) { active := false.B }
}
