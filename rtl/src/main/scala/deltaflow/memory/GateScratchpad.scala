package deltaflow.memory
import chisel3._
import chisel3.util._

class GateWrite(side: Int) extends Bundle {
  val row = UInt(log2Ceil(4 * side).W)
  val block = UInt(2.W)
  val values = Vec(side, UInt(32.W))
}
class GateQuery(side: Int) extends Bundle {
  val column = Bool()
  val fixed = UInt(log2Ceil(4 * side).W)
  val block = UInt(2.W)
  val tag = UInt(6.W)
}
class GateResponse(side: Int) extends Bundle {
  val tag = UInt(6.W)
  val values = Vec(side, UInt(32.W))
}

/** bank=(row+column)%side, address=(row/side)*(4*side)+column. Either a side-element row or column
  * visits every bank exactly once. Actual 1R1W macros; no transpose copy or combinational array of
  * FP32 words.
  */
class GateScratchpad(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val tokens = 4 * side
  val depth = 4 * tokens
  val bankBits = log2Ceil(side)
  val io = IO(new Bundle {
    val write = Flipped(Decoupled(new GateWrite(side)))
    val request = Flipped(Decoupled(new GateQuery(side)))
    val response = Decoupled(new GateResponse(side))
  })
  val memory = Module(new MacroScratchpad(side, depth, 4))
  val tags = Module(new Queue(new GateQuery(side), 2, pipe = false, flow = false))
  val results = Module(new Queue(new GateResponse(side), 2, pipe = false, flow = false))
  io.response <> results.io.deq
  io.write.ready := memory.io.write.map(_.ready).reduce(_ && _)
  val readsReady = memory.io.read.map(_.ready).reduce(_ && _)
  io.request.ready := tags.io.enq.ready && readsReady
  tags.io.enq.valid := io.request.fire
  tags.io.enq.bits := io.request.bits
  val returns = memory.io.response.map(_.valid).reduce(_ && _)
  results.io.enq.valid := tags.io.deq.valid && returns
  results.io.enq.bits.tag := tags.io.deq.bits.tag
  tags.io.deq.ready := results.io.enq.ready && returns
  for (b <- 0 until side) {
    val writeLane = (b.U - io.write.bits.row)(bankBits - 1, 0)
    val col = Cat(io.write.bits.block, writeLane)
    memory.io.write(b).valid := io.write.valid
    memory.io.write(b).bits.address := (io.write.bits.row >> bankBits) * tokens.U + col
    memory.io.write(b).bits.data := io.write.bits.values(writeLane)
    memory.io.write(b).bits.mask := 15.U
    val readLane = (b.U - io.request.bits.fixed)(bankBits - 1, 0)
    val varying = Cat(io.request.bits.block, readLane)
    val r = Mux(io.request.bits.column, varying, io.request.bits.fixed)
    val c = Mux(io.request.bits.column, io.request.bits.fixed, varying)
    memory.io.read(b).bits := (r >> bankBits) * tokens.U + c
    memory.io.read(b).valid := io.request.valid && tags.io.enq.ready && readsReady
    memory.io.response(b).ready := results.io.enq.ready && tags.io.deq.valid && returns
  }
  for (j <- 0 until side) {
    val bank = (tags.io.deq.bits.fixed + j.U)(bankBits - 1, 0)
    results.io.enq.bits.values(j) := memory.io.response(bank).bits
  }
}
