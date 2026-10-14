package deltaflow.memory

import chisel3._
import chisel3.util._

class BankWrite(depth: Int, wordBytes: Int) extends Bundle {
  val address = UInt(log2Ceil(depth).W)
  val data = UInt((8 * wordBytes).W)
  val mask = UInt(wordBytes.W)
}

/** Inferred 1R1W synchronous SRAM per bank, with two reserved response slots. No reset of SRAM
  * contents. Only initialized addresses may be read. A simultaneous same-address write wins: the
  * read request is backpressured.
  */
class BankedScratchpad(val banks: Int, val depth: Int, val wordBytes: Int = 16) extends Module {
  require(banks >= 1 && banks <= 32)
  require(depth >= 2 && depth <= 4096)
  require(wordBytes >= 1 && wordBytes <= 32 && isPow2(wordBytes))
  val io = IO(new Bundle {
    val read = Flipped(Vec(banks, Decoupled(UInt(log2Ceil(depth).W))))
    val response = Vec(banks, Decoupled(UInt((wordBytes * 8).W)))
    val write = Flipped(Vec(banks, Decoupled(new BankWrite(depth, wordBytes))))
  })
  for (b <- 0 until banks) {
    val memory = SyncReadMem(depth, Vec(wordBytes, UInt(8.W)))
    val responses = Module(new Queue(UInt((wordBytes * 8).W), 2, pipe = true))
    val pending = RegNext(io.read(b).fire, false.B)
    io.write(b).ready := io.write(b).bits.address < depth.U
    val collision = io.write(b).fire && io.write(b).bits.address === io.read(b).bits
    val occupancy = responses.io.count +& pending.asUInt
    io.read(b).ready :=
      io.read(b).bits < depth.U && !collision && (occupancy < 2.U || responses.io.deq.fire)
    val readData = memory.read(io.read(b).bits, io.read(b).fire)
    when(io.write(b).fire) {
      memory.write(
        io.write(b).bits.address,
        io.write(b).bits.data.asTypeOf(Vec(wordBytes, UInt(8.W))),
        io.write(b).bits.mask.asBools
      )
    }
    responses.io.enq.valid := pending
    responses.io.enq.bits := readData.asUInt
    when(pending) { assert(responses.io.enq.ready, "unreserved SRAM response") }
    io.response(b) <> responses.io.deq
  }
}
