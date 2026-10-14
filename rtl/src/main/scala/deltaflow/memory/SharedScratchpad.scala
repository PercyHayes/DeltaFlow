package deltaflow.memory

import chisel3._
import chisel3.util._

/** Two clients (compute / DMA), each with independent word requests per bank. Independent
  * round-robin read/write arbitration, write wins same-address collision. The existing macro shell
  * reserves two total read responses per bank. An ordered owner queue routes responses; a stalled
  * owner can block that bank, but never another bank. No SRAM reset or implicit initialization.
  * This is a standalone integration component, not yet wired into GDN.
  */
class SharedScratchpad(val banks: Int = 16, val depth: Int = 1024, val wordBytes: Int = 16)
    extends Module {
  require(banks >= 1 && banks <= 32 && depth >= 2 && depth <= 4096 && isPow2(depth))
  require(wordBytes >= 1 && wordBytes <= 32 && isPow2(wordBytes))
  val io = IO(new Bundle {
    val read = Flipped(Vec(2, Vec(banks, Decoupled(UInt(log2Ceil(depth).W)))))
    val write = Flipped(Vec(2, Vec(banks, Decoupled(new BankWrite(depth, wordBytes)))))
    val response = Vec(2, Vec(banks, Decoupled(UInt((8 * wordBytes).W))))
  })
  val memory = Module(new MacroScratchpad(banks, depth, wordBytes))
  for (b <- 0 until banks) {
    val readTurn = RegInit(false.B)
    val writeTurn = RegInit(false.B)
    val readValid = VecInit((0 until 2).map(c => io.read(c)(b).valid))
    val writeValid = VecInit((0 until 2).map(c => io.write(c)(b).valid))
    val readChoice = Mux(readValid(readTurn), readTurn, !readTurn)
    val writeChoice = Mux(writeValid(writeTurn), writeTurn, !writeTurn)
    val owners = Module(new Queue(Bool(), 2, pipe = true))

    memory.io.write(b).valid := writeValid.asUInt.orR
    memory.io.write(b).bits := Mux(writeChoice, io.write(1)(b).bits, io.write(0)(b).bits)
    memory.io.read(b).valid := readValid.asUInt.orR && owners.io.enq.ready
    memory.io.read(b).bits := Mux(readChoice, io.read(1)(b).bits, io.read(0)(b).bits)
    owners.io.enq.valid := memory.io.read(b).fire
    owners.io.enq.bits := readChoice
    owners.io.deq.ready := memory.io.response(b).fire
    memory.io.response(b).ready := owners.io.deq.valid && Mux(
      owners.io.deq.bits,
      io.response(1)(b).ready,
      io.response(0)(b).ready
    )
    for (c <- 0 until 2) {
      io.write(c)(b).ready := (writeChoice === c.U) && memory.io.write(b).ready
      io.read(c)(b).ready := (readChoice === c.U) && memory.io.read(b).ready && owners.io.enq.ready
      io.response(c)(b).valid :=
        memory.io.response(b).valid && owners.io.deq.valid && owners.io.deq.bits === c.U
      io.response(c)(b).bits := memory.io.response(b).bits
    }
    when(memory.io.read(b).fire) { readTurn := !readChoice }
    when(memory.io.write(b).fire) { writeTurn := !writeChoice }
    when(memory.io.response(b).valid) { assert(owners.io.deq.valid, "missing SRAM response owner") }
  }
}
