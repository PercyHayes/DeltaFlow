package deltaflow.memory

import chisel3._
import chisel3.util._

/** Standalone memory closure for N publication under concurrent DMA traffic. Not an additional GDN
  * tile or compute array; no N scaling arithmetic here.
  */
class NPanelStore(val depth: Int = 1024) extends Module {
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new NPackCommand(depth)))
    val input = Flipped(Decoupled(new NPackColumn))
    val completed = Decoupled(new NPackedBlock(depth))
    val writeEnable = Input(UInt(16.W))
    val dmaWrite = Flipped(Vec(16, Decoupled(new BankWrite(depth, 16))))
    val read = Flipped(Vec(16, Decoupled(UInt(log2Ceil(depth).W))))
    val response = Vec(16, Decoupled(UInt(128.W)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val committed = Output(UInt(16.W))
  })
  val packer = Module(new NPanelPacker(depth))
  val memory = Module(new SharedScratchpad(16, depth, 16))
  packer.io.start <> io.start
  packer.io.input <> io.input
  io.completed <> packer.io.completed
  io.busy := packer.io.busy
  io.done := packer.io.done
  for (b <- 0 until 16) {
    memory.io.write(0)(b).valid := packer.io.write(b).valid && io.writeEnable(b)
    memory.io.write(0)(b).bits := packer.io.write(b).bits
    packer.io.write(b).ready := memory.io.write(0)(b).ready && io.writeEnable(b)
    memory.io.read(0)(b).valid := false.B
    memory.io.read(0)(b).bits := 0.U
    memory.io.response(0)(b).ready := true.B
    memory.io.write(1)(b) <> io.dmaWrite(b)
    memory.io.read(1)(b) <> io.read(b)
    io.response(b) <> memory.io.response(1)(b)
  }
  io.committed := VecInit(packer.io.write.map(_.fire)).asUInt
}
