package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.memory.{BankedScratchpad, BankWrite}
import deltaflow.sa.{BlockCoefficient, SolvedColumn, TwoBlockCore}

class BlockCommand(depth: Int, maxColumns: Int) extends Bundle {
  val sourceBase = UInt(log2Ceil(depth).W)
  val destinationBase = UInt(log2Ceil(depth).W)
  val columns = UInt(log2Ceil(maxColumns + 1).W)
}

/** Finite two-block engine. Host ports own SRAM while idle; execution owns it while busy.
  * Source/destination ranges must be disjoint. A result transfers atomically to the SRAM write port
  * and to the externally backpressured tap.
  */
class TwoBlockEngine(val side: Int = 4, val depth: Int = 1024, val maxColumns: Int = 256)
    extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  require(depth >= 2 * maxColumns && depth <= 4096)
  require(maxColumns >= 1 && maxColumns <= 256)
  val banksPerBlock = side / 4
  val bankCount = 2 * banksPerBlock
  val tagBits = log2Ceil(maxColumns + 1)
  val io = IO(new Bundle {
    val coefficient = Flipped(Decoupled(new BlockCoefficient(side)))
    val command = Flipped(Decoupled(new BlockCommand(depth, maxColumns)))
    val hostWrite = Flipped(Vec(bankCount, Decoupled(new BankWrite(depth, 16))))
    val hostRead = Flipped(Vec(bankCount, Decoupled(UInt(log2Ceil(depth).W))))
    val hostResponse = Vec(bankCount, Decoupled(UInt(128.W)))
    val result = Decoupled(new SolvedColumn(side, tagBits))
    val busy = Output(Bool())
    val done = Output(Bool())
    val invalidCommand = Output(Bool())
  })
  val core = Module(new TwoBlockCore(side, tagBits))
  val memory = Module(new BankedScratchpad(bankCount, depth))
  val tags = Module(new Queue(UInt(tagBits.W), 4))
  val busy = RegInit(false.B)
  val source = Reg(UInt(log2Ceil(depth).W))
  val destination = Reg(UInt(log2Ceil(depth).W))
  val columns = Reg(UInt(tagBits.W))
  val issued = RegInit(0.U(tagBits.W))
  // Also prevent a command from overtaking host requests or host read responses.
  val hostOutstanding = RegInit(VecInit(Seq.fill(bankCount)(0.U(3.W))))
  val noHostTraffic = !hostOutstanding.map(_ =/= 0.U).reduce(_ || _) &&
    !io.hostRead.map(_.valid).reduce(_ || _) && !io.hostWrite.map(_.valid).reduce(_ || _)
  val sourceEnd = io.command.bits.sourceBase +& io.command.bits.columns
  val destinationEnd = io.command.bits.destinationBase +& io.command.bits.columns
  val commandLegal = io.command.bits.columns > 0.U && io.command.bits.columns <= maxColumns.U &&
    sourceEnd <= depth.U && destinationEnd <= depth.U &&
    (sourceEnd <= io.command.bits.destinationBase || destinationEnd <= io.command.bits.sourceBase)
  io.command.ready := !busy && core.io.idle && core.io.coefficientsReady && noHostTraffic &&
    !io.coefficient.valid && commandLegal
  io.invalidCommand := io.command.valid && !commandLegal
  core.io.coefficient.valid := io.coefficient.valid && !busy && !io.command.fire
  core.io.coefficient.bits := io.coefficient.bits
  io.coefficient.ready := core.io.coefficient.ready && !busy && !io.command.fire
  when(io.command.fire) {
    source := io.command.bits.sourceBase
    destination := io.command.bits.destinationBase
    columns := io.command.bits.columns
    issued := 0.U
    busy := true.B
  }
  val allReadReady = memory.io.read.map(_.ready).reduce(_ && _)
  val allResponses = memory.io.response.map(_.valid).reduce(_ && _)
  val allWriteReady = memory.io.write.map(_.ready).reduce(_ && _)
  val issue = busy && issued < columns && tags.io.enq.ready && allReadReady
  tags.io.enq.valid := issue
  tags.io.enq.bits := issued
  when(issue) { issued := issued + 1.U }
  core.io.in.valid := busy && allResponses && tags.io.deq.valid
  core.io.in.bits.tag := tags.io.deq.bits
  tags.io.deq.ready := busy && allResponses && core.io.in.ready
  for (r <- 0 until side) {
    core.io.in.bits.r0(r) := memory.io.response(r / 4).bits(32 * (r % 4) + 31, 32 * (r % 4))
    core.io.in.bits
      .r1(r) := memory.io.response(banksPerBlock + r / 4).bits(32 * (r % 4) + 31, 32 * (r % 4))
  }
  io.result.valid := busy && core.io.out.valid && allWriteReady
  io.result.bits := core.io.out.bits
  core.io.out.ready := busy && allWriteReady && io.result.ready
  val finalResult = io.result.fire && core.io.out.bits.tag === columns - 1.U
  io.done := RegNext(finalResult, false.B)
  when(finalResult) { busy := false.B }
  for (b <- 0 until bankCount) {
    val write = memory.io.write(b)
    val read = memory.io.read(b)
    val response = memory.io.response(b)
    val output = if (b < banksPerBlock) core.io.out.bits.x0 else core.io.out.bits.x1
    val first = (b % banksPerBlock) * 4
    val packed = Cat((0 until 4).reverse.map(i => output(first + i)))
    write.valid := Mux(
      busy,
      core.io.out.valid && io.result.ready && allWriteReady,
      io.hostWrite(b).valid
    )
    write.bits := io.hostWrite(b).bits
    when(busy) {
      write.bits.address := destination + core.io.out.bits.tag
      write.bits.data := Cat(0.U(64.W), packed)
      write.bits.mask := "hffff".U
    }
    io.hostWrite(b).ready := !busy && write.ready
    read.valid := Mux(busy, issue, io.hostRead(b).valid)
    read.bits := Mux(busy, source + issued, io.hostRead(b).bits)
    io.hostRead(b).ready := !busy && read.ready
    io.hostResponse(b).valid := !busy && response.valid
    io.hostResponse(b).bits := response.bits
    response.ready := Mux(
      busy,
      core.io.in.ready && allResponses && tags.io.deq.valid,
      io.hostResponse(b).ready
    )
    when(io.hostRead(b).fire =/= io.hostResponse(b).fire) {
      when(io.hostRead(b).fire) { hostOutstanding(b) := hostOutstanding(b) + 1.U }
        .otherwise { hostOutstanding(b) := hostOutstanding(b) - 1.U }
    }
    when(io.hostResponse(b).fire) { assert(hostOutstanding(b) =/= 0.U) }
    when(busy) { assert(hostOutstanding(b) === 0.U) }
  }
  io.busy := busy
}
