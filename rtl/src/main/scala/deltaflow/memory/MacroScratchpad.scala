package deltaflow.memory
import chisel3._
import chisel3.util._

/** Simulation body is synthesizable; area flow blackboxes this exact macro name. Byte masks and
  * one-cycle read match BankedScratchpad's inferred memory.
  */
class ScratchpadSram(depth: Int, bytes: Int)
    extends BlackBox(Map("DEPTH" -> depth, "BYTES" -> bytes))
    with HasBlackBoxInline {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val ren = Input(Bool())
    val raddr = Input(UInt(log2Ceil(depth).W))
    val rdata = Output(UInt((bytes * 8).W))
    val wen = Input(Bool())
    val waddr = Input(UInt(log2Ceil(depth).W))
    val wdata = Input(UInt((bytes * 8).W))
    val wmask = Input(UInt(bytes.W))
  })
  setInline(
    "ScratchpadSram.sv",
    """
module ScratchpadSram #(
    parameter DEPTH = 1024,
    parameter BYTES = 16
) (
    input clock,
    input ren,
    input wen,
    input [$clog2(DEPTH)-1:0] raddr,
    input [$clog2(DEPTH)-1:0] waddr,
    input [8*BYTES-1:0] wdata,
    input [BYTES-1:0] wmask,
    output reg [8*BYTES-1:0] rdata
);
    reg [8*BYTES-1:0] mem [0:DEPTH-1];
    integer lane;

    always @(posedge clock) begin
        if (ren) begin
            rdata <= mem[raddr];
        end
        if (wen) begin
            for (lane = 0; lane < BYTES; lane = lane + 1) begin
                if (wmask[lane]) begin
                    mem[waddr][8*lane +: 8] <= wdata[8*lane +: 8];
                end
            end
        end
    end
endmodule
"""
  )
}

class ScratchpadPort(banks: Int, depth: Int, wordBytes: Int = 16) extends Bundle {
  val read = Flipped(Vec(banks, Decoupled(UInt(log2Ceil(depth).W))))
  val response = Vec(banks, Decoupled(UInt((wordBytes * 8).W)))
  val write = Flipped(Vec(banks, Decoupled(new BankWrite(depth, wordBytes))))
}

/** Same 1R1W reservation/collision policy as BankedScratchpad, explicit macro. Only SRAM contents
  * are blackboxed: response queues and control remain logic.
  */
class MacroScratchpadIO(
    banks: Int,
    depth: Int,
    wordBytes: Int,
    earlyWriteConflict: Boolean
) extends ScratchpadPort(banks, depth, wordBytes) {
  val writeConflict = if (earlyWriteConflict) Some(Input(Vec(banks, Bool()))) else None
}

class MacroScratchpad(
    val banks: Int,
    val depth: Int,
    val wordBytes: Int = 16,
    val registeredAdmission: Boolean = false,
    val earlyWriteConflict: Boolean = false
) extends Module {
  require(banks >= 1 && banks <= 32 && depth >= 2 && depth <= 4096 && isPow2(depth))
  require(wordBytes >= 1 && wordBytes <= 32 && isPow2(wordBytes))
  val io = IO(new MacroScratchpadIO(banks, depth, wordBytes, earlyWriteConflict))
  for (b <- 0 until banks) {
    val memory = Module(new ScratchpadSram(depth, wordBytes))
    // Three reserved responses sustain one request per cycle with synchronous
    // SRAM, without borrowing the consumer's current-cycle ready signal.
    val responseDepth = if (registeredAdmission) 3 else 2
    val responses = Module(
      new Queue(UInt((wordBytes * 8).W), responseDepth, pipe = !registeredAdmission)
    )
    val pending = RegNext(io.read(b).fire, false.B)
    io.write(b).ready := io.write(b).bits.address < depth.U
    val originalCollision = io.write(b).fire && io.write(b).bits.address === io.read(b).bits
    val collision = io.writeConflict.map(_(b)).getOrElse(originalCollision)
    io.writeConflict.foreach { conflicts =>
      assert(
        conflicts(b) === originalCollision,
        "early SRAM collision differs from actual accepted write"
      )
    }
    val occupancy = responses.io.count +& pending.asUInt
    val room = if (registeredAdmission) {
      val reserved = RegInit(0.U(2.W))
      when(io.read(b).fire =/= responses.io.deq.fire) {
        reserved := Mux(io.read(b).fire, reserved + 1.U, reserved - 1.U)
      }
      assert(reserved === occupancy && reserved <= responseDepth.U)
      reserved < responseDepth.U
    } else occupancy < 2.U || responses.io.deq.fire
    io.read(b).ready := io.read(b).bits < depth.U && !collision && room
    memory.io.clock := clock
    memory.io.ren := io.read(b).fire
    memory.io.raddr := io.read(b).bits
    memory.io.wen := io.write(b).fire
    memory.io.waddr := io.write(b).bits.address
    memory.io.wdata := io.write(b).bits.data
    memory.io.wmask := io.write(b).bits.mask
    responses.io.enq.valid := pending
    responses.io.enq.bits := memory.io.rdata
    when(pending) { assert(responses.io.enq.ready, "unreserved macro response") }
    io.response(b) <> responses.io.deq
  }
}
