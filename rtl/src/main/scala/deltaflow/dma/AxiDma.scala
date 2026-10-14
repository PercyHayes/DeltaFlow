package deltaflow.dma
import chisel3._
import chisel3.util._

class AxiAddress extends Bundle {
  val addr = UInt(48.W)
  val id = UInt(1.W)
  val len = UInt(8.W)
  val size = UInt(3.W)
  val burst = UInt(2.W)
  val lock = Bool()
  val cache = UInt(4.W)
  val prot = UInt(3.W)
  val qos = UInt(4.W)
}
class AxiWrite extends Bundle {
  val data = UInt(256.W)
  val strb = UInt(32.W)
  val last = Bool()
}
class AxiRead extends Bundle {
  val data = UInt(256.W)
  val id = UInt(1.W)
  val resp = UInt(2.W)
  val last = Bool()
}
class AxiResponse extends Bundle {
  val id = UInt(1.W)
  val resp = UInt(2.W)
}
class AxiMaster extends Bundle {
  val ar = Decoupled(new AxiAddress)
  val r = Flipped(Decoupled(new AxiRead))
  val aw = Decoupled(new AxiAddress)
  val w = Decoupled(new AxiWrite)
  val b = Flipped(Decoupled(new AxiResponse))
}
class DmaCommand extends Bundle {
  val store = Bool()
  val region = Bool()
  val stride2 = Bool()
  val address = UInt(48.W)
  val local = UInt(16.W)
  val beats = UInt(16.W)
  val tag = UInt(16.W)
}
class DmaLocalAddress extends Bundle {
  val region = Bool()
  val index = UInt(11.W)
}
class DmaLocalWrite extends DmaLocalAddress { val data = UInt(256.W) }
class DmaLocalPort extends Bundle {
  val write = Decoupled(new DmaLocalWrite)
  val read = Decoupled(new DmaLocalAddress)
  val response = Flipped(Decoupled(UInt(256.W)))
}
class DmaCompletion extends Bundle {
  val tag = UInt(16.W)
  val error = Bool()
  val store = Bool()
}

/** One 256-bit port, eight bounded in-flight INCR bursts, AXI ID zero. Burst boundaries are
  * independent of descriptor length and never cross 4KiB. Load completion means SRAM writes
  * accepted; store completion waits for B. One direction per descriptor enforces <=32 payload
  * bytes/port/cycle.
  */
class AxiDma(val maxBurst: Int = 16, val maxInflight: Int = 8) extends Module {
  require(maxBurst >= 1 && maxBurst <= 128 && isPow2(maxBurst))
  require(maxInflight >= 1 && maxInflight <= 16)
  val io = IO(new Bundle {
    val command = Flipped(Decoupled(new DmaCommand))
    val completion = Decoupled(new DmaCompletion)
    val axi = new AxiMaster
    val local = new DmaLocalPort
    val idle = Output(Bool())
  })
  val commands = Module(new Queue(new DmaCommand, 2))
  commands.io.enq <> io.command
  val idle :: run :: finish :: Nil = Enum(3)
  val state = RegInit(idle)
  val cmd = Reg(new DmaCommand)
  val error = RegInit(false.B)
  val issued = RegInit(0.U(17.W))
  val consumed = RegInit(0.U(17.W))
  val fetched = RegInit(0.U(17.W))
  // Capture store-read eligibility at the descriptor/read handshake. This
  // removes the live 17-bit range comparison from scratchpad arbitration.
  // It changes neither the fetched address nor the four existing credits.
  val fetchAvailable = RegInit(false.B)
  val inflight = RegInit(0.U(log2Ceil(maxInflight + 1).W))
  val writableResponses = RegInit(0.U(log2Ceil(maxInflight + 1).W))
  val lengths = Module(new Queue(UInt(8.W), maxInflight))
  val inBurst = RegInit(0.U(8.W))
  val address = Reg(UInt(48.W))
  val returns = Module(new Queue(UInt(256.W), 4))
  val readCredits = RegInit(0.U(3.W))
  commands.io.deq.ready := state === idle
  when(commands.io.deq.fire) {
    val c = commands.io.deq.bits
    cmd := c
    address := c.address
    issued := 0.U
    consumed := 0.U
    fetched := 0.U
    inBurst := 0.U
    error := false.B
    fetchAvailable := c.beats =/= 0.U
    val end = c.address +& (c.beats << 5)
    val invalid = c
      .address(4, 0)
      .orR || c.beats === 0.U || (c.local +& ((c.beats - 1.U) << c.stride2.asUInt)) >= Mux(
      c.region,
      1024.U,
      2048.U
    ) || end(48)
    error := invalid
    state := Mux(invalid, finish, run)
    assert(
      inflight === 0.U && readCredits === 0.U && !lengths.io.deq.valid && !returns.io.deq.valid
    )
  }
  io.completion.valid := state === finish
  io.completion.bits.tag := cmd.tag
  io.completion.bits.error := error
  io.completion.bits.store := cmd.store
  when(io.completion.fire) { state := idle }
  io.idle := state === idle && !commands.io.deq.valid
  val remaining = cmd.beats - issued
  val boundary = 128.U(9.W) - Cat(0.U(2.W), address(11, 5))
  val limited = Mux(remaining < maxBurst.U, remaining, maxBurst.U)
  val burst = Mux(limited < boundary, limited, boundary)
  val offer =
    state === run && issued < cmd.beats && inflight < maxInflight.U && lengths.io.enq.ready
  for (a <- Seq(io.axi.ar, io.axi.aw)) {
    a.bits := 0.U.asTypeOf(a.bits)
    a.bits.addr := address
    a.bits.len := burst - 1.U
    a.bits.size := 5.U
    a.bits.burst := 1.U
  }
  io.axi.ar.valid := offer && !cmd.store
  io.axi.aw.valid := offer && cmd.store
  val accept = io.axi.ar.fire || io.axi.aw.fire
  lengths.io.enq.valid := accept
  lengths.io.enq.bits := burst
  when(accept) {
    issued := issued + burst
    address := address + (burst << 5)
  }
  val last = inBurst + 1.U === lengths.io.deq.bits
  val readData = state === run && !cmd.store && lengths.io.deq.valid
  io.local.write.valid := readData && io.axi.r.valid
  io.local.write.bits.region := cmd.region
  io.local.write.bits.index := cmd.local
  io.local.write.bits.data := io.axi.r.bits.data
  io.axi.r.ready := readData && io.local.write.ready
  when(io.axi.r.fire) {
    assert(io.axi.r.bits.id === 0.U && io.axi.r.bits.last === last, "AXI read framing/ID violation")
    when(io.axi.r.bits.resp =/= 0.U) { error := true.B }
  }
  io.local.read.valid := state === run && cmd.store && fetchAvailable && readCredits < 4.U
  io.local.read.bits.region := cmd.region
  io.local.read.bits.index := cmd.local
  // The descriptor's local field becomes the current SRAM beat address.
  // Loads advance on accepted SRAM writes; stores advance on accepted reads,
  // independently of AXI W retirement. Keep stride/add off SRAM arbitration.
  // Descriptor capture occurs only in idle, local transfers only in run.
  when(io.local.read.fire || io.local.write.fire) {
    cmd.local := cmd.local + Mux(cmd.stride2, 2.U, 1.U)
  }
  when(io.local.read.fire) {
    fetched := fetched + 1.U
    fetchAvailable := fetched + 1.U < cmd.beats
  }
  when(state === run && cmd.store) {
    assert(fetchAvailable === (fetched < cmd.beats), "store fetch eligibility diverged")
  }
  returns.io.enq <> io.local.response
  io.axi.w.valid := state === run && cmd.store && lengths.io.deq.valid && returns.io.deq.valid
  io.axi.w.bits.data := returns.io.deq.bits
  io.axi.w.bits.strb := "hffffffff".U
  io.axi.w.bits.last := last
  returns.io.deq.ready := io.axi.w.ready && state === run && cmd.store && lengths.io.deq.valid
  val dataFire = io.axi.r.fire || io.axi.w.fire
  val burstDone = dataFire && last
  lengths.io.deq.ready := burstDone
  when(dataFire) {
    consumed := consumed + 1.U
    inBurst := Mux(last, 0.U, inBurst + 1.U)
  }
  when(io.local.read.fire =/= io.axi.w.fire) {
    readCredits := Mux(io.local.read.fire, readCredits + 1.U, readCredits - 1.U)
  }
  when(io.local.response.fire) { assert(readCredits =/= 0.U, "unsolicited SRAM response") }
  io.axi.b.ready := state === run && cmd.store && writableResponses =/= 0.U
  when(io.axi.b.fire) {
    assert(io.axi.b.bits.id === 0.U)
    when(io.axi.b.bits.resp =/= 0.U) { error := true.B }
  }
  val writeDone = io.axi.w.fire && last
  when(writeDone =/= io.axi.b.fire) {
    writableResponses := Mux(writeDone, writableResponses + 1.U, writableResponses - 1.U)
  }
  val retire = Mux(cmd.store, io.axi.b.fire, io.axi.r.fire && last)
  when(accept =/= retire) { inflight := Mux(accept, inflight + 1.U, inflight - 1.U) }
  when(state === run && issued === cmd.beats && consumed === cmd.beats && inflight === 0.U) {
    state := finish
  }
  assert(inflight <= maxInflight.U && readCredits <= 4.U)
}
