package deltaflow.sa
import chisel3._
import chisel3.util._
import deltaflow.memory._

/** One physical shared scratchpad supplies the two mixed arrays and stores CS results. Inputs
  * occupy words [0,6144); results [6144,8192). Reset cancels the entire frame; SRAM is
  * uninitialized until explicitly written by the caller.
  */
class BankedReadyPair(val side: Int = 16, val frames: Int = 4) extends Module {
  val io = IO(new Bundle {
    val request = Flipped(Vec(4, Decoupled(new VectorAddress)))
    val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val commitEnable = Input(Bool())
    val osEnable = Input(Bool())
    val coefficient = Flipped(Vec(2, Decoupled(new WaveCoefficient(side))))
    val seed = Flipped(Vec(2, Decoupled(new RetimedRowData(side))))
    val read = Flipped(Vec(2, Decoupled(new RetimedRow(side))))
    val response = Vec(2, Decoupled(new RetimedRowData(side)))
    val publicationRequest = Flipped(Vec(4, Decoupled(new CompletionTicket(65))))
    val observer = Vec(3, Decoupled(new PublishedVector(side)))
    val hostRead = Flipped(Decoupled(UInt(9.W)))
    val hostResponse = Decoupled(Vec(side, UInt(32.W)))
    val idle = Output(Bool())
    val issue = Output(Vec(2, Valid(new RetimedPanelInput(side, 2))))
    val result = Valid(new RetimedResult(side, 2))
    val returned = Output(Bool())
    val captured = Output(UInt(4.W))
    val occupied = Output(UInt(7.W))
    val advance = Output(Vec(2, Bool()))
    val fetched = Output(UInt(4.W))
    val fed = Output(UInt(4.W))
    val bankReads = Output(UInt(16.W))
    val bankReturns = Output(UInt(16.W))
    val conflict = Output(UInt(4.W))
    val capacity = Output(UInt(4.W))
    val frameCounts = Output(Vec(4, UInt(log2Ceil(frames + 1).W)))
  })
  val reader = Module(new BankedVectorReader(side, frames))
  val pair = Module(new MixedPublishedPair(side, cutResponseReady = true))
  val hostPending = RegInit(false.B)
  val writes = RegInit(0.U(10.W))
  val idle = reader.io.idle && pair.io.idle && !hostPending
  io.idle := idle
  reader.io.bankEnable := io.bankEnable
  val requestAny = io.request.map(_.valid).reduce(_ || _)
  val controlPermit = RegInit(true.B)
  when(io.request.map(_.fire).reduce(_ || _) || io.hostRead.fire) { controlPermit := false.B }
    .elsewhen(idle) { controlPermit := true.B }
  for (p <- 0 until 4) {
    reader.io.request(p).valid := io.request(p).valid && !hostPending && !io.hostRead.valid
    reader.io.request(p).bits := io.request(p).bits
    io.request(p).ready := reader.io.request(p).ready && !hostPending && !io.hostRead.valid
    when(io.request(p).fire) { assert(io.request(p).bits.word +& reader.sizes(p).U <= 6144.U) }
  }
  val hostAllowed = controlPermit && !requestAny && io.hostRead.bits < writes
  when(io.hostRead.valid && hostAllowed) {
    reader.io.request(3).valid := true.B
    reader.io.request(3).bits.word := 6144.U + io.hostRead.bits * (side / 4).U
    reader.io.request(3).bits.key := 0.U
  }
  io.hostRead.ready := hostAllowed && reader.io.request(3).ready
  when(io.hostRead.fire) { hostPending := true.B }
  io.hostResponse.valid := hostPending && reader.io.response(3).valid
  when(io.hostResponse.fire) { hostPending := false.B }
  def bits(p: Int) = Cat(reader.io.response(p).bits.data.reverse)
  pair.io.cs.valid := reader.io.response(0).valid
  reader.io.response(0).ready := pair.io.cs.ready
  pair.io.cs.bits.os := false.B
  pair.io.cs.bits.context := 0.U
  pair.io.cs.bits.tag := reader.io.response(0).bits.key(1, 0)
  for (a <- 0 until 2) {
    pair.io.os(a).valid := reader.io.response(a + 1).valid
    reader.io.response(a + 1).ready := pair.io.os(a).ready
    pair.io.os(a).bits.os := true.B
    pair.io.os(a).bits.context := reader.io.response(a + 1).bits.key(1, 0)
    pair.io.os(a).bits.tag := reader.io.response(a + 1).bits.key(1, 0)
  }
  pair.io.rhs.valid := reader.io.response(3).valid && !hostPending
  reader.io.response(3).ready := Mux(hostPending, io.hostResponse.ready, pair.io.rhs.ready)
  pair.io.rhs.bits.key := reader.io.response(3).bits.key
  for (k <- 0 until side) {
    pair.io.cs.bits.north(k) := bits(0)(16 * k + 15, 16 * k)
    pair.io.cs.bits.west(k) := bits(0)(16 * side + 32 * k + 31, 16 * side + 32 * k)
    for (a <- 0 until 2) {
      pair.io.os(a).bits.north(k) := bits(a + 1)(16 * k + 15, 16 * k)
      pair.io.os(a).bits.west(k) := bits(a + 1)(16 * side + 16 * k + 15, 16 * side + 16 * k)
    }
    pair.io.rhs.bits.values(k) := bits(3)(32 * k + 31, 32 * k)
    io.hostResponse.bits(k) := bits(3)(32 * k + 31, 32 * k)
  }
  pair.io.osEnable := io.osEnable
  pair.io.request <> io.publicationRequest
  io.observer <> pair.io.observer
  for (a <- 0 until 2) {
    val safe = controlPermit && !requestAny && !hostPending
    pair.io.coefficient(a).valid := io.coefficient(a).valid && safe
    pair.io.coefficient(a).bits := io.coefficient(a).bits
    io.coefficient(a).ready := pair.io.coefficient(a).ready && safe
    pair.io.seed(a).valid := io.seed(a).valid && safe
    pair.io.seed(a).bits := io.seed(a).bits
    io.seed(a).ready := pair.io.seed(a).ready && safe
    pair.io.read(a).valid := io.read(a).valid && safe
    pair.io.read(a).bits := io.read(a).bits
    io.read(a).ready := pair.io.read(a).ready && safe
    io.response(a) <> pair.io.response(a)
  }
  pair.io.result.ready := io.commitEnable
  when(pair.io.result.fire) {
    assert(writes < 512.U)
    writes := writes + 1.U
  }
  for (b <- 0 until 16) {
    val hit = (0 until side / 4).map(w => (6144.U + writes * (side / 4).U + w.U)(3, 0) === b.U)
    val commit = pair.io.result.fire && hit.reduce(_ || _)
    val initAllowed = controlPermit && !requestAny && !io.hostRead.valid
    reader.io.write(b).valid := commit || (io.init(b).valid && initAllowed)
    reader.io.write(b).bits := io.init(b).bits
    when(commit) {
      reader.io.write(b).bits.address := Mux1H(
        hit,
        (0 until side / 4).map(w => (6144.U + writes * (side / 4).U + w.U)(12, 4))
      )
      reader.io.write(b).bits.data := Mux1H(
        hit,
        (0 until side / 4).map(w =>
          Cat((0 until 4).reverse.map(k => pair.io.result.bits.values(w * 4 + k)))
        )
      )
      reader.io.write(b).bits.mask := "hffff".U
      assert(reader.io.write(b).ready)
    }
    io.init(b).ready := initAllowed && reader.io.write(b).ready
  }
  io.issue := pair.io.issue
  io.result.valid := pair.io.result.fire
  io.result.bits := pair.io.result.bits
  io.returned := pair.io.returned
  io.captured := pair.io.captured
  io.occupied := pair.io.occupied
  io.advance := pair.io.advance
  io.fetched := reader.io.granted
  io.fed := VecInit(reader.io.response.map(_.fire)).asUInt
  io.bankReads := reader.io.bankReads
  io.bankReturns := reader.io.bankReturns
  io.conflict := reader.io.conflict
  io.capacity := reader.io.capacity
  io.frameCounts := reader.io.occupied
}
