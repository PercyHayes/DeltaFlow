package deltaflow.sa
import chisel3._
import chisel3.util._

/** Two real microarrays to verify one arithmetic successor plus three independent observers. Not a
  * full tile: b16 has 512 main MAC, not 4096. No extra PE C32. Returned producer rows use the same
  * reserved 65-slot publisher as the unit.
  */
class PublishedWavePair(val side: Int = 16, val depth: Int = 65) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new RetimedPanelInput(side, 2)))
    val producerCoefficient = Flipped(Decoupled(new WaveCoefficient(side)))
    val followerCoefficient = Flipped(Decoupled(new WaveCoefficient(side)))
    val ticket = Output(new CompletionTicket(depth))
    val request = Flipped(Vec(4, Decoupled(new CompletionTicket(depth))))
    val rhs = Input(Vec(side, UInt(32.W)))
    val launchEnable = Input(Bool())
    val observer = Vec(3, Decoupled(new PublishedVector(side)))
    val out = Decoupled(new RetimedResult(side, 2))
    val reserved = Output(UInt(log2Ceil(depth + 1).W))
    val idle = Output(Bool())
    val returned = Output(Bool())
    val captured = Output(UInt(4.W))
    val launched = Output(Bool())
  })
  val producer = Module(new RetimedWavePanel(side))
  val follower = Module(new RetimedWavePanel(side))
  val publish = Module(new CompletionPublisher(side, depth))
  for (p <- Seq(producer, follower)) {
    p.io.seed.valid := false.B
    p.io.seed.bits := 0.U.asTypeOf(p.io.seed.bits)
    p.io.clear.valid := false.B
    p.io.clear.bits := 0.U
    p.io.read.valid := false.B
    p.io.read.bits := 0.U.asTypeOf(p.io.read.bits)
    p.io.response.ready := true.B
  }
  producer.io.coefficient <> io.producerCoefficient
  follower.io.coefficient <> io.followerCoefficient
  publish.io.reserve.valid := io.in.valid && producer.io.in.ready
  publish.io.reserve.bits := "b1111".U
  producer.io.in.valid := io.in.valid && publish.io.reserve.ready
  producer.io.in.bits := io.in.bits
  producer.io.in.bits.tag := publish.io.ticket.key(1, 0)
  io.in.ready := producer.io.in.ready && publish.io.reserve.ready
  when(io.in.valid) { assert(!io.in.bits.os, "publication pair accepts CS producers") }
  io.ticket := publish.io.ticket
  publish.io.returned.valid := producer.io.out.valid
  publish.io.returned.bits := producer.io.out.bits.values
  producer.io.out.ready := publish.io.returned.ready
  when(producer.io.out.valid) {
    assert(publish.io.returned.ready, "unreserved producer return")
    assert(producer.io.out.bits.tag === publish.io.expectedKey(1, 0))
  }
  publish.io.request <> io.request
  follower.io.in.valid := publish.io.response(0).valid && io.launchEnable
  publish.io.response(0).ready := follower.io.in.ready && io.launchEnable
  follower.io.in.bits.os := false.B
  follower.io.in.bits.context := 0.U
  follower.io.in.bits.tag := publish.io.response(0).bits.key(1, 0)
  follower.io.in.bits.north := VecInit(publish.io.response(0).bits.values.map(_(15, 0)))
  // Caller holds RHS for the corresponding response key until follower fire.
  follower.io.in.bits.west := io.rhs
  for (c <- 1 until 4) io.observer(c - 1) <> publish.io.response(c)
  io.out <> follower.io.out
  io.reserved := publish.io.occupied
  io.idle := producer.io.idle && follower.io.idle && publish.io.idle
  io.returned := publish.io.returned.fire
  io.captured := publish.io.captured
  io.launched := follower.io.in.fire
}
