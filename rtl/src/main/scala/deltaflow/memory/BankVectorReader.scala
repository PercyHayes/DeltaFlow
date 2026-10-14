package deltaflow.memory

import chisel3._
import chisel3.util._

class BankVectorRequest extends Bundle {
  val mask = UInt(16.W)
  val tag = UInt(10.W)
  val addresses = Vec(16, UInt(10.W))
}
class BankVectorResult extends Bundle {
  val tag = UInt(10.W)
  val words = Vec(16, UInt(128.W))
}

/** Ordered gather with independent bank admission. Accepting a descriptor reserves the complete
  * result. Banks may accept on different edges; the descriptor and remaining mask stay local until
  * every requested bank has accepted exactly once. No data FIFO or SRAM is added. With all banks
  * ready, acceptance and response timing match atomic gather.
  */
class BankVectorReader(
    val registeredOccupancy: Boolean = false,
    val captureAtAdmission: Boolean = false
) extends Module {
  val io = IO(new Bundle {
    val request = Flipped(Decoupled(new BankVectorRequest))
    val result = Decoupled(new BankVectorResult)
    val read = Vec(16, Decoupled(UInt(10.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val bankEnable = Input(UInt(16.W))
    val idle = Output(Bool())
  })

  val tags = Module(new Queue(UInt(26.W), 8, pipe = false, flow = false))
  val results = Module(new Queue(new BankVectorResult, 8, pipe = false, flow = false))
  io.result <> results.io.deq
  val reserved = RegInit(0.U(4.W))
  // Count descriptors until their result is consumed, including rows in the
  // original result FIFO. This is exactly reserved + results.count, but does
  // not derive admission/idle from that FIFO's read/write pointer arithmetic.
  // The original tags and all eight payload entries remain unchanged.
  val outstanding = if (registeredOccupancy) Some(RegInit(0.U(4.W))) else None
  val total = outstanding.getOrElse(reserved +& results.io.count)
  outstanding.foreach { count =>
    when(io.request.fire =/= io.result.fire) {
      count := Mux(io.request.fire, count + 1.U, count - 1.U)
    }
    assert(count === (reserved +& results.io.count), "reader occupancy lost a reserved row")
  }

  // Store control fields only for a partially issued vector. Upstream may
  // offer a different request immediately after acceptance, including changes
  // in seed/frame arbitration. It must not change this vector's bank addresses.
  val partial = RegInit(false.B)
  val descriptor = Reg(new BankVectorRequest)
  val remaining = Reg(UInt(16.W))
  val room = tags.io.enq.ready && total < 8.U
  val current = Mux(partial, descriptor, io.request.bits)
  val needed = Mux(partial, remaining, io.request.bits.mask)

  // Admission reserves the existing descriptor and result capacity. Even a
  // completely blocked vector can be accepted: its addresses/mask then stay
  // in the original partial descriptor until the physical banks are ready.
  // No bank-ready -> caller admission -> wide row enqueue chain remains.
  io.request.ready := !partial && room

  // Valid never depends on another bank's ready. Admission can issue zero,
  // some, or all banks immediately; unfinished banks use the same descriptor.
  val offer = partial || (room && io.request.valid)
  for (b <- 0 until 16) {
    io.read(b).valid := offer && needed(b) && io.bankEnable(b)
    io.read(b).bits := current.addresses(b)
  }
  val acceptedBanks = VecInit(io.read.map(_.fire)).asUInt
  val left = needed & ~acceptedBanks
  when(io.request.fire) {
    partial := left.orR
    if (captureAtAdmission) {
      // These fields are read only while partial is set. Capturing on every
      // descriptor acceptance removes the all-bank completion reduction from
      // their register enables. An immediately completed vector leaves an
      // unused snapshot; the next accepted request replaces it before use.
      descriptor := io.request.bits
      remaining := left
    } else {
      when(left.orR) {
        descriptor := io.request.bits
        remaining := left
      }
    }
  }
  when(partial) {
    remaining := left
    when(!left.orR) { partial := false.B }
  }
  when(!partial && acceptedBanks.orR) {
    assert(io.request.fire, "bank read without a reserved vector")
  }
  when(partial) {
    assert(remaining.orR && reserved =/= 0.U, "partial vector lost its reservation")
    assert((acceptedBanks & ~remaining) === 0.U, "bank read accepted twice")
  }

  tags.io.enq.valid := io.request.fire
  tags.io.enq.bits := Cat(io.request.bits.mask, io.request.bits.tag)
  val mask = tags.io.deq.bits(25, 10)
  val returned = (0 until 16)
    .map { b =>
      !mask(b) || io.response(b).valid
    }
    .reduce(_ && _)
  results.io.enq.valid := tags.io.deq.valid && returned
  results.io.enq.bits.tag := tags.io.deq.bits(9, 0)
  tags.io.deq.ready := returned && results.io.enq.ready

  for (b <- 0 until 16) {
    io.response(b).ready := tags.io.deq.valid && mask(b) && returned && results.io.enq.ready
    results.io.enq.bits.words(b) := Mux(mask(b), io.response(b).bits, 0.U)
  }
  when(io.request.fire =/= results.io.enq.fire) {
    reserved := Mux(io.request.fire, reserved + 1.U, reserved - 1.U)
  }
  assert((reserved +& results.io.count) <= 8.U, "vector result capacity exceeded")
  when(results.io.enq.valid) {
    assert(results.io.enq.ready && reserved =/= 0.U, "unreserved vector result")
  }
  io.idle :=
    (if (registeredOccupancy) !partial && total === 0.U
     else !partial && reserved === 0.U && !results.io.deq.valid)
}
