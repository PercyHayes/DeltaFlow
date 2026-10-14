package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic.FloatArithmetic

object StreamPipes {

  /** Elaboration-time bounded registers; no runtime loops or implicit FIFOs. */
  def delay[T <: Data](
      in: ValidIO[T],
      cycles: Int,
      advance: Bool,
      validOnlyReset: Boolean = false
  ): ValidIO[T] = {
    require(cycles >= 0 && cycles <= 256)
    (0 until cycles).foldLeft(in) { (previous, _) =>
      if (validOnlyReset) {
        // Payload is observable only while valid. Reset must invalidate a
        // frozen pipe, but need not reset data that will be overwritten first.
        val payload = Reg(chiselTypeOf(in.bits))
        val valid = RegInit(false.B)
        val next = Wire(chiselTypeOf(in))
        when(advance) {
          payload := previous.bits
          valid := previous.valid
        }
        next.bits := payload
        next.valid := valid
        next
      } else {
        val next = RegInit(0.U.asTypeOf(chiselTypeOf(in)))
        when(advance) { next := previous }
        next
      }
    }
  }
}

/** Fixed row-to-reduction-column wiring, registered after explicit conversion. For this first RTL
  * the MAC latency is one enabled cycle; no extra deskew.
  */
class DiagonalTurn(val lanes: Int) extends Module {
  require(lanes >= 2 && lanes <= 16)
  val io = IO(new Bundle {
    val advance = Input(Bool())
    val in = Flipped(Vec(lanes, Valid(UInt(32.W))))
    val out = Vec(lanes, Valid(UInt(16.W)))
  })
  for (i <- 0 until lanes) {
    val value = Wire(Valid(UInt(16.W)))
    value.valid := io.in(i).valid
    value.bits := FloatArithmetic.narrow(io.in(i).bits)
    io.out(i) := StreamPipes.delay(value, 1, io.advance)
  }
}
