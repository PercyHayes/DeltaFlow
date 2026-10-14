package deltaflow.memory

import chisel3._
import chisel3.util._
import deltaflow.sa.Panel

/** C64/DK128/DV128 result formatter for the sixteen 128-bit Operand banks. U retains the original
  * four-BF16 / low-64-bit word format. W packs eight BF16 values per word, alternating the two
  * pairs INSIDE each producer's four-bank group. Mixed U/W producers therefore never share a write
  * bank.
  *
  * Atomic fork: a bank may fire only if every selected bank can fire. This connects to
  * BankedScratchpad, whose write ready is independent of valid. No queue, extra MAC, precision
  * conversion or combinational ready cycle.
  */
class CompactResultWriter extends Module {
  val io = IO(new Bundle {
    val result = Flipped(Vec(4, Decoupled(new Panel(16, 16, 9))))
    val write = Vec(16, Decoupled(new BankWrite(1024, 16)))
    val invalid = Output(Vec(4, Bool()))
  })
  for (g <- 0 until 4) {
    val in = io.result(g)
    val legal = in.bits.tag < 256.U
    val isW = in.bits.tag >= 128.U
    val selected = VecInit((0 until 4).map(j => !isW || in.bits.tag(0) === (j / 2).U))
    val ready = VecInit((0 until 4).map(j => !selected(j) || io.write(4 * g + j).ready))
    in.ready := legal && ready.asUInt.andR
    io.invalid(g) := in.valid && !legal
    for (j <- 0 until 4) {
      val out = io.write(4 * g + j)
      val others = (0 until 4).filter(_ != j).map(k => ready(k)).reduce(_ && _)
      out.valid := in.valid && legal && selected(j) && others
      out.bits.address := Mux(isW, 512.U + ((in.bits.tag - 128.U) >> 1), 256.U + in.bits.tag)
      val u = Cat(0.U(64.W), Cat((0 until 4).reverse.map(r => in.bits.values(j * 4 + r))))
      val w = Cat((0 until 8).reverse.map(r => in.bits.values((j % 2) * 8 + r)))
      out.bits.data := Mux(isW, w, u)
      out.bits.mask := "hffff".U
      when(out.fire) { assert(in.fire, "partial result vector write") }
    }
  }
}
