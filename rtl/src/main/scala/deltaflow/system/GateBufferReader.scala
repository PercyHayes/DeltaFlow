package deltaflow.system
import chisel3._
import chisel3.util._
import deltaflow.frontend.GateToken

/** Read activated g/beta from high banks of raw slot rows 252..255. DMA is the only payload
  * ingress. One bounded chunk staging register is reused after its consumer handshake; no extra
  * SRAM or wide input bus.
  */
class GateBufferReader(val side: Int) extends Module {
  val rows = side / 4
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(Bool()))
    val read = Vec(8, Decoupled(UInt(10.W)))
    val response = Flipped(Vec(8, Decoupled(UInt(128.W))))
    val result = Decoupled(Vec(4 * side, new GateToken))
    val error = Output(Bool())
    val idle = Output(Bool())
  })
  val active = RegInit(false.B)
  val full = RegInit(false.B)
  val slot = Reg(Bool())
  val row = RegInit(0.U(2.W))
  val sent = RegInit(0.U(8.W))
  val received = RegInit(0.U(8.W))
  val tokens = Reg(Vec(4 * side, new GateToken))
  val invalid = RegInit(false.B)
  io.start.ready := !active && !full
  io.idle := !active && !full
  when(io.start.fire) {
    active := true.B
    slot := io.start.bits
    row := 0.U
    sent := 0.U
    received := 0.U
    invalid := false.B
  }
  val bad = Wire(Vec(8, Bool()))
  for (i <- 0 until 8) {
    io.read(i).valid := active && !sent(i)
    io.read(i).bits := Mux(slot, 1020.U, 252.U) + row
    io.response(i).ready := active && sent(i) && !received(i)
    bad(i) := io.response(i).fire && (0 until 4)
      .map(j => io.response(i).bits(32 * j + 31, 32 * j) > "h3f800000".U)
      .reduce(_ || _)
    when(io.response(i).fire) {
      for (j <- 0 until 2) {
        val index = if (rows == 1) (2 * i + j).U else Cat(row, (2 * i + j).U(4.W))
        tokens(index).decay := io.response(i).bits(64 * j + 31, 64 * j)
        tokens(index).beta := io.response(i).bits(64 * j + 63, 64 * j + 32)
      }
    }
  }
  when(bad.asUInt.orR) { invalid := true.B }
  when(active) {
    sent := sent | VecInit(io.read.map(_.fire)).asUInt
    received := received | VecInit(io.response.map(_.fire)).asUInt
    when(received.andR) {
      sent := 0.U
      received := 0.U
      when(row === (rows - 1).U) {
        active := false.B
        full := true.B
      }.otherwise { row := row + 1.U }
    }
  }
  io.result.valid := full
  io.result.bits := tokens
  io.error := invalid
  when(io.result.fire) { full := false.B }
}
