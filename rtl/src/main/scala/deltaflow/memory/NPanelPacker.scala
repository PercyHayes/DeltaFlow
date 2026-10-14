package deltaflow.memory

import chisel3._
import chisel3.util._

class NPackCommand(depth: Int) extends Bundle {
  val base = UInt(log2Ceil(depth).W)
  val columns = UInt(8.W)
  val epoch = UInt(32.W)
}
class NPackColumn extends Bundle {
  val column = UInt(7.W)
  val values = Vec(64, UInt(16.W))
}
class NPackedBlock(depth: Int) extends Bundle {
  val base = UInt(log2Ceil(depth).W)
  val group = UInt(4.W)
  val columns = UInt(4.W)
  val epoch = UInt(32.W)
}

/** Transpose already rounded N[key,token] columns into complete SRAM words. Two 8-key x 64-token
  * BF16 frames (2 KiB), plus two output words per bank (512 B). No FMA, PE, arithmetic reordering
  * or implicit SRAM initialization. bank=(token+key/8)%16; address=base+(token/16)*16+key/8.
  * Per-bank registered queues decouple SRAM ready from input/ownership logic. A block is published
  * only after all 64 words have committed. Frames stay owned until that publication is queued;
  * end-of-command waits its consumption. SRAM destination leases outlive this writer: the enclosing
  * scheduler must wait for H consumers before reusing a destination slot, even after done.
  */
class NPanelPacker(val depth: Int = 1024) extends Module {
  require(depth >= 64 && depth <= 4096 && isPow2(depth))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new NPackCommand(depth)))
    val input = Flipped(Decoupled(new NPackColumn))
    val write = Vec(16, Decoupled(new BankWrite(depth, 16)))
    val completed = Decoupled(new NPackedBlock(depth))
    val busy = Output(Bool())
    val done = Output(Bool())
  })
  val active = RegInit(false.B)
  val cmd = Reg(new NPackCommand(depth))
  val received = RegInit(0.U(8.W))
  val fill = RegInit(false.B)
  val head = RegInit(false.B)
  val full = RegInit(VecInit(Seq.fill(2)(false.B)))
  val group = Reg(Vec(2, UInt(4.W)))
  val columns = Reg(Vec(2, UInt(4.W)))
  val data = Reg(Vec(2, Vec(16, Vec(4, Vec(8, UInt(16.W))))))
  val issued = RegInit(VecInit(Seq.fill(16)(0.U(3.W))))
  val committed = RegInit(VecInit(Seq.fill(16)(0.U(3.W))))
  val events = Module(new Queue(new NPackedBlock(depth), 2, pipe = false, flow = false))
  io.completed <> events.io.deq
  io.start.ready := !active && !events.io.deq.valid
  io.busy := active
  io.done := false.B
  when(io.start.fire) {
    assert(io.start.bits.columns > 0.U && io.start.bits.columns <= 128.U)
    assert(io.start.bits.base +& 64.U <= depth.U)
    active := true.B
    cmd := io.start.bits
    received := 0.U
    fill := false.B
    head := false.B
    for (s <- 0 until 2) full(s) := false.B
    for (b <- 0 until 16) {
      issued(b) := 0.U
      committed(b) := 0.U
    }
  }
  io.input.ready := active && received < cmd.columns && !full(fill)
  val inputGroup = received(6, 3)
  val rotated = Wire(Vec(16, Vec(4, UInt(16.W))))
  for (b <- 0 until 16; r <- 0 until 4) {
    val source = (b.U(4.W) - inputGroup)(3, 0)
    val values = VecInit((0 until 16).map(t => io.input.bits.values(r * 16 + t)))
    rotated(b)(r) := values(source)
  }
  when(io.input.fire) {
    assert(io.input.bits.column === received(6, 0), "N columns must arrive in order")
    when(received(2, 0) === 0.U) { group(fill) := inputGroup }
    for (s <- 0 until 2; b <- 0 until 16; r <- 0 until 4; l <- 0 until 8) {
      // Rotate scalar lanes on ingress, avoiding a 128-bit bank crossbar at
      // the memory output. Only the selected BF16 lane of the frame is written.
      when(fill === s.U && received(2, 0) === l.U) { data(s)(b)(r)(l) := rotated(b)(r) }
    }
    when(received(2, 0) === 7.U || received + 1.U === cmd.columns) {
      columns(fill) := received(2, 0) +& 1.U
      full(fill) := true.B
      fill := !fill
    }
    received := received + 1.U
  }
  for (b <- 0 until 16) {
    val q = Module(new Queue(new BankWrite(depth, 16), 2, pipe = false, flow = false))
    io.write(b) <> q.io.deq
    q.io.enq.valid := active && full(head) && issued(b) < 4.U
    q.io.enq.bits.address := cmd.base + Cat(issued(b)(1, 0), group(head))
    q.io.enq.bits.mask := "hffff".U
    q.io.enq.bits.data := VecInit(
      (0 until 8).map(l => Mux(l.U < columns(head), data(head)(b)(issued(b)(1, 0))(l), 0.U(16.W)))
    ).asUInt
    when(q.io.enq.fire) { issued(b) := issued(b) + 1.U }
    when(q.io.deq.fire) { committed(b) := committed(b) + 1.U }
    assert(committed(b) <= issued(b) && issued(b) <= 4.U)
  }
  events.io.enq.valid := active && full(head) && committed.map(_ === 4.U).reduce(_ && _)
  events.io.enq.bits.base := cmd.base
  events.io.enq.bits.group := group(head)
  events.io.enq.bits.columns := columns(head)
  events.io.enq.bits.epoch := cmd.epoch
  when(events.io.enq.fire) {
    full(head) := false.B
    head := !head
    for (b <- 0 until 16) {
      issued(b) := 0.U
      committed(b) := 0.U
    }
  }
  when(
    io.completed.fire &&
      (Cat(
        io.completed.bits.group,
        0.U(3.W)
      ) +& io.completed.bits.columns) === cmd.columns
  ) {
    assert(received === cmd.columns && !full.asUInt.orR)
    active := false.B
    io.done := true.B
  }
}
