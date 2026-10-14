package deltaflow.arithmetic
import chisel3._
import chisel3.util._

class InverseWork(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val operands = Vec(side, new FmaOperands)
}
class InverseAddress(side: Int) extends Bundle {
  val row = UInt(log2Ceil(side).W)
  val k = UInt(log2Ceil(side).W)
}

/** Independent future rows hide the external FMA latency. Each individual J element still consumes
  * k=1..r-1 in order, exactly as RowInverseController. L and partial J are bounded local registers;
  * no arithmetic is duplicated.
  */
class PipelinedRowInverse(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(Bool()))
    val load = Flipped(Decoupled(new FloatRow(side)))
    val request = Decoupled(new InverseWork(side))
    val response = Flipped(Valid(new FloatRow(side)))
    val result = Decoupled(new FloatRow(side))
    val busy = Output(Bool())
  })
  val active = RegInit(false.B)
  val loaded = RegInit(VecInit(Seq.fill(side)(false.B)))
  val done = RegInit(VecInit(Seq.fill(side)(false.B)))
  val pending = RegInit(VecInit(Seq.fill(side)(false.B)))
  val l = Reg(Vec(side, Vec(side, UInt(32.W))))
  val sums = Reg(Vec(side, Vec(side, UInt(32.W))))
  val next = Reg(Vec(side, UInt(log2Ceil(side).W)))
  val published = RegInit(0.U(log2Ceil(side).W))
  val rr = RegInit(0.U(log2Ceil(side).W))
  val locked = RegInit(false.B)
  val held = Reg(UInt(log2Ceil(side).W))
  val requests = Module(new Queue(new InverseWork(side), 2, pipe = false, flow = false))
  val addresses = Module(new Queue(new InverseAddress(side), 2, pipe = false, flow = false))
  io.request <> requests.io.deq
  io.start.ready := !active && !requests.io.deq.valid && !addresses.io.deq.valid
  io.busy := active
  io.load.ready := active && !loaded(io.load.bits.row)
  io.result.valid := active && done(published)
  io.result.bits.row := published
  io.result.bits.values := sums(published)
  when(io.start.fire) {
    active := true.B
    published := 0.U
    rr := 0.U
    locked := false.B
    for (r <- 0 until side) {
      loaded(r) := false.B
      done(r) := false.B
      pending(r) := false.B
      next(r) := 1.U
    }
  }
  when(io.load.fire) {
    val r = io.load.bits.row
    loaded(r) := true.B
    l(r) := io.load.bits.values
    for (j <- 0 until side)
      sums(r)(j) := Mux(
        j.U < r,
        io.load.bits.values(j) ^ "h80000000".U,
        Mux(j.U === r, "h3f800000".U, 0.U)
      )
    when(r <= 1.U) { done(r) := true.B }
  }
  val eligible = VecInit(
    (0 until side).map(r =>
      if (r < 2) false.B else loaded(r) && !done(r) && !pending(r) && done(next(r))
    )
  )
  val order = Wire(Vec(side, UInt(log2Ceil(side).W)))
  val choices = Wire(Vec(side, Bool()))
  for (i <- 0 until side) {
    order(i) := rr + i.U
    choices(i) := eligible(order(i))
  }
  val row = Mux(locked, held, order(PriorityEncoder(choices.asUInt)))
  addresses.io.enq.valid := active && (locked || choices.asUInt.orR)
  addresses.io.enq.bits.row := row
  addresses.io.enq.bits.k := next(row)
  requests.io.enq.valid := addresses.io.deq.valid
  addresses.io.deq.ready := requests.io.enq.ready
  val address = addresses.io.deq.bits
  requests.io.enq.bits.row := address.row
  for (j <- 0 until side) {
    requests.io.enq.bits.operands(j).a := l(address.row)(address.k) ^ "h80000000".U
    requests.io.enq.bits.operands(j).b := sums(address.k)(j)
    requests.io.enq.bits.operands(j).c := sums(address.row)(j)
  }
  when(addresses.io.enq.valid && !addresses.io.enq.ready) {
    locked := true.B
    held := row
  }
  when(addresses.io.enq.fire) {
    pending(row) := true.B
    rr := row + 1.U
    locked := false.B
  }
  when(io.response.valid) {
    val r = io.response.bits.row
    val k = next(r)
    assert(active && pending(r) && r > 1.U && k < r)
    for (j <- 0 until side) when(j.U < k) { sums(r)(j) := io.response.bits.values(j) }
    pending(r) := false.B
    when(k === r - 1.U) { done(r) := true.B }.otherwise { next(r) := k + 1.U }
  }
  when(io.result.fire) {
    when(published === (side - 1).U) {
      active := false.B
      assert(!pending.asUInt.orR && !requests.io.deq.valid && !addresses.io.deq.valid)
    }
      .otherwise { published := published + 1.U }
  }
}
