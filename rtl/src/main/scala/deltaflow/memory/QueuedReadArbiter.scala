package deltaflow.memory
import chisel3._
import chisel3.util._

/** Two or three synchronous SRAM readers. Registered requests break the multi-bank gather -> grant
  * combinational path. Response ownership lasts to consumption. Two address slots per client/bank.
  * Optional credit-reserved response slots for secondary clients break cross-bank gather cycles
  * when client masks differ.
  */
class QueuedReadArbiter(
    val responseDepth: Int = 0,
    val clients: Int = 2,
    val registeredOwnerReady: Boolean = false
) extends Module {
  require(Set(0, 2).contains(responseDepth))
  require(Set(2, 3).contains(clients))
  require(clients == 2 || responseDepth == 2)
  val io = IO(new Bundle {
    val request = Flipped(Vec(clients, Decoupled(UInt(10.W))))
    val response = Vec(clients, Decoupled(UInt(128.W)))
    val read = Decoupled(UInt(10.W))
    val returned = Flipped(Decoupled(UInt(128.W)))
    val shared = Input(Bool())
    val idle = Output(Bool())
  })
  val requests = Seq.fill(clients)(Module(new Queue(UInt(10.W), 2, pipe = false, flow = false)))
  val arb = Module(new RRArbiter(UInt(10.W), clients))
  // Match a three-response macro reservation without a return-ready -> issue
  // combinational bypass. Existing internal arbiters keep their old contract.
  val owners = Module(
    new Queue(
      UInt(log2Ceil(clients).W),
      if (registeredOwnerReady) 3 else 2,
      pipe = !registeredOwnerReady,
      flow = false
    )
  )
  val bypass = !io.shared && requests.map(q => !q.io.deq.valid).reduce(_ && _)
  for (i <- 0 until clients) {
    requests(i).io.enq <> io.request(i)
    arb.io.in(i) <> requests(i).io.deq
  }
  // Exclusive phases retain the original single-reader latency. Switching
  // back waits for both address queues to drain; the ordered owner queue still
  // covers all earlier SRAM responses, including those accepted while shared.
  when(bypass) {
    requests(0).io.enq.valid := false.B
    io.request(0).ready := io.read.ready && owners.io.enq.ready
  }
  io.read.valid := Mux(bypass, io.request(0).valid, arb.io.out.valid) && owners.io.enq.ready
  io.read.bits := Mux(bypass, io.request(0).bits, arb.io.out.bits)
  arb.io.out.ready := io.read.ready && owners.io.enq.ready && !bypass
  owners.io.enq.valid := io.read.fire
  owners.io.enq.bits := Mux(bypass, 0.U, arb.io.chosen)
  io.returned.ready := owners.io.deq.valid && io.response(owners.io.deq.bits).ready
  owners.io.deq.ready := io.returned.fire
  for (i <- 0 until clients) {
    io.response(i).valid := io.returned.valid && owners.io.deq.valid && owners.io.deq.bits === i.U
    io.response(i).bits := io.returned.bits
  }
  val bufferedIdle = Wire(Vec(clients - 1, Bool()))
  for (i <- 1 until clients) {
    bufferedIdle(i - 1) := true.B
    if (responseDepth > 0) {
      val q = Module(new Queue(UInt(128.W), responseDepth, pipe = false, flow = false))
      val pending = RegInit(0.U(log2Ceil(responseDepth + 1).W))
      val credit = (q.io.count +& pending) < responseDepth.U
      arb.io.in(i).valid := requests(i).io.deq.valid && credit
      requests(i).io.deq.ready := arb.io.in(i).ready && credit
      val issued = io.read.fire && !bypass && arb.io.chosen === i.U
      q.io.enq.valid := io.returned.valid && owners.io.deq.valid && owners.io.deq.bits === i.U
      q.io.enq.bits := io.returned.bits
      io.response(i) <> q.io.deq
      when(owners.io.deq.bits === i.U) {
        io.returned.ready := owners.io.deq.valid && q.io.enq.ready
      }
      when(issued =/= q.io.enq.fire) { pending := Mux(issued, pending + 1.U, pending - 1.U) }
      when(q.io.enq.valid) { assert(q.io.enq.ready && pending =/= 0.U) }
      assert((q.io.count +& pending) <= responseDepth.U)
      bufferedIdle(i - 1) := pending === 0.U && !q.io.deq.valid
    }
  }
  io.idle := !owners.io.deq.valid &&
    requests
      .map(q => !q.io.deq.valid)
      .reduce(_ && _) && bufferedIdle.asUInt.andR
  when(io.returned.valid) { assert(owners.io.deq.valid) }
}
