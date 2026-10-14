package deltaflow.memory
import chisel3._
import chisel3.util._
import deltaflow.sa.PublishedVector
import deltaflow.frontend.TransformCommand

/** Fixed token-group banks; two complete vectors reserved per group. K shares the actual Gram
  * operand words. V occupies row 128 onward in the same banks. Bank ownership is granted only after
  * Gram's pending reads drain.
  */
class TransformRhsReader(
    val side: Int = 16,
    val valuesOnly: Boolean = false,
    val keyLease: Boolean = false,
    val reordered: Boolean = false,
    val skipValues: Boolean = false,
    val boundedValues: Boolean = false,
    val independentReads: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(!skipValues || valuesOnly)
  require(!reordered || (!valuesOnly && !boundedValues))
  val words = (side + 7) / 8
  val io = IO(new Bundle {
    val start = Input(Valid(new TransformCommand))
    val valueReady = if (boundedValues) Some(Input(Vec(4, UInt(8.W)))) else None
    val skipFirst = if (skipValues) Some(Input(Bool())) else None
    val halfFirst = if (reordered) Some(Input(Bool())) else None
    val allowKeys = if (keyLease) Some(Input(Bool())) else None
    val available = Input(UInt(16.W))
    val bankEnable = Input(UInt(16.W))
    val read = Vec(16, Decoupled(UInt(9.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val result = Vec(4, Decoupled(new PublishedVector(side)))
    val reads = Output(UInt(16.W))
    val fed = Output(UInt(4.W))
    val idle = Output(Bool())
  })
  val active = RegInit(false.B)
  val columns = Reg(UInt(9.W))
  val values = Reg(UInt(8.W))
  when(io.start.valid) {
    active := true.B
    columns :=
      (if (valuesOnly) io.start.bits.valueColumns
       else io.start.bits.keyColumns +& io.start.bits.valueColumns)
    values := io.start.bits.valueColumns
  }
  for (b <- 0 until 16) {
    io.read(b).valid := false.B
    io.read(b).bits := 0.U
    io.response(b).ready := false.B
  }
  val split = Reg(Bool())
  when(io.start.valid) { split := io.halfFirst.getOrElse(false.B) }
  val first = Mux(values > (4 * side).U, (4 * side).U, values)
  val keys = columns - values
  // Decode the split lengths once, outside the steady-state SRAM grant path.
  // Per-group phase/local-column registers name the physical operand directly;
  // CS tickets remain monotonic and are checked against the canonical mapping.
  val firstEnd = if (reordered) Some(Reg(UInt(8.W))) else None
  val keyEnd = if (reordered) Some(Reg(UInt(8.W))) else None
  if (reordered) when(io.start.valid) {
    firstEnd.get := Mux(
      io.halfFirst.get && io.start.bits.valueColumns > (4 * side).U,
      (4 * side - 1).U,
      io.start.bits.valueColumns - 1.U
    )
    keyEnd.get := io.start.bits.keyColumns - 1.U
  }
  val done = Wire(Vec(4, Bool()))
  for (g <- 0 until 4) {
    val issued = RegInit(0.U(9.W))
    val consumed = RegInit(0.U(9.W))
    val occupied = RegInit(0.U(2.W))
    val tags = Module(new Queue(UInt(9.W), 2, pipe = false, flow = false))
    val out = Module(new Queue(new PublishedVector(side), 2, pipe = false, flow = false))
    val ports = (g * words until (g + 1) * words)
    val logical = Mux(
      split,
      Mux(
        issued < first,
        issued,
        Mux(issued < first + keys, values + issued - first, issued - keys)
      ),
      issued
    )
    val phase = if (reordered) Some(RegInit(0.U(2.W))) else None
    val localColumn = if (reordered) Some(RegInit(0.U(8.W))) else None
    val isValue = if (reordered) phase.get =/= 1.U else logical < values
    val valueAvailable = io.valueReady.map(x => logical < x(g)).getOrElse(true.B)
    val address =
      if (reordered)
        Mux(isValue, Cat(0.U(1.W), 1.U(1.W), localColumn.get(6, 0)), Cat(0.U(1.W), localColumn.get))
      else Mux(logical < values, 128.U + logical, logical - values)
    val partial = if (independentReads) Some(RegInit(false.B)) else None
    val heldAddress = if (independentReads) Some(Reg(UInt(9.W))) else None
    val remaining = if (independentReads) Some(Reg(UInt(words.W))) else None
    val eligible = ports.map(b => io.available(b) && io.bankEnable(b)).reduce(_ && _)
    val logicalReady = active && issued < columns && (!isValue || valueAvailable) &&
      (isValue || io.allowKeys.getOrElse(true.B)) && occupied < 2.U && tags.io.enq.ready && eligible
    // Occupy the original tag/result slot before physical grants. A partly
    // issued vector keeps its original address; another ticket cannot pass it.
    val can =
      if (independentReads) logicalReady && !partial.get
      else
        logicalReady && ports.map(b => io.read(b).ready).reduce(_ && _)
    tags.io.enq.valid := can
    tags.io.enq.bits := issued
    if (independentReads) {
      val needed = Mux(partial.get, remaining.get, ((1 << words) - 1).U(words.W))
      val offer = partial.get || can
      for ((b, local) <- ports.zipWithIndex) {
        io.read(b).valid := offer && needed(local) && io.available(b) && io.bankEnable(b)
        io.read(b).bits := Mux(partial.get, heldAddress.get, address)
      }
      val accepted = VecInit(ports.map(b => io.read(b).fire)).asUInt
      val left = needed & ~accepted
      when(can) {
        partial.get := left.orR
        when(left.orR) {
          heldAddress.get := address
          remaining.get := left
        }
      }
      when(partial.get) {
        remaining.get := left
        when(!left.orR) { partial.get := false.B }
        assert(occupied =/= 0.U, "partial RHS vector lost its reserved result")
        assert((accepted & ~remaining.get) === 0.U, "RHS bank accepted twice")
      }
      when(!partial.get && accepted.orR) {
        assert(can, "RHS physical read without logical reservation")
      }
      when(io.start.valid) {
        assert(!partial.get, "RHS restart before physical reads retire")
        partial.get := false.B
      }
    } else {
      for (b <- ports) {
        io.read(b).valid := can
        io.read(b).bits := address
      }
    }
    if (reordered) {
      when(can) {
        assert(
          isValue === (logical < values) && address === Mux(
            logical < values,
            128.U + logical,
            logical - values
          ),
          "registered RHS phase/column disagrees with canonical CS ticket"
        )
        when(phase.get === 0.U && localColumn.get === firstEnd.get) {
          phase.get := 1.U
          localColumn.get := 0.U
        }
          .elsewhen(phase.get === 1.U && localColumn.get === keyEnd.get) {
            phase.get := 2.U
            localColumn.get := (4 * side).U
          }
          .otherwise { localColumn.get := localColumn.get + 1.U }
      }
      when(io.start.valid) {
        phase.get := 0.U
        localColumn.get := 0.U
      }
    }
    val returned = ports.map(b => io.response(b).valid).reduce(_ && _)
    out.io.enq.valid := tags.io.deq.valid && returned
    out.io.enq.bits.key := tags.io.deq.bits
    for (r <- 0 until side)
      out.io.enq.bits
        .values(r) := io.response(g * words + r / 8).bits(16 * (r % 8) + 15, 16 * (r % 8))
    tags.io.deq.ready := returned && out.io.enq.ready
    for (b <- ports)
      io.response(b).ready := tags.io.deq.valid && out.io.enq.ready &&
        ports
          .filter(_ != b)
          .map(x => io.response(x).valid)
          .foldLeft(true.B)(_ && _)
    io.result(g) <> out.io.deq
    when(can) { issued := issued + 1.U }
    when(io.result(g).fire) {
      assert(io.result(g).bits.key === consumed)
      consumed := consumed + 1.U
    }
    when(can =/= io.result(g).fire) { occupied := Mux(can, occupied + 1.U, occupied - 1.U) }
    when(io.start.valid) {
      assert(occupied === 0.U)
      issued := Mux(io.skipFirst.getOrElse(false.B), (4 * side).U, 0.U)
      consumed := Mux(io.skipFirst.getOrElse(false.B), (4 * side).U, 0.U)
      occupied := 0.U
    }
    done(g) := consumed === columns && occupied === 0.U
  }
  when(active && done.asUInt.andR) { active := false.B }
  io.idle := !active
  io.reads := VecInit(io.read.map(_.fire)).asUInt
  io.fed := VecInit(io.result.map(_.fire)).asUInt
}
