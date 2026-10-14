package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.arithmetic._
import deltaflow.memory._
import deltaflow.sa.StreamPipes

class AllocatedGramRow(side: Int) extends Bundle {
  val row = new GramRow(side)
  val group = UInt(2.W)
  val slot = UInt(2.W)
}

/** Gram completion -> gated L/M and local J, using exactly 4*side+1 FP32 lanes. Gate production,
  * scaling and inverse updates share those lanes. The row/column gate SRAM is explicit; all other
  * storage is logic.
  */
class GramPreparation(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val tokens = 4 * side
  val frames = 4
  val io = IO(new Bundle {
    val gates = Flipped(Decoupled(Vec(tokens, new GateToken)))
    val start = Flipped(Decoupled(Bool()))
    val gram = Flipped(Vec(4, Decoupled(new GramRow(side))))
    val result = Vec(4, Decoupled(new GramRow(side)))
    val prefix = Output(Vec(tokens, UInt(32.W)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val gatesReady = Output(Bool())
    val fmaIssue = Output(UInt(4.W))
    val inverseIssue = Output(UInt(4.W))
    val gateIssue = Output(Bool())
  })
  val active = RegInit(false.B)
  val configured = RegInit(false.B)
  val gate = Reg(Vec(tokens, new GateToken))
  val prefix = Reg(Vec(tokens, UInt(32.W)))
  val previous = Reg(Vec(tokens, UInt(32.W)))
  val runningPrefix = Reg(UInt(32.W))
  val generating = RegInit(false.B)
  val genFlight = RegInit(false.B)
  val genSent = RegInit(false.B)
  val genRow = Reg(UInt(log2Ceil(tokens).W))
  val writeRow = Reg(UInt(log2Ceil(tokens).W))
  val writeBlock = Reg(UInt(2.W))
  val writePending = RegInit(false.B)
  val count = RegInit(0.U(log2Ceil(20 * side + 1).W))
  val store = Module(new GateScratchpad(side))
  val solvers = Seq.fill(4)(Module(new PipelinedRowInverse(side)))
  val lanes = Seq.fill(4, side)(Module(new RegisteredFp32))
  val prefixLane = Module(new RegisteredFp32)
  val state = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(frames)(0.U(3.W))))))
  val jobs = Reg(Vec(4, Vec(frames, UInt(5.W))))
  val rows = Reg(Vec(4, Vec(frames, UInt(log2Ceil(side).W))))
  val data = Reg(Vec(4, Vec(frames, Vec(side, UInt(32.W)))))
  val factors = Reg(Vec(4, Vec(frames, Vec(side, UInt(32.W)))))
  val output = Seq.fill(4)(Module(new Queue(new GramRow(side), 2, pipe = false, flow = false)))
  val jobI = VecInit(SharedGramLayout.jobs.map(_._2.U(2.W)))
  val jobJ = VecInit(SharedGramLayout.jobs.map(_._3.U(2.W)))
  val diagJobs = VecInit(Seq(0, 2, 5, 9).map(_.U(5.W)))
  io.busy := active
  io.done := false.B
  io.gatesReady := active && !generating
  io.prefix := prefix
  def legal(x: UInt) = !x(31) && x <= "h3f800000".U
  val validGates = io.gates.bits.map(g => legal(g.decay) && legal(g.beta)).reduce(_ && _)
  io.gates.ready := !active && !io.start.valid && validGates
  when(io.gates.fire) {
    gate := io.gates.bits
    configured := true.B
  }
  io.start.ready := !active && configured
  val start = io.start.fire
  when(start) {
    active := true.B
    generating := true.B
    genFlight := false.B
    genSent := false.B
    genRow := 0.U
    writePending := false.B
    count := 0.U
    runningPrefix := "h3f800000".U
    for (j <- 0 until tokens) previous(j) := 0.U
    for (g <- 0 until 4; s <- 0 until frames) state(g)(s) := 0.U
  }
  val genRequest = active && generating && !genFlight && !genSent
  io.gateIssue := genRequest
  prefixLane.io.in.valid := genRequest
  prefixLane.io.in.bits.a := runningPrefix
  prefixLane.io.in.bits.b := gate(genRow).decay
  prefixLane.io.in.bits.c := 0.U
  when(genRequest) { genFlight := true.B }
  when(prefixLane.io.out.valid) {
    assert(genFlight && !writePending)
    genFlight := false.B
    writePending := true.B
    writeBlock := 0.U
    writeRow := genRow
    runningPrefix := prefixLane.io.out.bits
    prefix(genRow) := prefixLane.io.out.bits
    for (j <- 0 until tokens) {
      assert(lanes(j / side)(j % side).io.out.valid)
      previous(j) := Mux(
        j.U < genRow,
        lanes(j / side)(j % side).io.out.bits,
        Mux(j.U === genRow, "h3f800000".U, 0.U)
      )
    }
    when(genRow === (tokens - 1).U) { genSent := true.B }.otherwise { genRow := genRow + 1.U }
  }
  store.io.write.valid := writePending
  store.io.write.bits.row := writeRow
  store.io.write.bits.block := writeBlock
  for (j <- 0 until side)
    store.io.write.bits.values(j) := previous(Cat(writeBlock, j.U(log2Ceil(side).W)))
  when(store.io.write.fire) {
    when(writeBlock === 3.U) {
      writePending := false.B
      when(genSent) { generating := false.B }
    }
      .otherwise { writeBlock := writeBlock + 1.U }
  }
  // Reserve a frame before issuing the synchronous gate lookup. Every physical
  // result port may target any logical block-row group; arbitration is bounded.
  val input = Module(new RRArbiter(new GramRow(side), 4))
  val free = VecInit(
    (0 until 4).map(g => VecInit((0 until frames).map(s => state(g)(s) === 0.U)).asUInt)
  )
  for (p <- 0 until 4) {
    val allowed = active && !generating && free(jobI(io.gram(p).bits.job)).orR
    input.io.in(p).valid := io.gram(p).valid && allowed
    input.io.in(p).bits := io.gram(p).bits
    io.gram(p).ready := input.io.in(p).ready && allowed
    when(io.gram(p).fire) { assert(io.gram(p).bits.job < 20.U) }
  }
  // Reserve the frame at admission; write the wide payload only after a
  // registered allocation boundary. This avoids job decode -> arbitration ->
  // frame selection -> wide register enables in one cycle.
  val allocated = Module(new Queue(new AllocatedGramRow(side), 2, pipe = false, flow = false))
  val admitted = input.io.out.bits
  val target = jobI(admitted.job)
  val slot = PriorityEncoder(free(target))
  allocated.io.enq.valid := input.io.out.valid
  input.io.out.ready := allocated.io.enq.ready
  allocated.io.enq.bits.row := admitted
  allocated.io.enq.bits.group := target
  allocated.io.enq.bits.slot := slot
  when(input.io.out.fire) { state(target)(slot) := 1.U }
  val incoming = allocated.io.deq.bits.row
  val selectedGroup = allocated.io.deq.bits.group
  val selectedSlot = allocated.io.deq.bits.slot
  store.io.request.valid := allocated.io.deq.valid
  allocated.io.deq.ready := store.io.request.ready
  store.io.request.bits.column := incoming.job >= 10.U
  store.io.request.bits.fixed := Cat(
    Mux(incoming.job < 10.U, jobI(incoming.job), jobJ(incoming.job)),
    incoming.row
  )
  store.io.request.bits.block := Mux(incoming.job < 10.U, jobJ(incoming.job), jobI(incoming.job))
  store.io.request.bits.tag := Cat(selectedGroup, selectedSlot)
  when(allocated.io.deq.fire) {
    assert(state(selectedGroup)(selectedSlot) === 1.U)
    jobs(selectedGroup)(selectedSlot) := incoming.job
    rows(selectedGroup)(selectedSlot) := incoming.row
    data(selectedGroup)(selectedSlot) := incoming.values
  }
  store.io.response.ready := true.B
  when(store.io.response.fire) {
    val g = store.io.response.bits.tag(3, 2)
    val s = store.io.response.bits.tag(1, 0)
    assert(state(g)(s) === 1.U)
    state(g)(s) := 2.U
    factors(g)(s) := store.io.response.bits.values
  }
  val issued = Wire(Vec(4, Bool()))
  val inverseIssued = Wire(Vec(4, Bool()))
  for (g <- 0 until 4) {
    val solver = solvers(g)
    solver.io.start.valid := start
    solver.io.start.bits := true.B
    when(start) { assert(solver.io.start.ready) }
    val rr = RegInit(0.U(2.W))
    val preferInverse = RegInit(false.B)
    val order = Wire(Vec(frames, UInt(2.W)))
    val candidates = Wire(Vec(frames, Bool()))
    for (s <- 0 until frames) {
      order(s) := rr + s.U
      candidates(s) := state(g)(order(s)) === 2.U || state(g)(order(s)) === 4.U
    }
    val s = order(PriorityEncoder(candidates.asUInt))
    val hasScale = candidates.asUInt.orR
    val inverse = solver.io.request.valid && (!hasScale || preferInverse)
    val issue = active && !generating && (hasScale || inverse)
    val beta = state(g)(s) === 4.U
    val tag = Wire(Valid(UInt(6.W)))
    tag.valid := issue
    tag.bits := Mux(
      inverse,
      Cat(2.U(2.W), solver.io.request.bits.row.pad(4)),
      Cat(Mux(beta, 1.U(2.W), 0.U(2.W)), 0.U(2.W), s)
    )
    val returned = StreamPipes.delay(tag, 9, true.B)
    solver.io.request.ready := issue && inverse
    solver.io.response.valid := returned.valid && returned.bits(5, 4) === 2.U
    solver.io.response.bits.row := returned.bits(3, 0)
    issued(g) := issue
    inverseIssued(g) := issue && inverse
    for (j <- 0 until side) {
      val lane = lanes(g)(j)
      lane.io.in.valid := genRequest || issue
      lane.io.in.bits.a := Mux(
        genRequest,
        previous(g * side + j),
        Mux(inverse, solver.io.request.bits.operands(j).a, data(g)(s)(j))
      )
      lane.io.in.bits.b := Mux(
        genRequest,
        gate(genRow).decay,
        Mux(
          inverse,
          solver.io.request.bits.operands(j).b,
          Mux(beta, gate(Cat(g.U(2.W), rows(g)(s))).beta, factors(g)(s)(j))
        )
      )
      lane.io.in.bits.c := Mux(inverse && !genRequest, solver.io.request.bits.operands(j).c, 0.U)
      solver.io.response.bits.values(j) := lane.io.out.bits
      when(returned.valid) { assert(lane.io.out.valid) }
    }
    when(issue) {
      preferInverse := !inverse
      when(!inverse) {
        state(g)(s) := Mux(beta, 5.U, 3.U)
        rr := s + 1.U
      }
    }
    when(returned.valid && returned.bits(5, 4) =/= 2.U) {
      val rslot = returned.bits(1, 0)
      val phase = returned.bits(5, 4)
      assert(state(g)(rslot) === Mux(phase === 0.U, 3.U, 5.U))
      val job = jobs(g)(rslot)
      val row = rows(g)(rslot)
      for (j <- 0 until side) {
        val keep = Mux(
          job < 10.U,
          Cat(jobJ(job), j.U(log2Ceil(side).W)) < Cat(g.U(2.W), row),
          Cat(g.U(2.W), j.U(log2Ceil(side).W)) >= Cat(jobJ(job), row)
        )
        data(g)(rslot)(j) := Mux(keep, lanes(g)(j).io.out.bits, 0.U)
      }
      state(g)(rslot) := Mux(phase === 0.U && job < 10.U, 4.U, 6.U)
    }
    val diagonal = VecInit(
      (0 until frames).map(s => state(g)(s) === 6.U && jobs(g)(s) === diagJobs(g))
    )
    val di = PriorityEncoder(diagonal.asUInt)
    solver.io.load.valid := diagonal.asUInt.orR
    solver.io.load.bits.row := rows(g)(di)
    solver.io.load.bits.values := data(g)(di)
    when(solver.io.load.fire) { state(g)(di) := 0.U }
    val pub = VecInit((0 until frames).map(s => state(g)(s) === 6.U && jobs(g)(s) =/= diagJobs(g)))
    val ps = PriorityEncoder(pub.asUInt)
    output(g).io.enq.valid := solver.io.result.valid || pub.asUInt.orR
    output(g).io.enq.bits.job := Mux(solver.io.result.valid, diagJobs(g), jobs(g)(ps))
    output(g).io.enq.bits.row := Mux(solver.io.result.valid, solver.io.result.bits.row, rows(g)(ps))
    output(g).io.enq.bits.values := Mux(
      solver.io.result.valid,
      solver.io.result.bits.values,
      data(g)(ps)
    )
    solver.io.result.ready := output(g).io.enq.ready
    when(output(g).io.enq.fire && !solver.io.result.valid) { state(g)(ps) := 0.U }
    io.result(g) <> output(g).io.deq
  }
  io.fmaIssue := issued.asUInt
  io.inverseIssue := inverseIssued.asUInt
  val commits = PopCount(VecInit(io.result.map(_.fire)))
  when(commits =/= 0.U) {
    count := count + commits
    when(count +& commits === (20 * side).U) {
      active := false.B
      io.done := true.B
      assert(!generating)
    }
  }
}
