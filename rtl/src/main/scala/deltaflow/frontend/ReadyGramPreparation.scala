package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.arithmetic._
import deltaflow.memory._
import deltaflow.sa.{StreamPipes, PublishedVector}

class RhsPreparationPorts(side: Int, reordered: Boolean = false) extends Bundle {
  val columns = Input(UInt(9.W))
  val valueColumns = Input(UInt(8.W))
  val halfFirst = if (reordered) Some(Input(Bool())) else None
  val raw = Flipped(Vec(4, Decoupled(new PublishedVector(side))))
  val result = Vec(4, Decoupled(new PublishedVector(side)))
  val issued = Output(UInt(4.W))
  val factorIssued = Output(UInt(4.W))
  val coefficientsDone = Output(Bool())
}
class QueryScalePorts(side: Int) extends Bundle {
  val raw = Flipped(Vec(4, Decoupled(new ProjectionColumn(side))))
  val result = Vec(4, Decoupled(new PublishedVector(side)))
  val issued = Output(UInt(4.W))
}
class BorrowedScalePorts(side: Int) extends Bundle {
  val request = Flipped(Decoupled(Vec(4 * side, new FmaOperands)))
  val response = Decoupled(Vec(4 * side, UInt(32.W)))
  val tailGate = Output(Vec(4 * side, UInt(32.W)))
}

class ReadyAllocatedGramRow(side: Int) extends Bundle {
  val row = new GramRow(side)
  val group = UInt(2.W)
  val slot = UInt(2.W)
}
class DecodedGramRow(side: Int) extends Bundle {
  val row = new GramRow(side)
  val group = UInt(2.W)
  val diagonal = Bool()
  val requiredGateRow = UInt(log2Ceil(4 * side).W)
}

/** Ready Gram completion -> gated L/M and local J, using exactly 4*side+1 FP32 lanes. Gate
  * production, scaling and inverse updates share those lanes. The row/column gate SRAM is explicit;
  * all other storage is logic.
  */
class ReadyGramPreparation(
    val side: Int = 16,
    val withRhs: Boolean = false,
    val withQuery: Boolean = false,
    val withBorrowedScale: Boolean = false,
    val borrowedFma: Boolean = false,
    val interleavedBorrow: Boolean = false,
    val registeredBorrowCredits: Boolean = false,
    val reorderedRhs: Boolean = false,
    val registeredBorrowStatus: Boolean = false,
    val externalBorrow: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(!borrowedFma || withBorrowedScale)
  require(!registeredBorrowStatus || borrowedFma)
  require(!interleavedBorrow || borrowedFma)
  require(!registeredBorrowCredits || interleavedBorrow)
  val tokens = 4 * side
  val frames = 4
  val io = IO(new Bundle {
    val externalBorrowEnable = if (externalBorrow) Some(Input(Bool())) else None
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
    val rhs = if (withRhs) Some(new RhsPreparationPorts(side, reorderedRhs)) else None
    val query = if (withQuery) Some(new QueryScalePorts(side)) else None
    val scale = if (withBorrowedScale) Some(new BorrowedScalePorts(side)) else None
  })
  val active = RegInit(false.B)
  val configured = RegInit(false.B)
  val gate = Reg(Vec(tokens, new GateToken))
  val prefix = Reg(Vec(tokens, UInt(32.W)))
  val rowReady = RegInit(VecInit(Seq.fill(tokens)(false.B)))
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
  val lanes = Seq.fill(4, side)(Module(new RegisteredFp32(earlyProduct = true)))
  val prefixLane = Module(new RegisteredFp32(earlyProduct = true))
  val state = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(frames)(0.U(3.W))))))
  val jobs = Reg(Vec(4, Vec(frames, UInt(5.W))))
  val rows = Reg(Vec(4, Vec(frames, UInt(log2Ceil(side).W))))
  val betaValue = Reg(Vec(4, Vec(frames, UInt(32.W))))
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
  val queryDrained = WireDefault(true.B)
  val borrowedDrained = WireDefault(true.B)
  val borrowBlock = WireDefault(false.B)
  io.start.ready := !active && configured && queryDrained && borrowedDrained
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
    for (j <- 0 until tokens) {
      previous(j) := 0.U
      rowReady(j) := false.B
    }
    for (g <- 0 until 4; s <- 0 until frames) state(g)(s) := 0.U
  }
  val genRequest = active && generating && !genFlight && !genSent && !borrowBlock
  io.gateIssue := genRequest
  prefixLane.io.in.valid := genRequest
  prefixLane.io.in.bits.a := runningPrefix
  prefixLane.io.in.bits.b := gate(genRow).decay
  prefixLane.io.in.bits.c := 0.U
  when(genRequest) { genFlight := true.B }
  when(prefixLane.io.product.get.valid) {
    assert(genFlight && !writePending)
    genFlight := false.B
    writePending := true.B
    writeBlock := 0.U
    writeRow := genRow
    runningPrefix := prefixLane.io.product.get.bits
    prefix(genRow) := prefixLane.io.product.get.bits
    for (j <- 0 until tokens) {
      assert(lanes(j / side)(j % side).io.product.get.valid)
      previous(j) := Mux(
        j.U < genRow,
        lanes(j / side)(j % side).io.product.get.bits,
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
      rowReady(writeRow) := true.B
      writePending := false.B
      when(genSent) { generating := false.B }
    }
      .otherwise { writeBlock := writeBlock + 1.U }
  }
  // Reserve a frame before issuing the synchronous gate lookup. Every physical
  // result port may target any logical block-row group; arbitration is bounded.
  // Register the job decode before readiness and resource arbitration. A
  // single bounded entry per input also keeps payload and metadata aligned.
  // No ready/credit bypass: accepting a row always consumes a real free entry.
  val ingress =
    Seq.fill(4)(Module(new Queue(new DecodedGramRow(side), 1, pipe = false, flow = false)))
  for (p <- 0 until 4) {
    val incoming = io.gram(p).bits
    val job = incoming.job
    val q = ingress(p)
    q.io.enq.valid := io.gram(p).valid && active
    io.gram(p).ready := q.io.enq.ready && active
    q.io.enq.bits.row := incoming
    q.io.enq.bits.group := jobI(job)
    q.io.enq.bits.diagonal := job < 10.U && jobI(job) === jobJ(job)
    q.io.enq.bits.requiredGateRow := Mux(
      job < 10.U,
      Cat(jobI(job), incoming.row),
      Cat(jobI(job), (side - 1).U(log2Ceil(side).W))
    )
    when(io.gram(p).fire) { assert(job < 20.U) }
  }
  val input = Module(new RRArbiter(new DecodedGramRow(side), 4))
  val free = VecInit(
    (0 until 4).map(g => VecInit((0 until frames).map(s => state(g)(s) === 0.U)).asUInt)
  )
  val allocationRr = RegInit(0.U(2.W))
  val allocationOrder = Wire(Vec(4, UInt(2.W)))
  val allocationChoices = Wire(Vec(4, Bool()))
  for (g <- 0 until 4) {
    allocationOrder(g) := allocationRr + g.U
    allocationChoices(g) := free(allocationOrder(g)).orR
  }
  val scaleTarget = allocationOrder(PriorityEncoder(allocationChoices.asUInt))
  def targetFor(row: DecodedGramRow): UInt = Mux(row.diagonal, row.group, scaleTarget)
  val admission = Wire(Vec(4, Bool()))
  val isDiagonal = Wire(Vec(4, Bool()))
  val urgent = VecInit(
    (0 until 4).map(p => admission(p) && isDiagonal(p) && ingress(p).io.deq.valid)
  ).asUInt.orR
  for (p <- 0 until 4) {
    val row = ingress(p).io.deq.bits
    admission(p) := active && rowReady(row.requiredGateRow) && Mux(
      row.diagonal,
      free(row.group).orR,
      free.map(_.orR).reduce(_ || _)
    )
    isDiagonal(p) := row.diagonal
    val allowed = admission(p) && (!urgent || isDiagonal(p))
    input.io.in(p).valid := ingress(p).io.deq.valid && allowed
    input.io.in(p).bits := row
    ingress(p).io.deq.ready := input.io.in(p).ready && allowed
  }
  // Reserve the frame at admission; write the wide payload only after a
  // registered allocation boundary. This avoids job decode -> arbitration ->
  // frame selection -> wide register enables in one cycle.
  val allocated = Module(new Queue(new ReadyAllocatedGramRow(side), 2, pipe = false, flow = false))
  val admitted = input.io.out.bits
  val target = targetFor(admitted)
  val slot = PriorityEncoder(free(target))
  allocated.io.enq.valid := input.io.out.valid
  input.io.out.ready := allocated.io.enq.ready
  allocated.io.enq.bits.row := admitted.row
  allocated.io.enq.bits.group := target
  allocated.io.enq.bits.slot := slot
  when(input.io.out.fire) {
    state(target)(slot) := 1.U
    allocationRr := target + 1.U
  }
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
    betaValue(selectedGroup)(selectedSlot) := gate(Cat(jobI(incoming.job), incoming.row)).beta
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
  val rhsIssued = Wire(Vec(4, Bool()))
  val factorIssued = Wire(Vec(4, Bool()))
  val rhsDrained = Wire(Vec(4, Bool()))
  val laneBusy = Wire(Vec(4, Bool()))
  for (g <- 0 until 4) {
    val solver = solvers(g)
    solver.io.start.valid := start
    solver.io.start.bits := true.B
    when(start) { assert(solver.io.start.ready) }
    val rr = RegInit(0.U(2.W))
    val preferInverse = RegInit(false.B)
    val order = Wire(Vec(frames, UInt(2.W)))
    val candidates = Wire(Vec(frames, Bool()))
    val ready = VecInit((0 until frames).map(s => state(g)(s) === 2.U || state(g)(s) === 4.U))
    val diagonalReady = VecInit(
      (0 until frames).map(s => ready(s) && jobs(g)(s) === diagJobs(g))
    ).asUInt.orR
    for (s <- 0 until frames) {
      order(s) := rr + s.U
      candidates(s) := ready(order(s)) && (!diagonalReady || jobs(g)(order(s)) === diagJobs(g))
    }
    val s = order(PriorityEncoder(candidates.asUInt))
    val hasScale = candidates.asUInt.orR
    val inverse = solver.io.request.valid && (!hasScale || preferInverse)
    val extraGrant = WireDefault(false.B)
    val extraA = WireDefault(VecInit(Seq.fill(side)(0.U(32.W))))
    val extraB = WireDefault(VecInit(Seq.fill(side)(0.U(32.W))))
    rhsIssued(g) := false.B
    factorIssued(g) := false.B
    rhsDrained(g) := true.B
    if (withRhs) {
      val r = io.rhs.get
      val preferExtra = RegInit(false.B)
      val factorState = RegInit(0.U(2.W))
      val betaPrefix = Reg(Vec(side, UInt(32.W)))
      val reserved = RegInit(0.U(4.W))
      val sent = RegInit(0.U(9.W))
      val consumed = RegInit(0.U(9.W))
      val q = Module(new Queue(new PublishedVector(side), 8, pipe = false, flow = false))
      r.result(g) <> q.io.deq
      val factorWant = rowReady((g + 1) * side - 1) && factorState === 0.U
      val first = Mux(r.valueColumns > (4 * side).U, (4 * side).U, r.valueColumns)
      val isValue = Mux(
        r.halfFirst.getOrElse(false.B),
        r.raw(g).bits.key < first || r.raw(g).bits.key >= first + r.columns - r.valueColumns,
        r.raw(g).bits.key < r.valueColumns
      )
      val rawWant =
        r.raw(g).valid && (isValue || factorState === 2.U) && (q.io.count +& reserved) < 8.U &&
          sent < r.columns
      extraGrant := active && !genRequest && !borrowBlock && (factorWant || rawWant) &&
        (preferExtra || !(hasScale || inverse))
      val factor = extraGrant && factorWant
      r.raw(g).ready := extraGrant && !factorWant
      val raw = r.raw(g).fire
      rhsIssued(g) := raw
      factorIssued(g) := factor
      val tag = Wire(Valid(UInt(17.W)))
      tag.valid := extraGrant
      tag.bits := Cat(factor, r.raw(g).bits.key)
      val ret = StreamPipes.delay(tag, 5, true.B)
      q.io.enq.valid := ret.valid && !ret.bits(16)
      q.io.enq.bits.key := ret.bits(15, 0)
      for (j <- 0 until side) {
        extraA(j) := Mux(
          factor,
          gate(g * side + j).beta,
          Cat(r.raw(g).bits.values(j)(15, 0), 0.U(16.W))
        )
        extraB(j) := Mux(
          factor,
          prefix(g * side + j),
          Mux(isValue, gate(g * side + j).beta, betaPrefix(j))
        )
        q.io.enq.bits.values(j) := lanes(g)(j).io.product.get.bits
        when(ret.valid && ret.bits(16)) { betaPrefix(j) := lanes(g)(j).io.product.get.bits }
      }
      when(factor) { factorState := 1.U }
      when(ret.valid) { assert(lanes(g)(0).io.product.get.valid) }
      when(ret.valid && ret.bits(16)) { factorState := 2.U }
      when(q.io.enq.valid) { assert(q.io.enq.ready && reserved =/= 0.U) }
      when(raw =/= q.io.enq.fire) { reserved := Mux(raw, reserved + 1.U, reserved - 1.U) }
      when(raw) {
        assert(r.raw(g).bits.key === sent)
        sent := sent + 1.U
      }
      when(r.result(g).fire) {
        assert(r.result(g).bits.key === consumed)
        consumed := consumed + 1.U
      }
      when(active && !genRequest && (extraGrant || hasScale || inverse)) {
        preferExtra := !extraGrant
      }
      rhsDrained(
        g
      ) := consumed === r.columns && reserved === 0.U && q.io.count === 0.U && factorState === 2.U
      when(start) {
        preferExtra := false.B
        factorState := 0.U
        reserved := 0.U
        sent := 0.U
        consumed := 0.U
        assert(q.io.count === 0.U)
      }
    }
    val issue = active && !genRequest && !extraGrant && !borrowBlock && (hasScale || inverse)
    laneBusy(g) := genRequest || issue || extraGrant
    val beta = state(g)(s) === 4.U
    val tag = Wire(Valid(UInt(6.W)))
    tag.valid := issue && inverse
    tag.bits := Mux(
      inverse,
      Cat(2.U(2.W), solver.io.request.bits.row.pad(4)),
      Cat(Mux(beta, 1.U(2.W), 0.U(2.W)), 0.U(2.W), s)
    )
    val inverseReturned = StreamPipes.delay(tag, 9, true.B)
    val scaleTag = Wire(Valid(UInt(6.W)))
    scaleTag.valid := issue && !inverse
    scaleTag.bits := tag.bits
    val returned = StreamPipes.delay(scaleTag, 5, true.B)
    solver.io.request.ready := issue && inverse
    solver.io.response.valid := inverseReturned.valid
    solver.io.response.bits.row := inverseReturned.bits(3, 0)
    issued(g) := issue
    inverseIssued(g) := issue && inverse
    for (j <- 0 until side) {
      val lane = lanes(g)(j)
      lane.io.in.valid := genRequest || issue || extraGrant
      lane.io.in.bits.a := Mux(
        genRequest,
        previous(g * side + j),
        Mux(
          extraGrant,
          extraA(j),
          Mux(inverse, solver.io.request.bits.operands(j).a, data(g)(s)(j))
        )
      )
      lane.io.in.bits.b := Mux(
        genRequest,
        gate(genRow).decay,
        Mux(
          extraGrant,
          extraB(j),
          Mux(
            inverse,
            solver.io.request.bits.operands(j).b,
            Mux(beta, betaValue(g)(s), factors(g)(s)(j))
          )
        )
      )
      lane.io.in.bits.c := Mux(inverse && issue, solver.io.request.bits.operands(j).c, 0.U)
      solver.io.response.bits.values(j) := lane.io.out.bits
      when(inverseReturned.valid) { assert(lane.io.out.valid) }
      when(returned.valid) { assert(lane.io.product.get.valid) }
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
          Cat(jobJ(job), j.U(log2Ceil(side).W)) < Cat(jobI(job), row),
          Cat(jobI(job), j.U(log2Ceil(side).W)) >= Cat(jobJ(job), row)
        )
        data(g)(rslot)(j) := Mux(keep, lanes(g)(j).io.product.get.bits, 0.U)
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
    if (!withRhs) when(count +& commits === (20 * side).U) {
      active := false.B
      io.done := true.B
      assert(!generating)
    }
  }
  io.rhs.foreach { r =>
    r.issued := rhsIssued.asUInt
    r.factorIssued := factorIssued.asUInt
    r.coefficientsDone := count === (20 * side).U
    when(active && count === (20 * side).U && rhsDrained.asUInt.andR) {
      active := false.B
      io.done := true.B
      assert(!generating)
    }
  }
  if (withQuery) {
    // Qbar borrows only otherwise-idle slots of the SAME lanes. It cannot
    // displace gate, inverse, scale or RHS requests from the critical chain.
    val q = io.query.get
    val empty = Wire(Vec(4, Bool()))
    for (g <- 0 until 4) {
      val reserved = RegInit(0.U(2.W))
      val returned = Module(new Queue(new PublishedVector(side), 2, pipe = false, flow = false))
      q.result(g) <> returned.io.deq
      q.raw(g).ready := !laneBusy(g) && !borrowBlock && !io.start.valid && configured && rowReady(
        (g + 1) * side - 1
      ) && (reserved +& returned.io.count) < 2.U
      val grant = q.raw(g).fire
      val tag = Wire(Valid(UInt(8.W)))
      tag.valid := grant
      tag.bits := q.raw(g).bits.column
      val ret = StreamPipes.delay(tag, 5, true.B)
      returned.io.enq.valid := ret.valid
      returned.io.enq.bits.key := ret.bits
      for (j <- 0 until side) {
        when(grant) {
          lanes(g)(j).io.in.valid := true.B
          lanes(g)(j).io.in.bits.a := Cat(q.raw(g).bits.values(j), 0.U(16.W))
          lanes(g)(j).io.in.bits.b := prefix(g * side + j)
          lanes(g)(j).io.in.bits.c := 0.U
        }
        returned.io.enq.bits.values(j) := lanes(g)(j).io.product.get.bits
        when(ret.valid) { assert(lanes(g)(j).io.product.get.valid) }
      }
      when(ret.valid) { assert(returned.io.enq.ready && reserved =/= 0.U) }
      when(grant =/= ret.valid) { reserved := Mux(grant, reserved + 1.U, reserved - 1.U) }
      empty(g) := reserved === 0.U && returned.io.count === 0.U
      when(start) { assert(empty(g)) }
    }
    queryDrained := empty.asUInt.andR
    q.issued := VecInit(q.raw.map(_.fire)).asUInt
  }
  io.scale.foreach { port =>
    // Whole-vector ownership only after original preparation and Qbar drain.
    // RegisteredFp32 remains the only arithmetic. The original product-only
    // contract returns after five cycles; state epilogues select the complete
    // nine-cycle FMA. Preserve finite return credits at the selected latency.
    val depth = if (borrowedFma) 12 else 8
    val q = Module(new Queue(Vec(tokens, UInt(32.W)), depth, pipe = false, flow = false))
    val reserved = RegInit(0.U(4.W))
    // Keep accepted-but-not-consumed vectors in one register. Reconstructing
    // the same value from FIFO pointers plus in-flight reservations feeds a
    // long arithmetic/arbiter chain into every shared FP32 operand selector.
    val occupied = if (registeredBorrowCredits) Some(RegInit(0.U(log2Ceil(depth + 1).W))) else None
    val hasCredit = occupied.map(_ < depth.U).getOrElse((q.io.count +& reserved) < depth.U)
    port.response <> q.io.deq
    port.tailGate := previous
    if (registeredBorrowStatus) {
      // Exact next-cycle nonempty status, without a pointer comparison feeding
      // H/O return selection. Count is used only at this register boundary.
      val put = q.io.enq.valid
      val present = BorrowedReturnStatus(q.io.count, put, port.response.ready)
      port.response.valid := present
      assert(present === q.io.deq.valid)
      when(put) { assert(q.io.enq.ready) }
    }
    port.request.ready := !active && queryDrained &&
      !io.query
        .map(_.raw.map(_.valid).reduce(_ || _))
        .getOrElse(false.B) && configured && !io.start.valid && hasCredit
    if (interleavedBorrow) {
      // The same pipelined lanes may carry independent five-/nine-cycle tags.
      // Reserve alternate issue slots for a waiting state epilogue, and use
      // unclaimed slots for either client. No return or operand bypass.
      val turn = RegInit(false.B)
      when(active) { turn := !turn }
      val available =
        (configured || io.externalBorrowEnable.getOrElse(false.B)) && !io.start.valid && hasCredit
      borrowBlock := available && port.request.valid && (!active || turn)
      val queryWant = io.query.map(_.raw.map(_.valid).reduce(_ || _)).getOrElse(false.B)
      port.request.ready := available && (borrowBlock || (!laneBusy.asUInt.orR && !queryWant))
    }
    val returned = ShiftRegister(port.request.fire, if (borrowedFma) 9 else 5, false.B, true.B)
    q.io.enq.valid := returned
    for (j <- 0 until tokens) {
      val lane = lanes(j / side)(j % side)
      when(port.request.fire) {
        lane.io.in.valid := true.B
        lane.io.in.bits := port.request.bits(j)
        assert(!laneBusy(j / side))
      }
      val output = if (borrowedFma) lane.io.out else lane.io.product.get
      q.io.enq.bits(j) := output.bits
      when(returned) { assert(output.valid) }
      if (!borrowedFma) when(port.request.fire) {
        assert(port.request.bits(j).c === 0.U, "product-only borrowed port requires zero addend")
      }
    }
    when(port.request.fire =/= returned) {
      reserved := Mux(port.request.fire, reserved + 1.U, reserved - 1.U)
    }
    when(returned) { assert(q.io.enq.ready && reserved =/= 0.U) }
    occupied.foreach { n =>
      assert(
        n === q.io.count +& reserved && n <= depth.U,
        "borrowed FP32 credit differs from queued plus in-flight vectors"
      )
      when(port.request.fire =/= port.response.fire) {
        n := Mux(port.request.fire, n + 1.U, n - 1.U)
      }
      when(port.response.fire) { assert(n =/= 0.U) }
    }
    borrowedDrained := occupied.map(_ === 0.U).getOrElse(reserved === 0.U && q.io.count === 0.U)
  }
}
