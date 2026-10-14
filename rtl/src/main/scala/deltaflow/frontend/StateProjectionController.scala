package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.arithmetic.FloatArithmetic
import deltaflow.memory.LocalBankGrants

class ProjectionColumn(side: Int, val wide: Boolean = false) extends Bundle {
  val column = UInt(8.W)
  val values = Vec(side, UInt((if (wide) 32 else 16).W))
}
class ProjectionRow(side: Int) extends Bundle {
  val group = UInt(2.W)
  val half = Bool()
  val row = UInt(log2Ceil(side).W)
  val values = Vec(4 * side, UInt(16.W))
}
class ProjectionReadTag extends Bundle {
  val frame = Bool()
  val kind = UInt(2.W)
}
class ProjectionRowTag(side: Int, val joint: Boolean = false) extends Bundle {
  val group = UInt(2.W)
  val half = Bool()
  val row = UInt(log2Ceil(side).W)
  val z = if (joint) Some(Bool()) else None
}
class ProjectionWideRow(side: Int) extends Bundle {
  val group = UInt(2.W)
  val half = Bool()
  val row = UInt(log2Ceil(side).W)
  val values = Vec(4 * side, UInt(32.W))
}

/** E = BF16(U - W BF16(H0)), preserving each output's increasing k order. No arithmetic array or
  * storage macro here. U columns seed existing even C32 contexts, while Gram owns odd contexts. Two
  * feature frames reserve all SRAM return data before reads; panels independently capture each
  * broadcast wave. H0 is immutable for this operation. eReadsDone does NOT release the other
  * old-state projection's ownership. H/O consumers retain E in Work SRAM.
  */
class StateProjectionController(
    val side: Int = 16,
    val joint: Boolean = false,
    val directResidual: Boolean = false,
    val residentZ: Boolean = false,
    val pipelinedReads: Boolean = false,
    val columnLookahead: Boolean = false,
    val independentReads: Boolean = true,
    val localGrantEpoch: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(!directResidual || joint)
  require(!residentZ || (joint && !directResidual))
  require(!pipelinedReads || residentZ)
  require(!columnLookahead || (joint && residentZ))
  require(!localGrantEpoch || (independentReads && pipelinedReads))
  val t = 4 * side
  val words = (side + 7) / 8
  val hBanks = t / 4
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new TransformCommand))
    val earlyE = if (columnLookahead) Some(Input(Bool())) else None
    val skipE0 = if (columnLookahead) Some(Input(Bool())) else None
    val u = Flipped(Vec(4, Decoupled(new ProjectionColumn(side, wide = directResidual))))
    val written = Input(Vec(4, UInt(128.W)))
    val read = Vec(16, Decoupled(UInt(9.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val bankEnable = Input(UInt(16.W))
    val enable = Input(Bool())
    val input = Vec(16, Decoupled(new RetimedPanelInput(side, 2)))
    val seed = Vec(16, Decoupled(new RetimedRowData(side)))
    val clear = Vec(16, Decoupled(UInt(2.W)))
    val rowRead = Vec(16, Decoupled(new RetimedRow(side)))
    val rowResponse = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val result = Decoupled(new ProjectionRow(side))
    val idle = Output(Bool())
    val done = Output(Bool())
    val eReadsDone = Output(Bool())
    val allocated = Output(Bool())
    val request = Output(SInt(12.W))
    val captured = Output(SInt(12.W))
    val issued = Output(UInt(16.W))
    val issueKey = Output(UInt(9.W))
    val seeded = Output(Vec(4, SInt(10.W)))
    val frames = Output(UInt(2.W))
    val rowRequested = Output(SInt(10.W))
    val rowCaptured = Output(SInt(10.W))
    val residual = if (directResidual) Some(new Bundle {
      val result = Vec(4, Decoupled(new PublishedVector(side)))
      val seedE = Flipped(Vec(4, Decoupled(new ProjectionColumn(side))))
      val readColumn = Output(Vec(16, Bool()))
    })
    else None
    val joint =
      if (StateProjectionController.this.joint) Some(new Bundle {
        val written = Input(Vec(4, UInt(128.W)))
        val released = Input(Vec(16, UInt(2.W)))
        val read = Vec(16, Decoupled(UInt(9.W)))
        val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
        val result = Decoupled(new ProjectionWideRow(side))
        val issueZ = Output(UInt(16.W))
        val clearZ = Output(UInt(16.W))
        val clearContext = Output(UInt(16.W))
        val contexts = Output(Vec(16, UInt(2.W)))
        val keys = Output(Vec(16, UInt(8.W)))
        val initialized = if (residentZ) Some(Output(Vec(16, UInt(2.W)))) else None
      })
      else None
  })
  // The legacy row producer and the residual column bridge share one physical
  // panel read port. Each request reserves a response-owner slot before issue.
  val rowRead = Wire(chiselTypeOf(io.rowRead))
  io.rowRead <> rowRead
  val rowResponse = Wire(chiselTypeOf(io.rowResponse))
  rowResponse <> io.rowResponse
  val active = RegInit(false.B)
  val cmd = Reg(new TransformCommand)
  val early = RegInit(false.B)
  val skipped = RegInit(false.B)
  val seededPanels = if (columnLookahead) Some(RegInit(VecInit(Seq.fill(4)(0.U(4.W))))) else None
  def physicalContext(c: UInt): UInt = Mux(early, 1.U(2.W), c)
  // Configuration decode is outside the steady-state admission/credit cone.
  // Both masks and the W pointer are latched at start, without an extra cycle.
  val secondHalf = Reg(Bool())
  val panelMasks = Reg(Vec(2, UInt(16.W)))
  val seedEnds = Reg(Vec(2, UInt(8.W)))
  val valueMasks = Reg(Vec(2, UInt(t.W)))
  val seededHalves = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val wAddress = Reg(UInt(9.W))
  val clearHalf = RegInit(false.B)
  val clearPending = RegInit(0.U(16.W))
  val clearing = RegInit(false.B)
  val seedCount = RegInit(VecInit(Seq.fill(4)(0.U(8.W))))
  val uQueues = Seq.fill(4)(
    Module(
      new Queue(new ProjectionColumn(side, wide = directResidual), 2, pipe = false, flow = false)
    )
  )
  val residualBridge = if (directResidual) Some(Module(new ResidualColumnBridge(side))) else None
  val eSeedCount = if (directResidual) Some(RegInit(VecInit(Seq.fill(4)(0.U(8.W))))) else None
  val eSeeds =
    if (directResidual)
      Some(
        Seq.fill(4)(Module(new Queue(new ProjectionColumn(side), 2, pipe = false, flow = false)))
      )
    else None
  val rCompleted = if (directResidual) Some(RegInit(VecInit(Seq.fill(16)(0.U(2.W))))) else None
  residualBridge.foreach { bridge =>
    bridge.io.start.valid := io.start.fire
    bridge.io.start.bits := io.start.bits.valueColumns
    io.residual.get.result <> bridge.io.result
    bridge.io.completed := rCompleted.get
    for (g <- 0 until 4) eSeeds.get(g).io.enq <> io.residual.get.seedE(g)
    when(io.start.fire) {
      assert(bridge.io.start.ready)
      for (g <- 0 until 4) eSeedCount.get(g) := 0.U
      for (p <- 0 until 16) rCompleted.get(p) := 0.U
    }
  }
  val live = RegInit(VecInit(Seq.fill(2)(false.B)))
  val k = Reg(Vec(2, UInt(8.W)))
  val w = Reg(Vec(2, Vec(t, UInt(16.W))))
  val q = if (joint) Some(Reg(Vec(2, Vec(t, UInt(16.W))))) else None
  val zCleared = if (joint) Some(RegInit(VecInit(Seq.fill(16)(0.U(2.W))))) else None
  val panelHeads = if (joint) Some(RegInit(VecInit(Seq.fill(16)(false.B)))) else None
  val panelPhases =
    if (joint) Some(RegInit(VecInit(Seq.fill(16)(if (columnLookahead) 1.U(4.W) else 0.U(2.W)))))
    else None
  val completed = if (joint) Some(RegInit(VecInit(Seq.fill(2)(0.U(16.W))))) else None
  val h = Reg(Vec(2, Vec(2, Vec(t, UInt(16.W)))))
  val ready = RegInit(VecInit(Seq.fill(2)(0.U(3.W))))
  val fetch = RegInit(0.U(8.W))
  val fetchFrame = RegInit(false.B)
  val requestKind = RegInit(0.U(2.W))
  val reading = RegInit(false.B)
  val head = RegInit(false.B)
  val half = RegInit(false.B)
  val capturedPanels = RegInit(0.U(16.W))
  val draining = RegInit(false.B)
  val rowFetch = RegInit(0.U(log2Ceil((if (joint) 16 else 8) * side + 1).W))
  val rowFinish = RegInit(0.U(log2Ceil((if (joint) 16 else 8) * side + 1).W))
  val rowActive = RegInit(false.B)
  val rowPending = RegInit(0.U(4.W))
  val tags = Module(new Queue(new ProjectionReadTag, 4, pipe = false, flow = false))
  val rowTags = Module(new Queue(new ProjectionRowTag(side, joint), 4, pipe = false, flow = false))
  val results = Module(new Queue(new ProjectionRow(side), 2, pipe = false, flow = false))
  io.result <> results.io.deq
  val zResults =
    if (joint && !residentZ)
      Some(Module(new Queue(new ProjectionWideRow(side), 2, pipe = false, flow = false)))
    else None
  zResults.foreach(r => io.joint.get.result <> r.io.deq)
  if (residentZ) {
    // Z stays in the odd C32 contexts for O = Z + ME. The consumer must
    // additionally wait for each panel's queued/in-flight Z waves to retire.
    io.joint.get.result.valid := false.B
    io.joint.get.result.bits := 0.U.asTypeOf(io.joint.get.result.bits)
    io.joint.get.initialized.get := zCleared.get
  }
  val halves = Mux(secondHalf, 2.U, 1.U)
  val totalRows = halves * (if (joint && !residentZ) 2 * t else t).U
  io.start.ready := !active && !tags.io.deq.valid && !rowTags.io.deq.valid &&
    !results.io.deq.valid && zResults.map(r => !r.io.deq.valid).getOrElse(true.B) &&
    uQueues
      .map(_.io.count === 0.U)
      .reduce(_ && _)
  io.idle := !active
  io.done := false.B
  io.eReadsDone := active && draining
  when(io.start.fire) {
    assert(io.start.bits.keyColumns > 0.U && io.start.bits.keyColumns <= 128.U)
    assert(io.start.bits.valueColumns > 0.U && io.start.bits.valueColumns <= (2 * t).U)
    early := io.earlyE.getOrElse(false.B)
    skipped := io.skipE0.getOrElse(false.B)
    seededPanels.foreach(x => for (g <- 0 until 4) x(g) := 0.U)
    active := true.B
    cmd := io.start.bits
    clearHalf := io.skipE0.getOrElse(false.B)
    clearPending := "hffff".U
    clearing := true.B
    secondHalf := io.start.bits.valueColumns > t.U
    wAddress := Mux(io.earlyE.getOrElse(false.B), 0.U, io.start.bits.valueColumns)
    seedEnds(0) := Mux(io.start.bits.valueColumns > t.U, t.U, io.start.bits.valueColumns) - 1.U
    seedEnds(1) := io.start.bits.valueColumns - 1.U
    for (hh <- 0 until 2) {
      panelMasks(hh) := VecInit(
        (0 until 16).map(p => (hh * t + (p % 4) * side).U < io.start.bits.valueColumns)
      ).asUInt
      valueMasks(hh) := VecInit(
        (0 until t).map(c => (hh * t + c).U < io.start.bits.valueColumns)
      ).asUInt
    }
    for (g <- 0 until 4) {
      seedCount(g) := Mux(io.skipE0.getOrElse(false.B), t.U, 0.U)
      seededHalves(g) := Mux(io.skipE0.getOrElse(false.B), 1.U, 0.U)
    }
    for (f <- 0 until 2) {
      live(f) := false.B
      ready(f) := 0.U
    }
    fetch := 0.U
    fetchFrame := false.B
    requestKind := 0.U
    reading := false.B
    head := false.B
    half := false.B
    capturedPanels := 0.U
    draining := false.B
    rowFetch := Mux(io.skipE0.getOrElse(false.B), t.U, 0.U)
    rowFinish := Mux(io.skipE0.getOrElse(false.B), t.U, 0.U)
    rowActive := false.B
    rowPending := 0.U
    zCleared.foreach(z => for (p <- 0 until 16) z(p) := 0.U)
    panelHeads.foreach(x => for (p <- 0 until 16) x(p) := false.B)
    panelPhases.foreach(x =>
      for (p <- 0 until 16)
        x(p) :=
          (if (columnLookahead) Mux(io.skipE0.get, 2.U, 1.U)
           else Mux(io.skipE0.getOrElse(false.B), 1.U, 0.U))
    )
    completed.foreach(x => for (f <- 0 until 2) x(f) := 0.U)
  }
  for (p <- 0 until 16) {
    io.clear(p).valid := active && clearing && clearPending(p)
    io.clear(p).bits := physicalContext(Cat(clearHalf, 0.U(1.W)))
    if (joint) {
      val eligible = io.joint.get.released(p) & ~zCleared.get(p)
      val h = PriorityEncoder(eligible)
      when(!clearing && !early) {
        io.clear(p).valid := active && eligible.orR
        io.clear(p).bits := Cat(h, 1.U(1.W))
      }
      when(io.clear(p).fire && !clearing) { zCleared.get(p) := zCleared.get(p) | (1.U(2.W) << h) }
    }
    io.seed(p).valid := false.B
    io.seed(p).bits := 0.U.asTypeOf(io.seed(p).bits)
  }
  val clearFires = VecInit(io.clear.map(_.fire)).asUInt
  io.joint.foreach { j =>
    j.issueZ := VecInit(io.input.map(p => p.fire && p.bits.context(0))).asUInt
    j.clearZ := Mux(clearing, 0.U, clearFires)
    j.clearContext := VecInit(io.clear.map(_.bits(1))).asUInt
  }
  when(clearing) {
    clearPending := clearPending & ~clearFires
    when((clearPending & ~clearFires) === 0.U) {
      when(clearHalf || early) { clearing := false.B }.otherwise {
        clearHalf := true.B
        clearPending := "hffff".U
      }
    }
  }
  for (g <- 0 until 4) {
    uQueues(g).io.enq <> io.u(g)
    val q = uQueues(g).io.deq
    val p = (q.bits.column >> log2Ceil(side))(1, 0)
    val context = Cat(q.bits.column(log2Ceil(t)), 0.U(1.W))
    q.ready := active && !clearing && io.seed((g * 4).U(4.W) + p).ready &&
      eSeeds
        .map(x => !x(g).io.deq.valid)
        .getOrElse(true.B)
    for (c <- 0 until 4) {
      val port = io.seed(g * 4 + c)
      port.valid := active && !clearing && q.valid && p === c.U &&
        eSeeds
          .map(x => !x(g).io.deq.valid)
          .getOrElse(true.B)
      port.bits.context := physicalContext(context)
      port.bits.row := q.bits.column(log2Ceil(side) - 1, 0)
      for (r <- 0 until side)
        port.bits.values(r) :=
          (if (directResidual) q.bits.values(r)
           else Cat(q.bits.values(r), 0.U(16.W)))
    }
    io.seeded(g) := Mux(q.fire, q.bits.column.zext, (-1).S)
    when(q.fire) {
      seededPanels.foreach(x =>
        when(q.bits.column(log2Ceil(side) - 1, 0) === (side - 1).U) {
          x(g) := x(g) | (1.U(4.W) << p)
        }
      )
      assert(q.bits.column === seedCount(g) && q.bits.column < cmd.valueColumns)
      seedCount(g) := seedCount(g) + 1.U
      seededHalves(g) := seededHalves(g) | Cat(
        q.bits.column === seedEnds(1),
        q.bits.column === seedEnds(0)
      )
    }
  }
  if (directResidual) {
    // Returned E replaces only its already-captured R column. No matrix-wide
    // copy and no extra C32. Re-seeding wins over further column extraction;
    // otherwise a full return queue could block the reads that release it.
    for (g <- 0 until 4) {
      val e = eSeeds.get(g).io.deq
      val panel = (e.bits.column >> log2Ceil(side))(1, 0)
      e.ready := active && io.seed((g * 4).U(4.W) + panel).ready
      for (c <- 0 until 4) when(active && e.valid && panel === c.U) {
        val port = io.seed(g * 4 + c)
        port.valid := true.B
        port.bits.context := Cat(e.bits.column(log2Ceil(t)), 0.U(1.W))
        port.bits.row := e.bits.column(log2Ceil(side) - 1, 0)
        for (r <- 0 until side) port.bits.values(r) := Cat(e.bits.values(r), 0.U(16.W))
      }
      when(e.fire) {
        assert(e.bits.column === eSeedCount.get(g))
        eSeedCount.get(g) := eSeedCount.get(g) + 1.U
      }
    }
  }
  // Allocation reserves a frame no later than its first synchronous read.
  // With pipelinedReads the reservation and W/Q request may share a cycle;
  // the non-flowing tag FIFO prevents any response before ownership registers.
  // A frame cannot be reused until all
  // sixteen destinations have captured each required half into registered inputs.
  val allWritten = io.written.map(x => x(fetch(6, 0))).reduce(_ && _)
  val queryWritten =
    early || io.joint.map(j => j.written.map(x => x(fetch(6, 0))).reduce(_ && _)).getOrElse(true.B)
  val allocate = active && !draining && !reading && fetch < cmd.keyColumns && !live(
    fetchFrame
  ) && allWritten && queryWritten && io.enable
  io.allocated := allocate
  when(allocate) {
    live(fetchFrame) := true.B
    ready(fetchFrame) := 0.U
    k(fetchFrame) := fetch
    reading := true.B
    requestKind := 0.U
    completed.foreach(_(fetchFrame) := 0.U)
  }
  val needed = Wire(Vec(16, Bool()))
  for (b <- 0 until 16) {
    needed(b) := Mux(requestKind === 0.U, (b < 4 * words).B, (b < hBanks).B)
  }
  val readOwner = reading || (if (pipelinedReads) allocate else false.B)
  val doRead = Wire(Bool())
  val reserveRead = Wire(Bool())
  val address = Mux(requestKind === 0.U, wAddress, Cat(1.U(1.W), fetch(6, 0), requestKind(1)))
  for (b <- 0 until 16) {
    io.read(b).valid := false.B
    io.read(b).bits := address
  }
  io.joint.foreach { j =>
    for (b <- 0 until 16) {
      j.read(b).valid := false.B
      j.read(b).bits := Cat(1.U(1.W), 0.U(1.W), fetch(6, 0))
    }
  }

  if (independentReads) {
    // The allocated frame owns all return storage. Reserve its original tag
    // before physical grants, including when every bank is blocked. Hold
    // kind/frame/address until all banks accept; the tag enqueue no longer
    // depends on the SRAM collision/arbiter-ready path.
    val workBanks = math.max(4 * words, hBanks)
    val queryBanks = if (joint) 4 * words else 0
    val bankCount = workBanks + queryBanks
    val localGrants = if (localGrantEpoch) Some(Module(new LocalBankGrants(bankCount))) else None
    val originalGrants = if (!localGrantEpoch) Some(RegInit(0.U(bankCount.W))) else None
    val granted = localGrants.map(_.io.granted).getOrElse(originalGrants.get)
    val partial = RegInit(false.B)
    val offer = active && readOwner && (partial || tags.io.enq.ready)
    val workNeeded = VecInit((0 until workBanks).map(b => needed(b))).asUInt
    val queryNeeded = Mux(
      !early && requestKind === 0.U,
      ((BigInt(1) << queryBanks) - 1).U(queryBanks.max(1).W),
      0.U
    )
    val wanted = if (joint) Cat(queryNeeded, workNeeded) else workNeeded

    for (b <- 0 until workBanks) {
      io.read(b).valid := offer && needed(b) && !granted(b) && io.bankEnable(b)
    }
    io.joint.foreach { j =>
      for ((b, i) <- (4 * words until 8 * words).zipWithIndex) {
        j.read(b).valid := offer && queryNeeded(i) && !granted(workBanks + i) && io.bankEnable(b)
      }
    }
    val workFires = VecInit((0 until workBanks).map(b => io.read(b).fire)).asUInt
    val accepted = if (joint) {
      val queryFires = VecInit(
        (4 * words until 8 * words).map(b => io.joint.get.read(b).fire)
      ).asUInt
      Cat(queryFires, workFires)
    } else workFires
    reserveRead := offer && !partial
    doRead := offer && (((granted | accepted) & wanted) === wanted)
    localGrants.foreach { grants =>
      grants.io.inRequest := active && readOwner
      grants.io.offer := offer
      grants.io.needed := wanted
      grants.io.accepted := accepted
      doRead := grants.io.complete
    }
    when(offer) { partial := !doRead }
    originalGrants.foreach { old =>
      when(accepted.orR) { old := Mux(doRead, 0.U, old | accepted) }
    }
    assert(!(accepted & granted).orR, "projection bank accepted twice")
    when(partial) {
      assert(
        active && reading && tags.io.count =/= 0.U,
        "partial projection lost its frame or return tag"
      )
    }
    when(io.start.fire) { assert(!partial, "projection restarted with unfinished bank grants") }
  } else {
    // Atomic reference retained for the admission unit comparison only.
    val banksReady = (0 until 16)
      .map { b =>
        !needed(b) || (io.read(b).ready && io.bankEnable(b))
      }
      .reduce(_ && _)
    val queryReady = early ||
      io.joint
        .map { j =>
          (4 * words until 8 * words).map(b => j.read(b).ready && io.bankEnable(b)).reduce(_ && _)
        }
        .getOrElse(true.B)
    doRead :=
      active && readOwner && tags.io.enq.ready && banksReady && (requestKind =/= 0.U || queryReady)
    reserveRead := doRead
    for (b <- 0 until 16) { io.read(b).valid := doRead && needed(b) }
    io.joint.foreach { j =>
      for (b <- 4 * words until 8 * words) {
        j.read(b).valid := doRead && !early && requestKind === 0.U
      }
    }
  }
  tags.io.enq.valid := reserveRead
  tags.io.enq.bits.frame := fetchFrame
  tags.io.enq.bits.kind := requestKind
  when(reserveRead) {
    assert(
      tags.io.enq.ready && (live(fetchFrame) || allocate),
      "projection read must reserve its original frame and return tag"
    )
  }
  io.request := Mux(doRead, (fetch * 3.U + requestKind).zext, (-1).S)
  when(doRead) {
    when(requestKind === halves) {
      reading := false.B
      fetch := fetch + 1.U
      fetchFrame := !fetchFrame
      wAddress := wAddress + 1.U
      if (pipelinedReads) requestKind := 0.U
    }.otherwise { requestKind := requestKind + 1.U }
    if (pipelinedReads)
      assert(live(fetchFrame) || allocate, "a read must reserve its response frame")
  }
  if (pipelinedReads) when(allocate) {
    assert(requestKind === 0.U, "new frames begin with W/Q, even after a stalled reservation")
  }
  val rt = tags.io.deq.bits
  val available = (0 until 16)
    .map(b =>
      Mux(
        rt.kind === 0.U,
        (b >= 4 * words).B || io.response(b).valid,
        (b >= hBanks).B || io.response(b).valid
      )
    )
    .reduce(_ && _)
  val queryAvailable = early ||
    io.joint
      .map(j => (4 * words until 8 * words).map(b => j.response(b).valid).reduce(_ && _))
      .getOrElse(true.B)
  tags.io.deq.ready := available && (rt.kind =/= 0.U || queryAvailable)
  val capture = tags.io.deq.fire
  for (b <- 0 until 16)
    io.response(b).ready := capture && Mux(rt.kind === 0.U, (b < 4 * words).B, (b < hBanks).B)
  io.joint.foreach(j =>
    for (b <- 0 until 16)
      j.response(b).ready :=
        capture && !early && rt.kind === 0.U && (b >= 4 * words && b < 8 * words).B
  )
  io.captured := Mux(capture, (k(rt.frame) * 3.U + rt.kind).zext, (-1).S)
  when(capture) {
    assert(live(rt.frame))
    ready(rt.frame) := ready(rt.frame) | (1.U(3.W) << rt.kind)
    when(rt.kind === 0.U) {
      for (i <- 0 until t) {
        val b = (i / side) * words + (i % side) / 8
        val lane = (i % side) % 8
        w(rt.frame)(i) := io.response(b).bits(16 * lane + 15, 16 * lane)
      }
      if (joint) for (i <- 0 until t) {
        val b = 4 * words + (i / side) * words + (i % side) / 8
        val lane = (i % side) % 8
        q.get(rt.frame)(i) := io.joint.get.response(b).bits(16 * lane + 15, 16 * lane)
      }
    }
  }
  // Shared vector of converters: H snapshot traffic finishes before E rows are
  // extracted. Conversion outputs always enter frame/result registers.
  val convert = Wire(Vec(t, UInt(32.W)))
  val narrowed = Wire(Vec(t, UInt(16.W)))
  val rowValues = Wire(Vec(t, UInt(32.W)))
  for (i <- 0 until t) {
    convert(i) := Mux(
      draining,
      rowValues(i),
      io.response(i / 4).bits(32 * (i % 4) + 31, 32 * (i % 4))
    )
    narrowed(i) := FloatArithmetic.narrow(convert(i))
    when(capture && rt.kind =/= 0.U) {
      h(rt.frame)((rt.kind - 1.U)(0))(i) := Mux(valueMasks((rt.kind - 1.U)(0))(i), narrowed(i), 0.U)
    }
  }
  val activePanels = panelMasks(half)
  val hasData = live(head) && ready(head)(0) && Mux(half, ready(head)(2), ready(head)(1))
  val panelCompleted = Wire(Vec(16, Bool()))
  for (p <- 0 until 16) panelCompleted(p) := false.B
  for (p <- 0 until 16) {
    val g = p / 4
    val c = p % 4
    val f = panelHeads.map(_(p)).getOrElse(head)
    val phase = panelPhases.map(_(p)).getOrElse(Cat(half, 0.U(1.W)))
    // The column candidate uses one-hot E0/Z0/E1/Z1 control. Data context
    // addresses stay binary; phase readiness need not decode nested muxes.
    val ctx = if (columnLookahead) Cat(phase(2) || phase(3), phase(1) || phase(3)) else phase
    if (columnLookahead) assert(PopCount(phase) === 1.U)
    val hh = ctx(1)
    val z = ctx(0)
    val initialized =
      if (columnLookahead)
        Mux(
          early,
          seededPanels.get(g)(c),
          Mux1H(
            phase.asBools,
            Seq(seededHalves(g)(0), zCleared.get(p)(0), seededHalves(g)(1), zCleared.get(p)(1))
          )
        )
      else if (joint) Mux(z, zCleared.get(p)(hh), seededHalves(g)(hh))
      else seededHalves(g)(half)
    // A released frame has all completion bits set; allocation clears both
    // ready and completed before any response can arrive. Hence completed
    // already excludes stale payload. Avoid the redundant, high-fanout live
    // mux in the admission -> completion -> frame-reuse feedback cone.
    val halfReady =
      if (columnLookahead)
        ((phase(0) || phase(1)) && ready(f)(1)) || ((phase(2) || phase(3)) && ready(f)(2))
      else Mux(hh, ready(f)(2), ready(f)(1))
    val dataReady = if (joint) ready(f)(0) && halfReady && !completed.get(f)(p) else hasData
    val enabled = panelMasks(hh)(p)
    io.input(p).valid := active && !draining && dataReady && initialized && enabled &&
      (if (joint) true.B
       else
         !capturedPanels(p))
    if (columnLookahead) {
      val firstReady = ready(f)(1) && panelMasks(0)(p)
      val secondReady = ready(f)(2) && panelMasks(1)(p)
      val stageReady =
        (phase(0) && firstReady && seededHalves(g)(0)) ||
          (phase(1) && firstReady && zCleared.get(p)(0)) ||
          (phase(2) && secondReady && seededHalves(g)(1)) ||
          (phase(3) && secondReady && zCleared.get(p)(1))
      val canIssue = ready(f)(0) && !completed.get(f)(p) &&
        Mux(early, firstReady && seededPanels.get(g)(c), stageReady)
      assert(canIssue === (dataReady && initialized && enabled))
      io.input(p).valid := active && !draining && canIssue
    }
    io.input(p).bits.os := true.B
    io.input(p).bits.context := physicalContext(ctx)
    io.input(p).bits.tag := k(f)(1, 0)
    rCompleted.foreach { done =>
      when(io.input(p).fire && !z) {
        assert(!done(p)(hh), "R context cannot be updated after publication permission")
        when(k(f) === cmd.keyColumns - 1.U) { done(p) := done(p) | (1.U(2.W) << hh) }
      }
    }
    for (i <- 0 until side) {
      val negativeW = w(f)(g * side + i) ^ "h8000".U
      io.input(p).bits.west(i) :=
        (if (joint) Mux(z, q.get(f)(g * side + i), negativeW)
         else negativeW)
      io.input(p).bits.north(i) := h(f)(hh)(c * side + i)
    }
    if (joint) {
      io.joint.get.contexts(p) := ctx
      io.joint.get.keys(p) := k(f)
      // Masked tail panels advance without consuming a MAC. They still obey
      // the same frame version, so an old live slot cannot be consumed twice.
      val progressed = active && !draining && dataReady && (io.input(p).fire || !enabled)
      when(progressed) {
        val last =
          if (columnLookahead) phase(3) || (!secondHalf && phase(1))
          else ctx === Mux(secondHalf, 3.U, 1.U)
        when(early || last) {
          panelPhases.get(p) :=
            (if (columnLookahead) Mux(skipped, 2.U, 1.U)
             else Mux(skipped, 1.U, 0.U))
          panelHeads.get(p) := !f
          panelCompleted(p) := true.B
        }.otherwise {
          panelPhases.get(p) := (if (columnLookahead) (phase << 1)(3, 0) else ctx + 1.U)
        }
      }
    }
  }
  val fires = VecInit(io.input.map(_.fire)).asUInt
  io.issued := fires
  io.issueKey := (k(head) << 1) + half.asUInt
  if (joint) {
    for (f <- 0 until 2) {
      when(active && !live(f)) {
        assert(
          ready(f) === 0.U || completed.get(f).andR,
          "an unowned projection frame must not expose unfinished ready payload"
        )
      }
      val takes = VecInit(
        (0 until 16).map(p => panelCompleted(p) && panelHeads.get(p) === f.U)
      ).asUInt
      when(takes.orR) { completed.get(f) := completed.get(f) | takes }
      when(live(f) && (completed.get(f) | takes).andR) { live(f) := false.B }
    }
  } else
    when(active && hasData) {
      capturedPanels := capturedPanels | fires
      when(((capturedPanels | fires) & activePanels) === activePanels) {
        capturedPanels := 0.U
        when(half || halves === 1.U) {
          live(head) := false.B
          head := !head
          half := false.B
        }.otherwise { half := true.B }
      }
    }
  io.frames := live.asUInt
  when(active && fetch === cmd.keyColumns && !reading && !live.asUInt.orR && !tags.io.deq.valid) {
    draining := true.B
  }
  // Rows remain C32-owned until every extraction request and response completes.
  val rowTag = Wire(new ProjectionRowTag(side, joint))
  def decodeRow(index: UInt): ProjectionRowTag = {
    val r = Wire(new ProjectionRowTag(side, joint))
    if (directResidual) {
      // Z is already final while the triangular wave advances. Publish it
      // first, then each completed E token group. With two response slots,
      // contiguous rows need no alternating-group workaround.
      val z = index < Mux(secondHalf, (2 * t).U, t.U)
      val local = Mux(z, index, index - Mux(secondHalf, (2 * t).U, t.U))
      r.half := local(log2Ceil(t))
      r.group := local(log2Ceil(t) - 1, log2Ceil(side))
      r.row := local(log2Ceil(side) - 1, 0)
      r.z.get := z
    } else {
      r.half := index(log2Ceil(t) + (if (joint && !residentZ) 1 else 0))
      r.group := (if (joint) index(1, 0) else (index >> log2Ceil(side))(1, 0))
      r.row := (if (joint) (index >> 2)(log2Ceil(side) - 1, 0) else index(log2Ceil(side) - 1, 0))
      r.z.foreach(_ := (if (residentZ) false.B else index(log2Ceil(t))))
    }
    r
  }
  rowTag := decodeRow(rowFetch)
  def rowReady(tag: ProjectionRowTag): Bool = if (directResidual)
    tag.z.get || eSeedCount.get(tag.group) > seedEnds(tag.half)
  else true.B
  val extractionReady = rowReady(rowTag)
  val takeRow =
    active && draining && extractionReady && rowFetch < totalRows && !rowActive &&
      rowTags.io.enq.ready
  rowTags.io.enq.valid := takeRow
  rowTags.io.enq.bits := rowTag
  for (p <- 0 until 16) {
    rowRead(p).valid := rowActive && rowPending(p % 4) && rowTag.group === (p / 4).U
    rowRead(p).bits.context := physicalContext(Cat(rowTag.half, rowTag.z.getOrElse(false.B)))
    rowRead(p).bits.row := rowTag.row
  }
  val rowFires = VecInit(
    (0 until 4).map(c => (0 until 4).map(g => rowRead(4 * g + c).fire).reduce(_ || _))
  ).asUInt
  val rowComplete = rowActive && (rowPending & ~rowFires) === 0.U
  val refill =
    (if (joint)
       rowComplete && rowFetch + 1.U < totalRows && rowTags.io.enq.ready && rowReady(
         decodeRow(rowFetch + 1.U)
       )
     else false.B)
  if (joint) {
    // A different panel group follows every row, so the depth-one panel
    // response queues can drain without throttling the whole row publisher.
    rowTags.io.enq.valid := takeRow || refill
    rowTags.io.enq.bits := Mux(refill, decodeRow(rowFetch + 1.U), rowTag)
  }
  when(takeRow) {
    rowActive := true.B
    rowPending := 15.U
  }
  when(rowActive) {
    rowPending := rowPending & ~rowFires
    when(rowComplete) {
      rowActive := refill
      rowFetch := rowFetch + 1.U
      when(refill) { rowPending := 15.U }
    }
  }
  val tag = rowTags.io.deq.bits
  val rowsValid = (0 until 16)
    .map(p => tag.group =/= (p / 4).U || rowResponse(p).valid)
    .reduce(_ && _)
  val isZ = tag.z.getOrElse(false.B)
  val destinationReady = zResults
    .map(r => Mux(isZ, r.io.enq.ready, results.io.enq.ready))
    .getOrElse(results.io.enq.ready)
  val takeResult = active && draining && rowTags.io.deq.valid && rowsValid && destinationReady
  rowTags.io.deq.ready := active && draining && rowsValid && destinationReady
  results.io.enq.valid := active && draining && rowTags.io.deq.valid && rowsValid && !isZ
  results.io.enq.bits.group := tag.group
  results.io.enq.bits.half := tag.half
  results.io.enq.bits.row := tag.row
  for (i <- 0 until t) {
    rowValues(i) := Mux1H(
      (0 until 4).map(g =>
        (tag.group === g.U) -> rowResponse(g * 4 + i / side).bits.values(i % side)
      )
    )
    results.io.enq.bits.values(i) := Mux(valueMasks(tag.half)(i), narrowed(i), 0.U)
  }
  for (p <- 0 until 16) {
    rowResponse(p).ready := takeResult && tag.group === (p / 4).U
    when(rowResponse(p).fire) {
      assert(
        rowResponse(p).bits.context === physicalContext(Cat(tag.half, isZ)) && rowResponse(
          p
        ).bits.row === tag.row
      )
    }
  }
  zResults.foreach { r =>
    r.io.enq.valid := active && draining && rowTags.io.deq.valid && rowsValid && isZ
    r.io.enq.bits.group := tag.group
    r.io.enq.bits.half := tag.half
    r.io.enq.bits.row := tag.row
    for (i <- 0 until t) r.io.enq.bits.values(i) := Mux(valueMasks(tag.half)(i), rowValues(i), 0.U)
  }
  io.rowRequested := Mux(refill, (rowFetch + 1.U).zext, Mux(takeRow, rowFetch.zext, (-1).S))
  io.rowCaptured := Mux(
    takeResult,
    (tag.half.asUInt *
      (if (joint) 2 * t
       else t).U + isZ.asUInt * t.U + tag.group * side.U + tag.row).zext,
    (-1).S
  )
  val commits = io.result.fire.asUInt +& io.joint.map(_.result.fire.asUInt).getOrElse(0.U)
  when(commits =/= 0.U) { rowFinish := rowFinish + commits }
  when(active && draining && rowFinish === totalRows) {
    active := false.B
    io.done := true.B
  }
  residualBridge.foreach { bridge =>
    // Permission follows the last feature admission for each even context;
    // the panel's contextIdle separately waits for arithmetic retirement.
    bridge.io.enable := active
    for (p <- 0 until 16) {
      val arb = Module(new RRArbiter(new RetimedRow(side), 2))
      val owners = Module(new Queue(Bool(), 2, pipe = false, flow = false))
      arb.io.in(0) <> rowRead(p)
      arb.io.in(1) <> bridge.io.read(p)
      io.rowRead(p).bits := arb.io.out.bits
      io.rowRead(p).valid := arb.io.out.valid && owners.io.enq.ready && !io.seed(p).valid
      arb.io.out.ready := io.rowRead(p).ready && owners.io.enq.ready && !io.seed(p).valid
      io.residual.get.readColumn(p) := arb.io.chosen === 1.U
      owners.io.enq.valid := io.rowRead(p).fire
      owners.io.enq.bits := arb.io.chosen === 1.U
      rowResponse(p).valid := io.rowResponse(p).valid && owners.io.deq.valid && !owners.io.deq.bits
      bridge.io
        .response(p)
        .valid := io.rowResponse(p).valid && owners.io.deq.valid && owners.io.deq.bits
      bridge.io.response(p).bits := io.rowResponse(p).bits
      io.rowResponse(p).ready := owners.io.deq.valid && Mux(
        owners.io.deq.bits,
        bridge.io.response(p).ready,
        rowResponse(p).ready
      )
      owners.io.deq.ready := io.rowResponse(p).fire
    }
  }
}
