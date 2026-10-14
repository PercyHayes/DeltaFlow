package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.memory._
import deltaflow.arithmetic.FloatArithmetic

/** Real SRAM adapters + borrowed FP32 scaling + resident-Z H/O control. No MAC, FP32 lane, C32 or
  * SRAM macro is instantiated here. N preparation uses released K read ports while the prefix can
  * still project E/Z.
  */
class GdnStateOutputBackend(
    val side: Int = 16,
    val streamed: Boolean = false,
    val stateEpilogue: Boolean = false,
    val columnLookahead: Boolean = false,
    val registerCompletion: Boolean = false,
    val captureStateRowRange: Boolean = false,
    val cacheSecondHalf: Boolean = false,
    val captureBankDescriptor: Boolean = false,
    val qualifiedHReturn: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  val t = 4 * side
  val words = (side + 7) / 8
  require(!stateEpilogue || streamed)
  require(!columnLookahead || stateEpilogue)
  require(!registerCompletion || stateEpilogue)
  require(!captureStateRowRange || stateEpilogue)
  require(!cacheSecondHalf || stateEpilogue)
  require(!captureBankDescriptor || stateEpilogue)
  require(!qualifiedHReturn || stateEpilogue)
  val io = IO(new Bundle {
    val start = Input(Valid(new TransformCommand))
    val canPrepare = Input(Bool())
    val prefixReady = Input(Bool())
    val decay = Input(UInt(32.W))
    val scale = Flipped(new BorrowedScalePorts(side))
    val bankEnable = Input(UInt(16.W))
    val panelEnable = Input(UInt(16.W))
    val operandRead = Vec(16, Decoupled(UInt(10.W)))
    val operandResponse = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val operandWrite = Vec(16, Decoupled(new BankWrite(1024, 16)))
    val operandOwner = Output(UInt(16.W))
    val workRead = Vec(16, Decoupled(UInt(10.W)))
    val workResponse = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val workWrite = Vec(16, Decoupled(new BankWrite(512, 16)))
    val workOwner = Output(Bool())
    val input = Vec(16, Decoupled(new RetimedPanelInput(side, 2)))
    val seed = Vec(16, Decoupled(new RetimedRowData(side)))
    val clear = if (stateEpilogue) Some(Vec(16, Decoupled(UInt(2.W)))) else None
    val rowRead = Vec(16, Decoupled(new RetimedRow(side)))
    val rowResponse = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val ownsPanels = Output(Bool())
    val done = Output(Bool())
    val issued = Output(UInt(16.W))
    val nDone = Output(Bool())
    val scaleIssued = Output(Bool())
    val committed = Output(Valid(UInt(9.W)))
    val h0Ready = if (columnLookahead) Some(Output(UInt(128.W))) else None
    val c1Released = Output(Bool()) // O of the first value half is fully committed.
    // Logical O address from the output engine, independent of H return
    // arbitration. Consumers use it only for a valid O bank write.
    val outputAddress = Output(UInt(9.W))
    // Valid H writes use the seed-return row directly. Arbitration chooses
    // the kind, while neither page address is selected through that arbiter.
    val stateAddress = Output(UInt(9.W))
    val outputWrite = Output(Bool())
  })
  val command = Reg(new TransformCommand)
  val active = RegInit(false.B)
  val epoch = RegInit(0.U(32.W))
  val nStarted = RegInit(false.B)
  val nFinished = RegInit(false.B)
  val owned = RegInit(false.B)
  val half = RegInit(false.B)
  val launched = RegInit(false.B)
  val engine = Module(
    new StateOutputController(
      side,
      residentZ = true,
      groupedSeeds = streamed,
      zeroState = stateEpilogue,
      outputFirst = columnLookahead,
      registeredResultHead = true
    )
  )
  if (stateEpilogue) io.clear.get <> engine.io.clear.get
  val epilogueRows =
    if (stateEpilogue)
      Some(Module(new Queue(new StateOutputRow(side), 4, pipe = false, flow = false)))
    else None
  val engineFinished = if (stateEpilogue) Some(RegInit(false.B)) else None
  val finishCondition = WireDefault(engine.io.done)
  val completionPending = if (registerCompletion) Some(RegInit(false.B)) else None
  val backendFinish = completionPending.getOrElse(finishCondition)

  // All H/O writes and SRAM reader returns have retired before this condition.
  // Hold ownership for one further cycle, then publish a single completion.
  // This cuts the drained-status -> next-phase -> SRAM-admission path without
  // changing array feedback, payload capacity or any arithmetic operation.
  completionPending.foreach { pending =>
    when(finishCondition) { pending := true.B }
    when(pending || io.start.valid) { pending := false.B }
    when(pending) {
      assert(finishCondition)
      assert(active && owned && launched)
      assert(!io.start.valid)
    }
  }
  val operand = Module(
    new BankVectorReader(registeredOccupancy = true, captureAtAdmission = captureBankDescriptor)
  )
  val work = Module(
    new BankVectorReader(registeredOccupancy = true, captureAtAdmission = captureBankDescriptor)
  )
  val pack = Module(new NPanelPacker(1024))
  io.operandRead <> operand.io.read
  operand.io.response <> io.operandResponse
  operand.io.bankEnable := io.bankEnable
  io.workRead <> work.io.read
  work.io.response <> io.workResponse
  work.io.bankEnable := io.bankEnable
  io.operandWrite <> pack.io.write
  io.input <> engine.io.input
  io.seed <> engine.io.panelSeed
  io.rowRead <> engine.io.rowRead
  engine.io.rowResponse <> io.rowResponse
  engine.io.panelEnable := io.panelEnable
  io.ownsPanels := owned
  io.workOwner := owned
  io.issued := engine.io.issued
  val firstOutputDone = if (columnLookahead) Some(RegInit(false.B)) else None
  val h0Committed = if (columnLookahead) Some(RegInit(0.U(128.W))) else None
  io.c1Released := owned && (half || firstOutputDone.getOrElse(false.B))
  if (columnLookahead) {
    io.h0Ready.get := h0Committed.get
    when(io.start.valid) {
      firstOutputDone.get := false.B
      h0Committed.get := 0.U
    }
    when(io.committed.valid && !io.committed.bits(log2Ceil(3 * t))) {
      when(io.committed.bits(log2Ceil(3 * t) - 1, 0) === (3 * t - 1).U) {
        firstOutputDone.get := true.B
      }
      when(io.committed.bits(log2Ceil(3 * t) - 1, 0) < command.keyColumns) {
        h0Committed.get := h0Committed.get |
          (1
            .U(128.W) << io.committed.bits(log2Ceil(3 * t) - 1, 0))
      }
    }
  }
  io.nDone := nFinished
  io.done := false.B
  val secondValueHalf = if (cacheSecondHalf) {
    Some(SecondValueHalf(io.start.valid, io.start.bits.valueColumns, t))
  } else None
  when(active) {
    secondValueHalf.foreach { second =>
      assert(
        second === (command.valueColumns > t.U),
        "cached value-half limit differs from the active command"
      )
    }
  }
  when(io.start.valid) {
    assert(!active && !owned && operand.io.idle && work.io.idle)
    assert(io.start.bits.keyColumns > 0.U && io.start.bits.keyColumns <= (2 * t).U)
    active := true.B
    command := io.start.bits
    epoch := epoch + 1.U
    nStarted := false.B
    nFinished := false.B
    half := false.B
    launched := false.B
  }
  pack.io.start.valid := active && !nStarted && io.canPrepare
  pack.io.start.bits.base := 512.U
  pack.io.start.bits.columns := command.keyColumns
  pack.io.start.bits.epoch := epoch
  when(pack.io.start.fire) { nStarted := true.B }
  pack.io.completed.ready := true.B
  when(pack.io.done) { nFinished := true.B }
  val producing = active && nStarted && !nFinished
  when(active && nFinished && io.prefixReady && !owned) { owned := true.B }
  io.operandOwner := Mux(owned, 65535.U, Mux(producing, ((1 << (4 * words)) - 1).U, 0.U))
  engine.io.start.valid := owned && !launched
  engine.io.start.bits.epoch := epoch
  engine.io.start.bits.half := half
  engine.io.oldStateReleased := io.prefixReady
  when(engine.io.start.fire) { launched := true.B }
  when(backendFinish) {
    assert(operand.io.idle && work.io.idle)
    val lastHalf = half ||
      secondValueHalf
        .map(second => !second)
        .getOrElse(command.valueColumns <= t.U)
    when(lastHalf) {
      active := false.B
      owned := false.B
      io.done := true.B
    }
      .otherwise {
        half := true.B
        launched := false.B
      }
  }
  val nFetch = RegInit(0.U(8.W))
  val nSent = RegInit(0.U(8.W))
  val nReturned = RegInit(0.U(8.W))
  when(io.start.valid) {
    nFetch := 0.U
    nSent := 0.U
    nReturned := 0.U
  }
  val seedFetch = RegInit(0.U(log2Ceil(2 * t + 1).W))
  // The H write address starts at this returned row tag. A registered head
  // removes the asynchronous tag-memory selection from the SRAM collision
  // path, using one of the original payload entries rather than another slot.
  val seedTags = Module(
    new StateRowTagQueue(if (stateEpilogue) 12 else 8, captureRange = captureStateRowRange)
  )
  seedTags.io.keyColumns := command.keyColumns
  // These exact registered queue-status copies cut pointer comparisons out
  // of the shared FP32 grant/return path. Capacity and acceptance cycles stay
  // unchanged; pointer-derived status remains independently checked each cycle.
  val seedRoom = WireDefault(seedTags.io.enq.ready)
  val seedPresent = WireDefault(seedTags.io.deq.valid)
  if (columnLookahead) {
    val occupied = RegInit(0.U(4.W))
    val room = RegInit(true.B)
    val present = RegInit(false.B)
    seedRoom := room
    seedPresent := present
    val put = seedTags.io.enq.valid
    val take = present && seedTags.io.deq.ready
    when(put =/= take) {
      occupied := Mux(put, occupied + 1.U, occupied - 1.U)
      room := Mux(put, occupied =/= 11.U, true.B)
      present := Mux(put, true.B, occupied =/= 1.U)
    }
    when(put) { assert(room && seedTags.io.enq.ready) }
    assert(occupied === seedTags.io.count && occupied <= 12.U)
    assert(room === seedTags.io.enq.ready && present === seedTags.io.deq.valid)
  }
  if (stateEpilogue) {
    when(engine.io.start.fire) { engineFinished.get := false.B }
    when(engine.io.done) { engineFinished.get := true.B }
    finishCondition := active && owned && launched && engineFinished.get && work.io.idle &&
      operand.io.idle && !epilogueRows.get.io.deq.valid && !seedPresent
  }
  val live = RegInit(VecInit(Seq.fill(2)(false.B)))
  val columns = Reg(Vec(2, new StateOutputColumn(side)))
  val nRequested = RegInit(VecInit(Seq.fill(2)(false.B)))
  val nReady = RegInit(VecInit(Seq.fill(2)(false.B)))
  val eReady = RegInit(VecInit(Seq.fill(2)(false.B)))
  val mReady = RegInit(VecInit(Seq.fill(2)(0.U(4.W))))
  val workKind = Reg(Vec(2, UInt(3.W)))
  val allocateHead = RegInit(false.B)
  val sendHead = RegInit(false.B)
  val nextToken = RegInit(0.U(log2Ceil(t + 1).W))
  when(engine.io.start.fire) {
    seedFetch := 0.U
    allocateHead := false.B
    sendHead := false.B
    nextToken := 0.U
    assert(!seedPresent)
    for (f <- 0 until 2) {
      live(f) := false.B
      nRequested(f) := false.B
      nReady(f) := false.B
      eReady(f) := false.B
      mReady(f) := 0.U
    }
  }
  when(launched && nextToken < t.U && !live(allocateHead)) {
    live(allocateHead) := true.B
    columns(allocateHead) := 0.U.asTypeOf(columns(allocateHead))
    columns(allocateHead).token := nextToken
    nRequested(allocateHead) := false.B
    nReady(allocateHead) := false.B
    eReady(allocateHead) := false.B
    mReady(allocateHead) := ((1.U(4.W) << (nextToken >> log2Ceil(side))) - 1.U)(3, 0)
    workKind(allocateHead) := 0.U
    allocateHead := !allocateHead
    nextToken := nextToken + 1.U
  }
  engine.io.column.valid := launched && live(sendHead) && nReady(sendHead) && eReady(
    sendHead
  ) && mReady(sendHead).andR
  engine.io.column.bits := columns(sendHead)
  when(engine.io.column.fire) {
    live(sendHead) := false.B
    sendHead := !sendHead
  }

  // N production and H seed reuse the SAME prepared floating-point lanes.
  // N and O share the wide rounding bank; M gather uses side extra converters.
  val result = if (stateEpilogue) Wire(Decoupled(new StateOutputRow(side))) else engine.io.result
  val selectState = WireDefault(false.B)
  val stateResultValid = WireDefault(
    engine.io.result.valid && engine.io.result.bits.index < (2 * t).U
  )
  val narrowed = Wire(Vec(t, UInt(16.W)))
  for (j <- 0 until t) {
    // H keeps FP32 throughout writeback; only O consumes this BF16 conversion.
    // Avoid routing H/O return arbitration through the rounding datapath.
    val outputValue =
      if (columnLookahead) engine.io.result.bits.values(j) else result.bits.values(j)
    narrowed(j) := FloatArithmetic.narrow(Mux(producing, io.scale.response.bits(j), outputValue))
  }
  val rawN = operand.io.result
  val rawW = work.io.result
  io.scale.request.valid := Mux(
    producing,
    rawN.valid,
    launched && rawW.valid && !rawW.bits.tag(9) && seedRoom &&
      epilogueRows.map(_.io.deq.valid).getOrElse(true.B)
  )
  for (j <- 0 until t) {
    val kval = rawN.bits
      .words((j / side) * words + (j % side) / 8)(16 * ((j % side) % 8) + 15, 16 * ((j % side) % 8))
    val hval = rawW.bits.words(j / 4)(32 * (j % 4) + 31, 32 * (j % 4))
    io.scale.request.bits(j).a := Mux(
      producing,
      Cat(kval, 0.U(16.W)),
      Mux((half.asUInt * t.U + j.U) < command.valueColumns, hval, 0.U)
    )
    io.scale.request.bits(j).b := Mux(producing, io.scale.tailGate(j), io.decay)
    io.scale.request.bits(j).c :=
      (if (stateEpilogue)
         Mux(producing, 0.U, epilogueRows.get.io.deq.bits.values(j))
       else 0.U)
  }
  io.scaleIssued := io.scale.request.fire
  pack.io.input.valid := producing && io.scale.response.valid
  pack.io.input.bits.column := nReturned(6, 0)
  for (j <- 0 until 64) pack.io.input.bits.values(j) := (if (j < t) narrowed(j) else 0.U)
  io.scale.response.ready := Mux(
    producing,
    pack.io.input.ready,
    engine.io.seed.ready && seedPresent
  )
  when(producing && io.scale.request.fire) {
    assert(rawN.bits.tag === nSent)
    nSent := nSent + 1.U
  }
  when(pack.io.input.fire) { nReturned := nReturned + 1.U }
  seedTags.io.enq.valid := !producing && io.scale.request.fire
  seedTags.io.enq.bits := rawW.bits.tag(7, 0)
  seedTags.io.deq.ready := !producing && engine.io.seed.ready && io.scale.response.valid
  engine.io.seed.valid := !producing && seedPresent && io.scale.response.valid
  engine.io.seed.bits.index := seedTags.io.deq.bits
  engine.io.seed.bits.values := io.scale.response.bits
  if (stateEpilogue) {
    val rows = epilogueRows.get
    val hasH = if (qualifiedHReturn) {
      HReturnPresent(producing, seedPresent, io.scale.response.valid)
    } else seedPresent && io.scale.response.valid && !producing
    if (columnLookahead) when(hasH) { assert(seedTags.io.deq.bits < (2 * t).U) }
    val isOutput = engine.io.result.bits.index >= (2 * t).U
    // Keep the selected row across partial bank writes. An arriving H FMA
    // response must not steal the completion mask from a stalled O row.
    val locked = RegInit(false.B)
    val heldH = Reg(Bool())
    val chooseH = Mux(locked, heldH, hasH)
    selectState := chooseH
    stateResultValid := chooseH && hasH
    result.valid := Mux(chooseH, hasH, engine.io.result.valid && isOutput)
    result.bits := engine.io.result.bits
    when(chooseH) {
      result.bits.index := seedTags.io.deq.bits
      result.bits.values := io.scale.response.bits
      result.bits.epoch := epoch
      result.bits.half := half
    }
    when(result.valid && !result.ready) {
      locked := true.B
      heldH := chooseH
    }
    when(result.fire) { locked := false.B }
    io.scale.response.ready := Mux(
      producing,
      pack.io.input.ready,
      result.ready && seedPresent && chooseH
    )
    seedTags.io.deq.ready := hasH && result.ready && chooseH
    engine.io.seed.valid := false.B
    rows.io.deq.ready := !producing && io.scale.request.fire
    when(rows.io.deq.fire) {
      assert(rows.io.deq.bits.index === rawW.bits.tag && rows.io.deq.bits.half === half)
    }
    engine.io.result.ready := Mux(
      isOutput,
      result.ready && !chooseH,
      work.io.request.ready && rows.io.enq.ready && launched
    )
    rows.io.enq.valid := engine.io.result.fire && !isOutput
    rows.io.enq.bits := engine.io.result.bits
  }

  val nChoices = VecInit((0 until 2).map(f => live(f) && !nRequested(f)))
  val nf = PriorityEncoder(nChoices.asUInt)
  val nt = columns(nf).token
  val njob = operand.io.request
  njob.valid := Mux(producing, nFetch < command.keyColumns, launched && nChoices.asUInt.orR)
  njob.bits.tag := Mux(producing, nFetch, nf)
  for (b <- 0 until 16) {
    val group = (b.U(4.W) - nt)(3, 0)
    njob.bits.addresses(b) := Mux(producing, nFetch, 512.U + (nt >> 4) * 16.U + group)
  }
  njob.bits.mask := Mux(
    producing,
    ((1 << (4 * words)) - 1).U,
    VecInit((0 until 16).map(b => (((b.U(4.W) - nt)(3, 0)) << 3) < command.keyColumns)).asUInt
  )
  when(njob.fire) {
    when(producing) { nFetch := nFetch + 1.U }.otherwise { nRequested(nf) := true.B }
  }
  rawN.ready := Mux(producing, io.scale.request.ready, true.B)
  when(rawN.fire && !producing) {
    val f = rawN.bits.tag(0)
    assert(live(f) && !nReady(f))
    nReady(f) := true.B
    for (k <- 0 until 2 * t) {
      val bank = (columns(f).token + (k / 8).U)(3, 0)
      columns(f).n(k) := Mux(
        k.U < command.keyColumns,
        rawN.bits.words(bank)(16 * (k % 8) + 15, 16 * (k % 8)),
        0.U
      )
    }
  }
  val wChoices = VecInit((0 until 2).map(f => live(f) && workKind(f) <= 4.U))
  val wf = PriorityEncoder(wChoices.asUInt)
  val wt = columns(wf).token
  val kind = workKind(wf)
  // In streaming mode E and M are in disjoint bank halves: one request
  // captures a complete token instead of 1 E plus up to 4 M gathers.
  // Interleave seed traffic while a token frame needs data, using only the
  // existing two frames and eight reserved SRAM responses.
  val seedTurn = if (streamed) Some(RegInit(true.B)) else None
  seedTurn.foreach(x => when(engine.io.start.fire) { x := true.B })
  val seedRequest =
    if (stateEpilogue)
      engine.io.result.valid && engine.io.result.bits.index < (2 * t).U &&
      epilogueRows.get.io.enq.ready
    else
      seedFetch < (2 * t).U && (if (streamed) seedTurn.get || !wChoices.asUInt.orR else true.B)
  val seedIndex =
    if (stateEpilogue) engine.io.result.bits.index
    else if (streamed)
      Cat(
        seedFetch(log2Ceil(side)),
        seedFetch(log2Ceil(2 * t) - 1, log2Ceil(side) + 1),
        seedFetch(log2Ceil(side) - 1, 0)
      )
    else seedFetch
  val mGroup = kind - 1.U
  val block = mGroup * (mGroup + 1.U) / 2.U + (wt >> log2Ceil(side))
  val mRow = block * side.U + wt(log2Ceil(side) - 1, 0)
  val mPart = mRow % (32 / side).U
  val wjob = work.io.request
  wjob.valid := launched && (seedRequest || wChoices.asUInt.orR)
  wjob.bits.tag := Mux(seedRequest, seedIndex, Cat(1.U(1.W), wf, 0.U(5.W), kind))
  for (b <- 0 until 16) {
    wjob.bits.addresses(b) := Mux(
      seedRequest,
      256.U + 2.U * seedIndex + half,
      (if (streamed) { if (b < 8) 128.U + 2.U * wt + half else wt }
       else
         Mux(kind === 0.U, 128.U + 2.U * wt + half, mRow / (32 / side).U))
    )
  }
  val mMask = VecInit(
    (0 until 16).map(b =>
      (b >= 8).B && (b - 8).S >= ((mPart * (side / 4).U).zext) && (b - 8).S <
        ((mPart * (side / 4).U + (side / 4).U).zext)
    )
  ).asUInt
  val packedMask =
    if (streamed)
      VecInit(
        (0 until 16).map(b =>
          if (b < t / 8) true.B
          else if (b >= 8 && b < 8 + t / 8) ((b - 8) * 8 + 7).U >= (wt >> log2Ceil(side)) * side.U
          else false.B
        )
      ).asUInt
    else 0.U
  wjob.bits.mask := Mux(
    seedRequest,
    Mux(seedIndex < command.keyColumns, ((1 << (t / 4)) - 1).U, 0.U),
    (if (streamed) packedMask else Mux(kind === 0.U, (((1 << (t / 8)) - 1) << 8).U, mMask))
  )
  when(wjob.fire) {
    when(seedRequest) { seedFetch := seedFetch + 1.U }
      .otherwise {
        workKind(wf) :=
          (if (streamed) 5.U
           else Mux(kind === 0.U, (wt >> log2Ceil(side)) +& 1.U, kind + 1.U))
      }
    seedTurn.foreach(_ := !seedRequest)
  }
  rawW.ready := Mux(
    rawW.bits.tag(9),
    true.B,
    io.scale.request.ready && seedRoom && !producing &&
      epilogueRows.map(_.io.deq.valid).getOrElse(true.B)
  )
  val f = rawW.bits.tag(8)
  val rk = rawW.bits.tag(2, 0)
  val returnedGroup = rk - 1.U
  val returnedBlock =
    returnedGroup * (returnedGroup + 1.U) / 2.U + (columns(f).token >> log2Ceil(side))
  val returnedPart =
    (returnedBlock * side.U + columns(f).token(log2Ceil(side) - 1, 0)) % (32 / side).U
  val mNarrow = Wire(Vec(side, UInt(16.W)))
  for (j <- 0 until side) {
    val bank = 8.U + returnedPart * (side / 4).U + (j / 4).U
    mNarrow(
      j
    ) :=
      (if (streamed) 0.U
       else FloatArithmetic.narrow(rawW.bits.words(bank(3, 0))(32 * (j % 4) + 31, 32 * (j % 4))))
  }
  when(rawW.fire && rawW.bits.tag(9)) {
    assert(live(f))
    if (streamed) {
      assert(rk === 0.U && !eReady(f))
      eReady(f) := true.B
      mReady(f) := 15.U
      for (j <- 0 until t) {
        columns(f).e(j) := rawW.bits.words(j / 8)(16 * (j % 8) + 15, 16 * (j % 8))
        // A b4 word can contain both a skipped and a valid Q block. Never
        // expose the uninitialized skipped half to the arithmetic array.
        columns(f).m(j) := Mux(
          (j / side).U < (columns(f).token >> log2Ceil(side)),
          0.U,
          rawW.bits.words(8 + j / 8)(16 * (j % 8) + 15, 16 * (j % 8))
        )
      }
    } else
      when(rk === 0.U) {
        assert(!eReady(f))
        eReady(f) := true.B
        for (j <- 0 until t)
          columns(f).e(j) := rawW.bits.words(8 + j / 8)(16 * (j % 8) + 15, 16 * (j % 8))
      }.otherwise {
        val g = (rk - 1.U)(1, 0)
        assert(!mReady(f)(g))
        mReady(f) := mReady(f) | (1.U(4.W) << g)
        for (j <- 0 until side) columns(f).m(Cat(g, j.U(log2Ceil(side).W))) := mNarrow(j)
      }
  }
  val isO = result.bits.index >= (2 * t).U
  io.outputWrite := GdnWriteSelection.output(engine.io.result.bits.index, selectState, side)
  val outputResultValid = engine.io.result.valid && io.outputWrite
  val stateRow = if (stateEpilogue) seedTags.io.deq.bits else engine.io.result.bits.index
  io.stateAddress := GdnStateAddress(stateRow, half, side)
  io.outputAddress := Cat(
    0.U((8 - log2Ceil(t)).W),
    engine.io.result.bits.index(log2Ceil(t) - 1, 0),
    half
  )
  val resultAddress =
    Mux(isO, 2.U * (result.bits.index - (2 * t).U) + half, 256.U + 2.U * result.bits.index + half)
  when(owned && result.valid && isO) {
    assert(result.bits.index < (3 * t).U, "O row index outside the engine range")
    assert(
      io.outputAddress === resultAddress(8, 0),
      "direct O address must match the selected output row"
    )
  }
  when(owned && result.valid) {
    assert(io.outputWrite === isO, "direct write kind must match the selected row")
    when(!isO) {
      assert(result.bits.index < (2 * t).U, "H row outside the state page")
      assert(
        io.stateAddress === resultAddress(8, 0),
        "direct H address must match the selected state row"
      )
    }
  }
  when(owned) {
    assert(
      result.valid === (stateResultValid || outputResultValid),
      "direct producer valid must match return arbitration"
    )
  }
  val stored = RegInit(0.U(16.W))
  val stateWithinRange =
    if (captureStateRowRange) seedTags.io.withinRange
    else stateRow < command.keyColumns
  val needed =
    Mux(isO, ((1 << (t / 8)) - 1).U(16.W), Mux(stateWithinRange, ((1 << (t / 4)) - 1).U(16.W), 0.U))
  val fires = VecInit(io.workWrite.map(_.fire)).asUInt
  result.ready := ((stored | fires) & needed) === needed
  when(result.valid) { stored := stored | fires }
  when(result.fire) { stored := 0.U }
  io.committed.valid := result.fire
  io.committed.bits := Cat(half, result.bits.index)
  for (b <- 0 until 16) {
    val w = io.workWrite(b)
    val bankValid =
      GdnWriteSelection.bankValid(stateResultValid, outputResultValid, stateWithinRange, b, side)
    w.valid := owned && bankValid && !stored(b) && io.bankEnable(b)
    w.bits.address := resultAddress
    val stateValues = if (stateEpilogue) io.scale.response.bits else engine.io.result.bits.values
    val hdata =
      if (b < t / 4) Cat((0 until 4).reverse.map(j => stateValues(b * 4 + j))) else 0.U(128.W)
    val odata =
      if (b < t / 8) Cat((0 until 8).reverse.map(j => narrowed(b * 8 + j))) else 0.U(128.W)
    w.bits.data := Mux(io.outputWrite, odata, hdata)
    w.bits.mask := 65535.U
  }
}
