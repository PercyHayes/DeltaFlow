package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.system._
import deltaflow.memory._
import deltaflow.arithmetic.FloatArithmetic

/** K/Q/V and scalar gates -> Gram, L/J/M, RHS, U/W on ONE 4096-MAC tile. Operand and work each have
  * sixteen 1R1W 128-bit banks. No HBM timing claim. No second array: CS joins and Gram OS requests
  * arbitrate at each panel.
  */
class ReadyTransformTile(
    val side: Int = 16,
    val registerIssue: Boolean = false,
    val withProjection: Boolean = false,
    val jointProjection: Boolean = false,
    val withStateOutput: Boolean = false,
    val returnFlow: Boolean = false,
    val eagerMulticast: Boolean = false,
    val directResidual: Boolean = false,
    val chunkLookahead: Boolean = false,
    val compactOutputs: Boolean = false,
    val streamedStateOutput: Boolean = false,
    val stateEpilogue: Boolean = false,
    val earlyFullGram: Boolean = false,
    val preparationLookahead: Boolean = false,
    val mixedLookahead: Boolean = false,
    val residentProjection: Boolean = false,
    val pipelinedProjection: Boolean = false,
    val columnLookahead: Boolean = false,
    val externalResources: Boolean = false,
    val rollingLookahead: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  val words = (side + 7) / 8
  require(
    !withProjection || registerIssue,
    "projection arbitration requires the registered panel boundary"
  )
  require(!jointProjection || withProjection)
  require(!withStateOutput || jointProjection)
  require(!directResidual || withStateOutput)
  require(!chunkLookahead || (withStateOutput && !directResidual))
  require(!compactOutputs || chunkLookahead)
  require(!streamedStateOutput || compactOutputs)
  require(!stateEpilogue || streamedStateOutput)
  require(!earlyFullGram || stateEpilogue)
  require(!preparationLookahead || earlyFullGram)
  require(!mixedLookahead || preparationLookahead)
  require(!residentProjection || mixedLookahead)
  require(!pipelinedProjection || residentProjection)
  require(!columnLookahead || pipelinedProjection)
  require(!rollingLookahead || (mixedLookahead && !columnLookahead))
  require(
    !externalResources || (registerIssue && withStateOutput && compactOutputs && !directResidual)
  )
  val io = IO(new Bundle {
    val resources = if (externalResources) Some(new TileResources(side)) else None
    val recurrentScale = if (externalResources) Some(new BorrowedScalePorts(side)) else None
    val recurrentEnable = if (externalResources) Some(Input(Bool())) else None
    val start = Flipped(Decoupled(new TransformCommand))
    val gates = Flipped(Decoupled(Vec(4 * side, new GateToken)))
    val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(20.W))
    val transformEnable = Input(Bool())
    val result = Vec(4, Decoupled(new PublishedVector(side)))
    val hostRead = Flipped(Decoupled(UInt(13.W)))
    val hostResponse = Decoupled(UInt(128.W))
    val busy = Output(Bool())
    val done = Output(Bool())
    val coefficientsDone = Output(Bool())
    val gramDone = Output(Bool())
    val gatesReady = Output(Bool())
    val preparationDone = Output(Bool())
    val gramIssue = Output(UInt(16.W))
    val gramJob = Output(Vec(16, UInt(5.W)))
    val dispatchedCS = Output(UInt(16.W))
    val dispatchedOS = Output(UInt(16.W))
    val ingressOccupancy = Output(Vec(16, UInt(2.W)))
    val frozen = Output(UInt(16.W))
    val csIssue = Output(UInt(10.W))
    val csReturned = Output(UInt(10.W))
    val csFed = Output(UInt(10.W))
    val captured = Output(Vec(10, UInt(4.W)))
    val occupied = Output(Vec(10, UInt(7.W)))
    val loaded = Output(UInt(10.W))
    val coefficientLoad = Output(UInt(10.W))
    val coefficient = Output(SInt(11.W))
    val matrix = Output(SInt(11.W))
    val gramReads = Output(UInt(16.W))
    val rhsReads = Output(UInt(16.W))
    val rhsFed = Output(UInt(4.W))
    val gateIssue = Output(Bool())
    val fmaIssue = Output(UInt(4.W))
    val inverseIssue = Output(UInt(4.W))
    val rhsIssue = Output(UInt(4.W))
    val factorIssue = Output(UInt(4.W))
    val lookahead =
      if (chunkLookahead) Some(new Bundle {
        // One future raw-input slot and one pending Gram descriptor. The normal
        // instruction interface starts each chunk; inputSlot/usePrepared are
        // sampled on start. All inputs must be initialized before prepare/start.
        val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
        val prepare = Flipped(Decoupled(UInt(8.W)))
        val rawReady = if (rollingLookahead) Some(Input(Bool())) else None
        val inputSlot = Input(Bool())
        val usePrepared = Input(Bool())
        val overlap = Input(Bool())
        val readPreviousOutput = Input(Bool())
        val cached = Output(Bool())
        val running = Output(Bool())
        val c1Released = Output(Bool())
        val issued = Output(UInt(16.W))
        val reads = Output(UInt(16.W))
        val cacheRows = Output(UInt(4.W))
        val earlyLease = if (earlyFullGram) Some(Output(Bool())) else None
        val claimed = if (earlyFullGram) Some(Output(Bool())) else None
        val gates =
          if (preparationLookahead) Some(Flipped(Decoupled(Vec(4 * side, new GateToken)))) else None
        val preparing = if (preparationLookahead) Some(Output(Bool())) else None
        val coefficientsReady = if (preparationLookahead) Some(Output(Bool())) else None
        val csLease = if (mixedLookahead) Some(Output(UInt(10.W))) else None
        val resultFuture = if (mixedLookahead) Some(Output(Bool())) else None
        val earlyE = if (columnLookahead) Some(Output(Bool())) else None
        val earlyDone = if (columnLookahead) Some(Output(Bool())) else None
        val uwReady = if (mixedLookahead) Some(Output(Bool())) else None
        val zReleased = if (mixedLookahead) Some(Output(UInt(16.W))) else None
        val z0Released = if (residentProjection) Some(Output(UInt(16.W))) else None
      })
      else None
    val compact = if (compactOutputs) Some(new Bundle {
      val writes = Output(UInt(16.W))
      val blocked = Output(UInt(16.W))
    })
    else None
    val output =
      if (withStateOutput) Some(new Bundle {
        val hostRead = Flipped(Decoupled(UInt(13.W)))
        val hostResponse = Decoupled(UInt(128.W))
        val prefixDone = Output(Bool())
        val nDone = Output(Bool())
        val owned = Output(Bool())
        val issued = Output(UInt(16.W))
        val scaleIssued = Output(Bool())
        val committed = Output(Valid(UInt(9.W)))
        val operandReads = Output(UInt(16.W))
        val workReads = Output(UInt(16.W))
      })
      else None
    val projection =
      if (withProjection) Some(new Bundle {
        val initH = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
        val result = Decoupled(new ProjectionRow(side))
        val enable = Input(Bool())
        val done = Output(Bool())
        val eReadsDone = Output(Bool())
        val issued = Output(UInt(16.W))
        val issueKey = Output(UInt(9.W))
        val request = Output(SInt(12.W))
        val captured = Output(SInt(12.W))
        val allocated = Output(Bool())
        val frames = Output(UInt(2.W))
        val seeded = Output(Vec(4, SInt(10.W)))
        val rowRequested = Output(SInt(10.W))
        val rowCaptured = Output(SInt(10.W))
        // Read-only boundary observations for component-cycle calibration.
        val uOffer = Output(Vec(4, SInt(10.W)))
        val uReady = Output(UInt(4.W))
        val seedReady = Output(UInt(16.W))
        val clearReady = Output(UInt(16.W))
        val clearFired = Output(UInt(16.W))
        val readReady = Output(UInt(16.W))
        val responseValid = Output(UInt(16.W))
        val inputReady = Output(UInt(16.W))
        val rowReady = Output(UInt(16.W))
        val rowValid = Output(UInt(16.W))
        val rowFired = Output(UInt(16.W))
        val outputReady = Output(Bool())
      })
      else None
    val joint =
      if (jointProjection) Some(new Bundle {
        val result = Decoupled(new ProjectionWideRow(side))
        val hostRead = Flipped(Decoupled(UInt(13.W)))
        val hostResponse = Decoupled(UInt(128.W))
        val issueZ = Output(UInt(16.W))
        val released = Output(Vec(16, UInt(2.W)))
        val contexts = Output(Vec(16, UInt(2.W)))
        val keys = Output(Vec(16, UInt(8.W)))
        val qRead = Output(UInt(16.W))
        val projectionRead = Output(UInt(16.W))
        val qIssue = Output(UInt(4.W))
        val qWrite = Output(SInt(12.W))
        val qAllocate = Output(SInt(10.W))
        val qCapture = Output(SInt(10.W))
        val clearZ = Output(UInt(16.W))
        val clearContext = Output(UInt(16.W))
        val qReadReady = Output(UInt(16.W))
        val qResponseValid = Output(UInt(16.W))
      })
      else None
  })
  val hostRead = if (withStateOutput) io.output.get.hostRead else io.hostRead
  val hostResponse = if (withStateOutput) io.output.get.hostResponse else io.hostResponse
  if (withStateOutput) {
    io.hostRead.ready := false.B
    io.hostResponse.valid := false.B
    io.hostResponse.bits := 0.U
  }
  // Gram completion checks that every bank reservation/tag has retired.
  // Inactive Gram cannot allocate new words, so its active bit is sufficient
  // for command admission. Keep the existing idle-retirement assertion live.
  val gram = Module(
    new SharedGramController(
      side,
      diagonalFirst = true,
      partitioned = chunkLookahead,
      retiredStart = columnLookahead || (externalResources && stateEpilogue)
    )
  )
  val prep = Module(
    new ReadyGramPreparation(
      side,
      withRhs = true,
      withQuery = jointProjection,
      withBorrowedScale = withStateOutput,
      borrowedFma = stateEpilogue,
      interleavedBorrow = preparationLookahead,
      registeredBorrowCredits = mixedLookahead,
      reorderedRhs = columnLookahead,
      // Return availability is independent of column-lookahead scheduling.
      // A registered exact status cuts pointer decode from external SRAM paths.
      registeredBorrowStatus = columnLookahead || (externalResources && stateEpilogue),
      externalBorrow = externalResources
    )
  )
  prep.io.externalBorrowEnable.foreach(_ := io.recurrentEnable.get)
  val transform = Module(
    new ReadyTransformController(
      side,
      returnFlow = returnFlow,
      eagerMulticast = eagerMulticast,
      registeredDrain = true
    )
  )
  val rhs = Module(
    new TransformRhsReader(
      side = side,
      keyLease = mixedLookahead,
      reordered = columnLookahead,
      independentReads = true
    )
  )
  val preparedU =
    if (mixedLookahead)
      Some(
        Module(
          new TransformRhsReader(
            side = side,
            valuesOnly = true,
            skipValues = columnLookahead,
            boundedValues = columnLookahead,
            independentReads = true
          )
        )
      )
    else None
  val operandMemory = new ResourceHandle(
    if (externalResources) io.resources.get.operand
    else Module(new MacroScratchpad(16, if (withStateOutput) 1024 else 512, 16)).io
  )
  val workMemory = new ResourceHandle(
    if (externalResources) io.resources.get.work
    else
      Module(new MacroScratchpad(16, if (chunkLookahead && !compactOutputs) 1024 else 512, 16)).io
  )
  val operandAccess = Wire(chiselTypeOf(operandMemory.io))
  val operandPorts = Wire(chiselTypeOf(operandMemory.io))
  val workPorts = Wire(chiselTypeOf(workMemory.io))
  operandMemory.io <> operandAccess
  workMemory.io <> workPorts
  for (b <- 0 until 16) {
    operandAccess.read(b).valid := operandPorts.read(b).valid
    operandAccess.read(b).bits := operandPorts.read(b).bits
    operandPorts.read(b).ready := operandAccess.read(b).ready
    operandAccess.write(b).valid := operandPorts.write(b).valid
    operandAccess.write(b).bits := operandPorts.write(b).bits
    operandPorts.write(b).ready := operandAccess.write(b).ready
    operandPorts.response(b).valid := operandAccess.response(b).valid
    operandPorts.response(b).bits := operandAccess.response(b).bits
    operandAccess.response(b).ready := operandPorts.response(b).ready
  }
  val panels = Seq.tabulate(16)(p =>
    new ResourceHandle(
      if (externalResources) io.resources.get.panels(p)
      else
        Module(
          new RetimedWavePanel(
            side,
            rowResponseDepth = if (directResidual) 2 else 1,
            registerInput = registerIssue,
            columnSeed = withProjection,
            columnRead = directResidual,
            osAdmissionStatus = chunkLookahead
          )
        ).io
    )
  )
  val panelPorts = panels.map(p => Wire(chiselTypeOf(p.io)))
  for (p <- 0 until 16) panels(p).io <> panelPorts(p)
  val backend =
    if (withStateOutput)
      Some(
        Module(
          new GdnStateOutputBackend(
            side,
            streamed = streamedStateOutput,
            stateEpilogue = stateEpilogue,
            columnLookahead = columnLookahead,
            registerCompletion = externalResources && stateEpilogue,
            captureStateRowRange = externalResources && stateEpilogue,
            cacheSecondHalf = externalResources && stateEpilogue,
            captureBankDescriptor = externalResources && stateEpilogue,
            qualifiedHReturn = externalResources && stateEpilogue
          )
        )
      )
    else None
  val prefixRetired = RegInit(false.B)
  val outputReady = RegInit(false.B)
  val projection =
    if (withProjection)
      Some(
        Module(
          new StateProjectionController(
            side = side,
            joint = jointProjection,
            directResidual = directResidual,
            residentZ = residentProjection,
            pipelinedReads = pipelinedProjection,
            columnLookahead = columnLookahead,
            localGrantEpoch = externalResources && stateEpilogue && pipelinedProjection
          )
        )
      )
    else None
  val query =
    if (jointProjection)
      Some(
        Module(
          new QueryCoefficientController(
            side = side,
            registerFetchEnd = externalResources && stateEpilogue
          )
        )
      )
    else None
  val gramOwner = if (jointProjection) Some(RegInit(VecInit(Seq.fill(16)(0.U(2.W))))) else None
  val qWritten = if (jointProjection) Some(RegInit(VecInit(Seq.fill(4)(0.U(128.W))))) else None
  val zWritten = if (jointProjection) Some(RegInit(VecInit(Seq.fill(8 * side)(false.B)))) else None
  val zHostPending = if (jointProjection) Some(RegInit(false.B)) else None
  val zHostBank = if (jointProjection) Some(Reg(UInt(4.W))) else None
  val queryPhase = if (jointProjection) Some(RegInit(false.B)) else None
  val arbiters = Seq.fill(10)(Module(new ReadyWaveIssue(side)))
  val active = RegInit(false.B)
  val command = Reg(new TransformCommand)
  val prepFinished = RegInit(false.B)
  val futurePending = if (chunkLookahead) Some(RegInit(false.B)) else None
  val futureRunning = if (chunkLookahead) Some(RegInit(false.B)) else None
  val futureCached = if (chunkLookahead) Some(RegInit(false.B)) else None
  val futureFeatures = if (chunkLookahead) Some(Reg(UInt(8.W))) else None
  val sourceSlot = if (chunkLookahead) Some(RegInit(false.B)) else None
  val outputSlots = if (chunkLookahead) Some(RegInit(0.U(2.W))) else None
  val cachedInput = if (chunkLookahead) Some(RegInit(false.B)) else None
  val cache =
    if (chunkLookahead) Some(Module(new GramCacheReader(side, full = earlyFullGram))) else None
  val earlyLease = if (earlyFullGram) Some(RegInit(false.B)) else None
  val zClaimed = if (earlyFullGram) Some(RegInit(false.B)) else None
  val claimingZ = WireDefault(false.B)
  val futureStart = WireDefault(false.B)
  val futurePrepStart = WireDefault(false.B)
  val futureConfigured = if (preparationLookahead) Some(RegInit(false.B)) else None
  val futurePreparing = if (preparationLookahead) Some(RegInit(false.B)) else None
  val futureMatrix =
    if (preparationLookahead) Some(RegInit(VecInit(Seq.fill(10 * side)(false.B)))) else None
  val currentDecay = if (preparationLookahead) Some(Reg(UInt(32.W))) else None
  val decayValid = if (preparationLookahead) Some(RegInit(false.B)) else None
  val futureRhsStarted = if (mixedLookahead) Some(RegInit(false.B)) else None
  val futureDone = if (mixedLookahead) Some(RegInit(false.B)) else None
  val preparedHandoff = if (mixedLookahead) Some(RegInit(false.B)) else None
  val resultU = if (mixedLookahead) Some(RegInit(VecInit(Seq.fill(4)(true.B)))) else None
  val resultColumn = if (mixedLookahead) Some(Reg(Vec(4, UInt(7.W)))) else None
  def resultIsU(g: Int): Bool =
    resultU.map(_(g)).getOrElse(transform.io.result(g).bits.key < command.valueColumns)
  def resultOffset(g: Int): UInt =
    resultColumn.map(_(g)).getOrElse((transform.io.result(g).bits.key - command.valueColumns)(6, 0))
  def resultKey(g: Int): UInt = if (mixedLookahead)
    Mux(resultU.get(g), resultColumn.get(g), command.valueColumns +& resultColumn.get(g))
  else transform.io.result(g).bits.key
  val preparedLayout = if (mixedLookahead) Some(RegInit(false.B)) else None
  val oldReadsRetired = if (mixedLookahead) Some(RegInit(false.B)) else None
  val zReleased = if (mixedLookahead) Some(RegInit(VecInit(Seq.fill(16)(false.B)))) else None
  val z0Released = if (residentProjection) Some(RegInit(VecInit(Seq.fill(16)(false.B)))) else None
  val backendFinished = if (mixedLookahead) Some(RegInit(false.B)) else None
  val futureRhsStart = WireDefault(false.B)
  val futureResults = futurePreparing.getOrElse(false.B) && (mixedLookahead.B)
  val earlyStarted = if (columnLookahead) Some(RegInit(false.B)) else None
  val earlyDone = if (columnLookahead) Some(RegInit(false.B)) else None
  val earlyMode = if (columnLookahead) Some(RegInit(false.B)) else None
  val futureU0 = if (columnLookahead) Some(RegInit(VecInit(Seq.fill(4)(0.U(8.W))))) else None
  val futureW = if (columnLookahead) Some(RegInit(VecInit(Seq.fill(4)(0.U(128.W))))) else None
  val earlyStart = WireDefault(false.B)
  val earlyProjection = earlyMode.getOrElse(false.B)
  val earlyTake = earlyDone.getOrElse(false.B) && io.lookahead.map(_.usePrepared).getOrElse(false.B)
  val consumePreparation = if (preparationLookahead) io.lookahead.get.usePrepared else false.B
  val hostPending = RegInit(false.B)
  val hostBank = Reg(UInt(4.W))
  val hostInOperand = if (compactOutputs) Some(RegInit(false.B)) else None
  val packedOutput =
    if (compactOutputs) Some(Wire(Vec(16, Decoupled(new BankWrite(1024, 16))))) else None
  packedOutput.foreach(p =>
    for (i <- 0 until 16) {
      p(i).valid := false.B
      p(i).bits := 0.U.asTypeOf(p(i).bits)
    }
  )
  val columns = command.keyColumns +& command.valueColumns
  val bankDrained = RegInit(VecInit(Seq.fill(16)(false.B)))
  val readCount = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
  val outstanding = RegInit(VecInit(Seq.fill(16)(0.U(2.W))))
  val written = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(256)(false.B)))))
  val matrixWritten = RegInit(VecInit(Seq.fill(10 * side)(false.B)))
  val eWritten = if (withProjection) Some(RegInit(VecInit(Seq.fill(8 * side)(false.B)))) else None
  val wWritten = if (withProjection) Some(RegInit(VecInit(Seq.fill(4)(0.U(128.W))))) else None
  val allowStart = !active && !hostPending && !hostRead.valid &&
    !io.init
      .map(_.valid)
      .reduce(_ || _) &&
    io.projection.map(p => !p.initH.map(_.valid).reduce(_ || _)).getOrElse(true.B) &&
    zHostPending.map(p => !p && !io.joint.get.hostRead.valid).getOrElse(true.B) &&
    io.lookahead
      .map(l =>
        !futurePending.get && !futureRunning.get && !l.init.map(_.valid).reduce(_ || _) &&
          (!futureConfigured.getOrElse(
            false.B
          ) && !futurePreparing.getOrElse(false.B) || l.usePrepared) &&
          (!l.usePrepared ||
            ((if (rollingLookahead) l.inputSlot =/= sourceSlot.get
              else
                l.inputSlot) && futureCached.get &&
              io.start.bits.keyColumns === futureFeatures.get &&
              (!mixedLookahead.B || io.start.bits.valueColumns === command.valueColumns)))
      )
      .getOrElse(true.B)
  val legal =
    io.start.bits.keyColumns > 0.U &&
      io.start.bits.keyColumns <=
      (if (withStateOutput) 8 * side
       else
         128).U && io.start.bits.valueColumns > 0.U &&
      io.start.bits.valueColumns <=
      (if (withProjection) 8 * side
       else
         128).U
  val preparationReady = Mux(
    consumePreparation,
    futurePreparing.getOrElse(false.B) && (if (mixedLookahead) preparedHandoff.get else true.B),
    prep.io.start.ready && transform.io.start.ready
  )
  val ready = gram.io.start.ready && preparationReady && rhs.io.idle
  io.start.ready := allowStart && ready && legal
  gram.io.start.valid := io.start.valid && allowStart && legal && prep.io.start.ready &&
    transform.io.start.ready && rhs.io.idle
  gram.io.start.bits := io.start.bits.keyColumns
  if (chunkLookahead) {
    val l = io.lookahead.get
    gram.io.partition.get := Mux(
      futureStart,
      (if (earlyFullGram) 0 else 1).U,
      Mux(l.usePrepared, 2.U, 0.U)
    )
    if (earlyFullGram) when(l.usePrepared) { gram.io.start.valid := false.B }
    when(futureStart) {
      gram.io.start.valid := true.B
      gram.io.start.bits := futureFeatures.get
    }
    l.prepare.ready :=
      (active || rollingLookahead.B) &&
        (rollingLookahead.B || !sourceSlot.get) && !futurePending.get && !futureRunning.get &&
        !futureCached.get && l.prepare.bits > 0.U &&
        l.prepare.bits <=
        (8 * side).U &&
        (!columnLookahead.B || l.prepare.bits === command.keyColumns)
    when(l.prepare.fire) {
      futurePending.get := true.B
      futureFeatures.get := l.prepare.bits
    }
    if (earlyFullGram) {
      when(l.prepare.fire) { earlyLease.get := l.overlap && !zClaimed.get && !claimingZ }
      futureStart := futurePending.get && gram.io.start.ready && (earlyLease.get || !active) &&
        l.rawReady
          .getOrElse(true.B)
      l.earlyLease.get := earlyLease.get
      l.claimed.get := zClaimed.get
      when(futureRunning.get && gram.io.done) { earlyLease.get := false.B }
    } else
      futureStart := futurePending.get && gram.io.start.ready && prefixRetired &&
        ((!active) || (l.overlap && backend.get.io.c1Released))
    when(futureStart) {
      futurePending.get := false.B
      futureRunning.get := true.B
    }
    when(futureRunning.get && gram.io.done) {
      futureRunning.get := false.B
      futureCached.get := true.B
    }
    cache.get.io.start :=
      (if (preparationLookahead) futurePrepStart
       else io.start.fire && l.usePrepared)
    cache.get.io.bankEnable := io.bankEnable
    l.cached := futureCached.get
    l.running := futureRunning.get
    l.c1Released := backend.get.io.c1Released
    l.issued := Mux(futureRunning.get, VecInit(gram.io.issued.map(_.valid)).asUInt, 0.U)
    l.reads := Mux(futureRunning.get, gram.io.allocated, 0.U)
    l.cacheRows := VecInit(cache.get.io.result.map(_.fire)).asUInt
  }
  prep.io.start.valid := io.start.valid && allowStart && legal && gram.io.start.ready &&
    transform.io.start.ready && rhs.io.idle && !consumePreparation
  prep.io.start.bits := true.B
  transform.io.start.valid := io.start.valid && allowStart && legal && gram.io.start.ready &&
    prep.io.start.ready && rhs.io.idle && !consumePreparation
  transform.io.start.bits :=
    (if (directResidual) io.start.bits.valueColumns
     else io.start.bits.keyColumns +& io.start.bits.valueColumns)
  rhs.io.start.valid := io.start.fire && !(mixedLookahead.B && consumePreparation)
  rhs.io.start.bits := io.start.bits
  if (mixedLookahead) {
    // Register the U/W role and canonical column separately from CS tickets.
    // The optional future order is U0, W, U1; the current order is U, W.
    // Register the decode,
    // avoiding tag-base subtraction followed by role/address decode at SRAM.
    // The transform still uses monotonic tickets; external results name logical columns.
    for (g <- 0 until 4) {
      when(io.start.fire || futurePrepStart) {
        resultU.get(g) := true.B
        resultColumn.get(g) := 0.U
      }
        .elsewhen(io.result(g).fire) {
          val ticket =
            if (columnLookahead)
              Mux(
                futureResults,
                Mux(
                  resultU.get(g),
                  Mux(
                    resultColumn.get(g) < (4 * side).U,
                    resultColumn.get(g),
                    command.keyColumns +& resultColumn.get(g)
                  ),
                  Mux(
                    command.valueColumns > (4 * side).U,
                    (4 * side).U,
                    command.valueColumns
                  ) +& resultColumn.get(g)
                ),
                resultKey(g)
              )
            else resultKey(g)
          assert(
            transform.io.result(g).bits.key === ticket,
            "completed CS ticket disagrees with U/W role and column"
          )
          when(columnLookahead.B && futureResults) {
            when(
              resultU.get(g) && resultColumn.get(g) === Mux(
                command.valueColumns > (4 * side).U,
                (4 * side).U,
                command.valueColumns
              ) - 1.U
            ) {
              resultU.get(g) := false.B
              resultColumn.get(g) := 0.U
            }
              .elsewhen(!resultU.get(g) && resultColumn.get(g) === command.keyColumns - 1.U) {
                resultU.get(g) := true.B
                resultColumn.get(g) := (4 * side).U
              }
              .otherwise { resultColumn.get(g) := resultColumn.get(g) + 1.U }
          }.elsewhen(resultU.get(g) && resultColumn.get(g) === command.valueColumns - 1.U) {
            resultU.get(g) := false.B
            resultColumn.get(g) := 0.U
          }.otherwise { resultColumn.get(g) := resultColumn.get(g) + 1.U }
        }
    }
    // Register the completed version before broadcasting the next command.
    // Publisher empty/CS-idle reductions must not feed start.fire's wide fanout.
    // Once latched, no producer restarts until that version is consumed.
    when(
      futurePreparing.get && futureDone.get && transform.io.idle && rhs.io.idle &&
        panels
          .map(_.io.csIdle)
          .reduce(_ && _)
    ) {
      preparedHandoff.get := true.B
    }
    when(io.start.fire && consumePreparation) {
      assert(
        preparedHandoff.get && futureDone.get && transform.io.idle && rhs.io.idle &&
          panels
            .map(_.io.csIdle)
            .reduce(_ && _)
      )
    }
    // Reuse each low-bank future-reader client after its last cached Gram
    // response retires. U may proceed on free high Work pages. W raw admission
    // waits for the old W readers, preventing a CS/OS circular dependency.
    when(projection.get.io.eReadsDone) { oldReadsRetired.get := true.B }
    futureRhsStart := futurePrepStart
    when(futureRhsStart) {
      assert(rhs.io.idle)
      rhs.io.start.valid := true.B
      rhs.io.start.bits.keyColumns := futureFeatures.get
      rhs.io.start.bits.valueColumns := command.valueColumns
      futureRhsStarted.get := true.B
    }
    rhs.io.allowKeys.get := !futureRhsStarted.get || oldReadsRetired.get
    val u = preparedU.get
    if (columnLookahead) {
      u.io.skipFirst.get := io.start.fire && earlyTake
      for (g <- 0 until 4)
        u.io.valueReady.get(g) := Mux(earlyProjection, futureU0.get(g), command.valueColumns)
    }
    u.io.start.valid := (io.start.fire && consumePreparation) || earlyStart
    u.io.start.bits := io.start.bits
    when(earlyStart) {
      u.io.start.bits.keyColumns := command.keyColumns
      u.io.start.bits.valueColumns := Mux(
        command.valueColumns > (4 * side).U,
        (4 * side).U,
        command.valueColumns
      )
    }
    u.io.available := 65535.U
    u.io.bankEnable := Cat(0.U(8.W), io.bankEnable(15, 8))
    io.lookahead.get.zReleased.get := zReleased.get.asUInt
    if (residentProjection) io.lookahead.get.z0Released.get := z0Released.get.asUInt
    io.lookahead.get.csLease.get := Mux(futureRhsStarted.get, transform.io.loaded, 0.U)
    io.lookahead.get.resultFuture.get := futureResults
    io.lookahead.get.uwReady.get := futurePreparing.get && futureDone.get && transform.io.idle
  }
  if (columnLookahead) {
    earlyStart := active && !sourceSlot.get && futureRhsStarted.get && !earlyStarted.get &&
      backend.get.io.c1Released && projection.get.io.idle && command.valueColumns > (4 * side).U
    when(earlyStart) {
      assert(projection.get.io.start.ready)
      earlyStarted.get := true.B
      earlyMode.get := true.B
      assert(preparedU.get.io.idle)
    }
    when(earlyProjection && projection.get.io.done) { earlyDone.get := true.B }
    when(io.start.fire) {
      earlyMode.get := false.B
      earlyStarted.get := false.B
      earlyDone.get := false.B
    }
    when(futurePrepStart) {
      for (g <- 0 until 4) {
        futureU0.get(g) := 0.U
        futureW.get(g) := 0.U
      }
    }
    for (g <- 0 until 4) when(futureResults && io.result(g).fire) {
      when(resultIsU(g) && resultColumn.get(g) < (4 * side).U) {
        assert(resultColumn.get(g) === futureU0.get(g), "U0 must publish in canonical column order")
        futureU0.get(g) := futureU0.get(g) + 1.U
      }
      when(!resultIsU(g)) { futureW.get(g) := futureW.get(g) | (1.U(128.W) << resultColumn.get(g)) }
    }
    io.lookahead.get.earlyE.get := earlyProjection
    io.lookahead.get.earlyDone.get := earlyDone.get
  }
  prep.io.gates.valid := io.gates.valid && !active
  prep.io.gates.bits := io.gates.bits
  io.gates.ready := prep.io.gates.ready && !active
  if (preparationLookahead) {
    val l = io.lookahead.get
    val mayConfigure =
      futureCached.get && prepFinished && backend.get.io.nDone && transform.io.idle &&
        query.get.io.idle && !futureConfigured.get && !futurePreparing.get &&
        (if (rollingLookahead)
           preparedU.get.io.idle
         else true.B)
    prep.io.gates.valid := Mux(
      mayConfigure,
      l.gates.get.valid,
      io.gates.valid && !active && !futureConfigured.get && !futurePreparing.get
    )
    prep.io.gates.bits := Mux(mayConfigure, l.gates.get.bits, io.gates.bits)
    io.gates.ready := prep.io.gates.ready && !active && !mayConfigure && !futureConfigured.get &&
      !futurePreparing.get
    l.gates.get.ready := prep.io.gates.ready && mayConfigure
    when(l.gates.get.fire) {
      assert(decayValid.get && backend.get.io.nDone)
      futureConfigured.get := true.B
    }
    // Gate acceptance already proved the old transform fully retired. No
    // transform producer may restart between that acceptance and this start.
    // Keep the wide publisher-empty tree out of the future command fanout.
    if (mixedLookahead) when(futureConfigured.get && !futurePreparing.get) {
      assert(transform.io.start.ready)
    }
    futurePrepStart := futureConfigured.get && !futurePreparing.get && prep.io.start.ready &&
      (if (mixedLookahead) true.B
       else
         transform.io.start.ready)
    when(futurePrepStart) {
      assert(prepFinished && backend.get.io.nDone && rhs.io.idle && query.get.io.idle)
      prep.io.start.valid := true.B
      transform.io.start.valid := true.B
      transform.io.start.bits := futureFeatures.get +& command.valueColumns
      futurePreparing.get := true.B
      for (r <- 0 until 10 * side) futureMatrix.get(r) := false.B
    }
    l.preparing.get := futurePreparing.get
    l.coefficientsReady.get :=
      futurePreparing.get && prep.io.rhs.get.coefficientsDone && transform.io.loaded.andR
    if (!mixedLookahead) when(futurePreparing.get) {
      assert(
        !io.result.map(_.valid).reduce(_ || _) && transform.io.reserved === 0.U,
        "future RHS must wait for state command"
      )
    }
  }
  when(io.start.fire) {
    if (mixedLookahead) {
      for (p <- 0 until 16) zReleased.get(p) := false.B
      backendFinished.get := false.B
      preparedHandoff.get := false.B
      preparedLayout.get := consumePreparation
      futureRhsStarted.get := false.B
      futureDone.get := false.B
      oldReadsRetired.get := false.B
    }
    active := true.B
    command := io.start.bits
    prepFinished := false.B
    prefixRetired := false.B
    outputReady := false.B
    for (b <- 0 until 16) {
      readCount(b) := 0.U
      outstanding(b) := 0.U
      bankDrained(b) := (b >= 8 * words).B
    }
    if (earlyFullGram) {
      earlyLease.get := false.B
      zClaimed.get := false.B
      when(io.lookahead.get.usePrepared) { for (b <- 0 until 16) bankDrained(b) := true.B }
    }
    for (g <- 0 until 4; c <- 0 until 256) written(g)(c) := false.B
    for (r <- 0 until 10 * side) matrixWritten(r) := false.B
    if (preparationLookahead) {
      futureConfigured.get := false.B
      futurePreparing.get := false.B
      decayValid.get := false.B
      when(consumePreparation) {
        for (r <- 0 until 10 * side) matrixWritten(r) := futureMatrix.get(r)
        if (mixedLookahead) {
          prepFinished := true.B
          currentDecay.get := prep.io.prefix(4 * side - 1)
          decayValid.get := true.B
        }
      }
      if (mixedLookahead) when(!consumePreparation) { futureDone.get := false.B }
    }
    eWritten.foreach(e => for (r <- 0 until 8 * side) e(r) := false.B)
    wWritten.foreach(w =>
      for (g <- 0 until 4) {
        w(g) := 0.U
        if (mixedLookahead) when(consumePreparation) {
          w(g) := VecInit((0 until 128).map(k => k.U < io.start.bits.keyColumns)).asUInt
        }
      }
    )
    gramOwner.foreach(o =>
      for (p <- 0 until 16)
        o(p) := Mux(
          io.lookahead.map(_.usePrepared).getOrElse(false.B),
          (if (p < 4) 2 else 0).U,
          (if (p < 4) 3 else 1).U
        )
    )
    if (earlyFullGram) when(io.lookahead.get.usePrepared) {
      for (p <- 0 until 16) gramOwner.get(p) := 0.U
    }
    qWritten.foreach(q => for (g <- 0 until 4) q(g) := 0.U)
    zWritten.foreach(z => for (r <- 0 until 8 * side) z(r) := false.B)
    queryPhase.foreach(_ := false.B)
    if (chunkLookahead) {
      sourceSlot.get := io.lookahead.get.inputSlot
      cachedInput.get := io.lookahead.get.usePrepared
      when(io.lookahead.get.usePrepared) { futureCached.get := false.B }
      when(!io.lookahead.get.inputSlot) { outputSlots.get := 0.U }
    }
  }
  if (chunkLookahead) when(futureStart) {
    for (p <- 0 until 16) gramOwner.get(p) := (if (earlyFullGram && p < 4) 3 else 1).U
  }
  if (jointProjection) {
    val q = query.get
    val p = projection.get.io.joint.get
    val o = io.joint.get
    q.io.start.valid := io.start.fire
    q.io.start.bits := io.start.bits.keyColumns
    q.io.available := bankDrained.asUInt
    q.io.bankEnable := io.bankEnable
    prep.io.query.get.raw <> q.io.scale
    q.io.scaled <> prep.io.query.get.result
    when(active) { queryPhase.get := !queryPhase.get }
    o.result <> p.result
    q.io.write.ready := active && !o.result.fire
    when(q.io.write.fire) {
      qWritten.get(q.io.write.bits.group) :=
        qWritten
          .get(q.io.write.bits.group) | (1.U(128.W) << q.io.write.bits.column(6, 0))
    }
    for (g <- 0 until 4) p.written(g) := qWritten.get(g)
    for (t <- 0 until 16) {
      val held =
        earlyLease.map(x => Mux(x, (if (t < 4) 3 else 1).U(2.W), 0.U(2.W))).getOrElse(0.U(2.W))
      p.released(t) := ~gramOwner.get(t) & ~held
      o.released(t) := p.released(t)
    }
    if (earlyFullGram) {
      claimingZ := (0 until 16)
        .map(t => p.clearZ(t) && (!p.clearContext(t) || (t < 4).B))
        .reduce(_ || _)
      when(claimingZ) { zClaimed.get := true.B }
      when(earlyLease.get) { assert(!claimingZ, "Z cannot take a leased future Gram context") }
      when(futureStart) { assert(gramOwner.get.map(_ === 0.U).reduce(_ && _)) }
    }
    o.issueZ := p.issueZ
    o.contexts := p.contexts
    o.keys := p.keys
    o.clearZ := p.clearZ
    o.clearContext := p.clearContext
    o.qIssue := prep.io.query.get.issued
    o.qAllocate := q.io.allocated
    o.qCapture := q.io.captured
    o.qRead := VecInit(q.io.read.map(_.fire)).asUInt
    o.projectionRead := VecInit(p.read.map(_.fire)).asUInt
    o.qReadReady := VecInit(p.read.map(_.ready)).asUInt
    o.qResponseValid := VecInit(p.response.map(_.valid)).asUInt
    o.qWrite := Mux(
      q.io.write.fire,
      (q.io.write.bits.column * 4.U + q.io.write.bits.group).zext,
      (-1).S
    )
    when(o.result.fire) {
      zWritten.get(
        (((o.result.bits.group * side.U + o.result.bits.row) << 1) + o.result.bits.half.asUInt)(
          log2Ceil(8 * side) - 1,
          0
        )
      ) := true.B
    }
    val address = o.hostRead.bits(12, 4)
    val bank = o.hostRead.bits(3, 0)
    val zLegal = bank < side.U && address >= 384.U && address < (384 + 8 * side).U && zWritten.get(
      (address - 384.U)(log2Ceil(8 * side) - 1, 0)
    )
    val qLegal =
      bank >= (4 * words).U && bank < (8 * words).U && address >= 256.U && address < 384.U &&
        qWritten.get(((bank - (4 * words).U) / words.U)(1, 0))((address - 256.U)(6, 0))
    val legal = zLegal || qLegal
    o.hostRead.ready := !active && !io.start.valid && !zHostPending.get && legal &&
      operandPorts
        .read(bank)
        .ready
    when(o.hostRead.fire) {
      zHostPending.get := true.B
      zHostBank.get := bank
    }
    o.hostResponse.valid := zHostPending.get && operandPorts.response(zHostBank.get).valid
    o.hostResponse.bits := operandPorts.response(zHostBank.get).bits
    when(o.hostResponse.fire) { zHostPending.get := false.B }
  }
  projection.foreach { p =>
    val o = io.projection.get
    p.io.start.valid := io.start.fire || earlyStart
    p.io.start.bits := io.start.bits
    if (columnLookahead) {
      p.io.earlyE.get := earlyStart
      p.io.skipE0.get := io.start.fire && earlyTake
      when(earlyStart) {
        p.io.start.bits.keyColumns := command.keyColumns
        p.io.start.bits.valueColumns := (4 * side).U
      }
    }
    when(io.start.fire) { assert(p.io.start.ready) }
    p.io.bankEnable := io.bankEnable
    p.io.enable := o.enable
    for (g <- 0 until 4)
      p.io.written(
        g
      ) :=
        (if (columnLookahead)
           Mux(earlyProjection, futureW.get(g) & backend.get.io.h0Ready.get, wWritten.get(g))
         else wWritten.get(g))
    val writesReady =
      if (columnLookahead)
        (0 until side / 2)
          .map(i =>
            io.bankEnable(i) && (!backend.get.io.workOwner || !backend.get.io.workWrite(i).valid)
          )
          .reduce(_ && _)
      else true.B
    o.result.bits := p.io.result.bits
    o.result.valid := p.io.result.valid && matrixWritten.asUInt.andR && writesReady
    p.io.result.ready := o.result.ready && matrixWritten.asUInt.andR && writesReady
    o.done := p.io.done
    o.eReadsDone := p.io.eReadsDone
    o.issued := p.io.issued
    o.issueKey := p.io.issueKey
    o.request := p.io.request
    o.captured := p.io.captured
    o.allocated := p.io.allocated
    o.frames := p.io.frames
    o.seeded := p.io.seeded
    o.rowRequested := p.io.rowRequested
    o.rowCaptured := p.io.rowCaptured
    o.uOffer := VecInit(p.io.u.map(u => Mux(u.valid, u.bits.column.zext, (-1).S)))
    o.uReady := VecInit(p.io.u.map(_.ready)).asUInt
    o.seedReady := VecInit(p.io.seed.map(_.ready)).asUInt
    o.clearReady := VecInit(p.io.clear.map(_.ready)).asUInt
    o.clearFired := VecInit(p.io.clear.map(_.fire)).asUInt
    o.readReady := VecInit(p.io.read.map(_.ready)).asUInt
    o.responseValid := VecInit(p.io.response.map(_.valid)).asUInt
    o.inputReady := VecInit(p.io.input.map(_.ready)).asUInt
    o.rowReady := VecInit(p.io.rowRead.map(_.ready)).asUInt
    o.rowValid := VecInit(p.io.rowResponse.map(_.valid)).asUInt
    o.rowFired := VecInit(p.io.rowRead.map(_.fire)).asUInt
    o.outputReady := p.io.result.ready
    if (mixedLookahead)
      when(
        o.result.fire && !earlyProjection &&
          (if (residentProjection) o.result.bits.half === (command.valueColumns > (4 * side).U)
           else command.valueColumns > (4 * side).U && o.result.bits.half) &&
          o.result.bits.group === 3.U && o.result.bits.row === (side - 1).U
      ) {
        // Both even contexts have been extracted. Snapshot mode has already
        // published Z0; resident mode instead transfers each odd context only
        // after its arithmetic retirement. Neither waits for a whole Z1 spill.
        assert(p.io.eReadsDone)
        prefixRetired := true.B
      }
    when(o.result.fire) {
      eWritten.get(
        (((o.result.bits.group * side.U + o.result.bits.row) << 1) + o.result.bits.half.asUInt)(
          log2Ceil(8 * side) - 1,
          0
        )
      ) := true.B
    }
  }
  prep.io.gram <> gram.io.result
  transform.io.coefficient <> prep.io.result
  if (chunkLookahead) for (g <- 0 until 4) {
    val arb = Module(new RRArbiter(new GramRow(side), 2))
    arb.io.in(0).valid := gram.io.result(g).valid && !futureRunning.get
    arb.io.in(0).bits := gram.io.result(g).bits
    arb.io.in(1) <> cache.get.io.result(g)
    prep.io.gram(g) <> arb.io.out
    gram.io.result(g).ready := arb.io.in(0).ready && !futureRunning.get
  }
  prep.io.rhs.get.columns := columns
  prep.io.rhs.get.valueColumns := command.valueColumns
  prep.io.rhs.get.raw <> rhs.io.result
  if (columnLookahead) {
    rhs.io.halfFirst.get := futureRhsStart
    prep.io.rhs.get.halfFirst.get := futureResults
  }
  val residualBk = if (directResidual) Some(Wire(Vec(4, Vec(side, UInt(16.W))))) else None
  if (directResidual) {
    transform.io.rhs <> projection.get.io.residual.get.result
    for (g <- 0 until 4) {
      val raw = prep.io.rhs.get.result(g)
      val u = projection.get.io.u(g)
      val isValue = raw.bits.key < command.valueColumns
      u.valid := raw.valid && isValue
      u.bits.column := raw.bits.key
      u.bits.values := raw.bits.values
      raw.ready := active && (!isValue || u.ready)
      for (r <- 0 until side) residualBk.get(g)(r) := FloatArithmetic.narrow(raw.bits.values(r))
      when(raw.fire && !isValue) {
        wWritten
          .get(g) := wWritten.get(g) | (1.U(128.W) << ((raw.bits.key - command.valueColumns)(6, 0)))
      }
    }
  } else { transform.io.rhs <> prep.io.rhs.get.result }
  transform.io.enable := io.transformEnable
  gram.io.bankEnable := io.bankEnable
  gram.io.jobEnable := io.jobEnable
  rhs.io.available := Mux(
    futureRhsStarted.getOrElse(false.B),
    cache.map(_.io.released).getOrElse(0.U),
    bankDrained.asUInt
  )
  rhs.io.bankEnable := io.bankEnable
  for (b <- 0 until 16) {
    operandPorts.write(b).valid := io.init(b).valid && !active && !io.start.valid &&
      zHostPending
        .map(p => !p)
        .getOrElse(true.B)
    operandPorts.write(b).bits := io.init(b).bits
    io.init(b).ready := operandPorts.write(b).ready && !active && !io.start.valid &&
      zHostPending
        .map(p => !p)
        .getOrElse(true.B)
    when(io.init(b).fire) { assert(io.init(b).bits.address < 256.U) }
    val owned = bankDrained(b)
    operandPorts.read(b).valid := Mux(owned, rhs.io.read(b).valid, gram.io.bankRead(b).valid)
    operandPorts.read(b).bits := Mux(owned, rhs.io.read(b).bits, gram.io.bankRead(b).bits)
    gram.io.bankRead(b).ready := operandPorts.read(b).ready && !owned
    rhs.io.read(b).ready := operandPorts.read(b).ready && owned
    gram.io.bankResponse(b).valid := operandPorts.response(b).valid && !owned
    gram.io.bankResponse(b).bits := operandPorts.response(b).bits
    rhs.io.response(b).valid := operandPorts.response(b).valid && owned
    rhs.io.response(b).bits := operandPorts.response(b).bits
    operandPorts.response(b).ready := Mux(
      owned,
      rhs.io.response(b).ready,
      gram.io.bankResponse(b).ready
    )
    if (jointProjection) {
      val q = query.get
      val p = projection.get.io.joint.get
      val o = io.joint.get
      q.io.read(b).ready := false.B
      q.io.response(b).valid := false.B
      q.io.response(b).bits := operandPorts.response(b).bits
      p.read(b).ready := false.B
      p.response(b).valid := false.B
      p.response(b).bits := operandPorts.response(b).bits
      if (b >= 4 * words && b < 8 * words) {
        // Once Qbar production has fully drained, its read slots belong to
        // the projection. idle is registered, so no consumer-ready path feeds
        // back into arbitration or a same-cycle returned credit.
        val owners = Module(new Queue(Bool(), 2, pipe = false, flow = false))
        val phase = queryPhase.get || q.io.idle
        when(owned) {
          operandPorts.read(b).valid := active && owners.io.enq.ready && Mux(
            phase,
            p.read(b).valid,
            q.io.read(b).valid
          )
          operandPorts.read(b).bits := Mux(phase, p.read(b).bits, q.io.read(b).bits)
          operandPorts.response(b).ready := owners.io.deq.valid && Mux(
            owners.io.deq.bits,
            p.response(b).ready,
            q.io.response(b).ready
          )
        }
        q.io
          .read(b)
          .ready := active && owned && !phase && owners.io.enq.ready && operandPorts.read(b).ready
        p.read(b).ready :=
          active && owned && phase && owners.io.enq.ready && operandPorts.read(b).ready
        q.io
          .response(b)
          .valid := active && owned && owners.io.deq.valid && !owners.io.deq.bits &&
          operandPorts
            .response(b)
            .valid
        p.response(b).valid := active && owned && owners.io.deq.valid && owners.io.deq.bits &&
          operandPorts
            .response(b)
            .valid
        owners.io.enq.valid := active && owned && operandPorts.read(b).fire
        owners.io.enq.bits := phase
        owners.io.deq.ready := active && owned && operandPorts.response(b).fire
        rhs.io.read(b).ready := false.B
        rhs.io.response(b).valid := false.B
        when(q.io.write.fire && q.io.write.bits.group === ((b - 4 * words) / words).U) {
          operandPorts.write(b).valid := true.B
          operandPorts.write(b).bits.address := 256.U + q.io.write.bits.column
          operandPorts.write(b).bits.mask := ((BigInt(1) << math.min(16, side * 2)) - 1).U
          operandPorts.write(b).bits.data := Cat(
            (0 until 8).reverse.map(i =>
              if ((b % words) * 8 + i < side) q.io.write.bits.values((b % words) * 8 + i)
              else 0.U(16.W)
            )
          )
        }
      }
      if (b < side) when(o.result.fire) {
        operandPorts.write(b).valid := true.B
        operandPorts
          .write(b)
          .bits
          .address := 384.U + ((o.result.bits.group * side.U + o.result.bits.row) << 1) +
          o.result.bits.half.asUInt
        operandPorts.write(b).bits.mask := 65535.U
        operandPorts.write(b).bits.data := Cat(
          (0 until 4).reverse.map(i => o.result.bits.values(4 * b + i))
        )
      }
      when(!active) {
        val address = o.hostRead.bits(12, 4)
        val zLegal = (b < side).B && address >= 384.U && address < (384 + 8 * side).U &&
          zWritten.get(
            (address - 384.U)(log2Ceil(8 * side) - 1, 0)
          )
        val qLegal =
          if (b >= 4 * words && b < 8 * words)
            address >= 256.U && address < 384.U && qWritten.get((b - 4 * words) / words)(
              (address - 256.U)(6, 0)
            )
          else false.B
        operandPorts
          .read(b)
          .valid := o.hostRead.valid && !io.start.valid && !zHostPending.get && o.hostRead.bits(
          3,
          0
        ) === b.U && (zLegal || qLegal)
        operandPorts.read(b).bits := o.hostRead.bits(12, 4)
        operandPorts
          .response(b)
          .ready := zHostPending.get && zHostBank.get === b.U && o.hostResponse.ready
      }
      when(operandPorts.write(b).valid) { assert(operandPorts.write(b).ready) }
    }
    val launch = gram.io.bankRead(b).fire
    val retire = gram.io.bankResponse(b).fire
    when(launch) { readCount(b) := readCount(b) + 1.U }
    when(launch =/= retire) {
      outstanding(b) := Mux(launch, outstanding(b) + 1.U, outstanding(b) - 1.U)
    }
    when(active && !owned && readCount(b) === command.keyColumns && outstanding(b) === 0.U) {
      bankDrained(b) := true.B
    }
  }
  for (t <- 0 until 16) {
    val panel = panelPorts(t)
    panel.clear <> gram.io.clear(t)
    panel.read <> gram.io.read(t)
    gram.io.response(t) <> panel.response
    if (externalResources && jointProjection && residentProjection) {
      // Gram uses this status only to choose a row candidate. Registered
      // occupancy includes queued OS, the entire arithmetic wave and clears.
      // A same-cycle offered token must not select a different read context
      // through this hint. The physical panel.read.ready retains its complete
      // same-cycle exclusion check, so no early candidate bypasses ownership.
      gram.io.contextIdle(t) := panel.residentContextIdle.get
      when(gram.io.read(t).fire) {
        assert(
          panel.contextIdle(gram.io.read(t).bits.context),
          "Gram snapshot requires the original physical read permission"
        )
      }
    } else {
      gram.io.contextIdle(t) := panel.contextIdle
    }
    panel.seed.valid := false.B
    panel.seed.bits := 0.U.asTypeOf(panel.seed.bits)
    val os = Wire(Decoupled(new RetimedPanelInput(side, 2)))
    if (withProjection) {
      val p = projection.get
      panel.seed <> p.io.seed(t)
      panel.seedColumn.get := true.B
      // Odd contexts remain Gram-owned; response tags route the independent
      // even-context extraction without relying on a global Gram-done barrier.
      val ca = Module(new RRArbiter(UInt(2.W), 2))
      ca.io.in(0) <> gram.io.clear(t)
      ca.io.in(1) <> p.io.clear(t)
      panel.clear <> ca.io.out
      val ra = Module(new RRArbiter(new RetimedRow(side), 2))
      ra.io.in(0) <> gram.io.read(t)
      ra.io.in(1) <> p.io.rowRead(t)
      panel.read <> ra.io.out
      panel.readColumn.foreach(_ := ra.io.chosen === 1.U && p.io.residual.get.readColumn(t))
      val gramResponse = panel.response.bits
        .context(0) && gramOwner.map(_(t)(panel.response.bits.context(1))).getOrElse(true.B)
      gram.io.response(t).valid := panel.response.valid && gramResponse
      p.io.rowResponse(t).valid := panel.response.valid && !gramResponse
      p.io.rowResponse(t).bits := panel.response.bits
      if (mixedLookahead && !residentProjection)
        when(
          p.io.rowResponse(t).fire && p.io.rowResponse(t).bits.context === 3.U &&
            p.io
              .rowResponse(t)
              .bits
              .row === (side - 1).U
        ) {
          zReleased.get(t) := true.B
        }
      panel.response.ready := Mux(
        gramResponse,
        gram.io.response(t).ready,
        p.io.rowResponse(t).ready
      )
      if (jointProjection)
        when(gram.io.response(t).fire && gram.io.response(t).bits.row === (side - 1).U) {
          val h = gram.io.response(t).bits.context(1)
          assert(gramOwner.get(t)(h))
          gramOwner.get(t) := gramOwner.get(t) & ~(1.U(2.W) << h)
        }
      val oa = Module(new RRArbiter(new RetimedPanelInput(side, 2), 2))
      oa.io.in(0) <> gram.io.in(t)
      oa.io.in(1) <> p.io.input(t)
      os <> oa.io.out
    } else { os <> gram.io.in(t) }
    if (t < 10) {
      arbiters(t).io.os <> os
      arbiters(t).io.cs <> transform.io.input(t)
      arbiters(t).io.allowCS := true.B
      arbiters(t).io.advance := panel.advance
      panel.in <> arbiters(t).io.out
      panel.coefficient <> transform.io.load(t)
      transform.io.returned(t) <> panel.out
    } else {
      panel.in <> os
      panel.coefficient.valid := false.B
      panel.coefficient.bits := 0.U.asTypeOf(panel.coefficient.bits)
      panel.out.ready := true.B
      assert(!panel.out.valid)
    }
  }
  val hostWord = hostRead.bits
  val hostAddress = hostWord(12, 4)
  val hostInputBank = hostWord(3, 0)
  val resultBank = hostInputBank < (4 * words).U
  val resultGroup = (hostInputBank / words.U)(1, 0)
  val matrixBank = hostInputBank >= (if (withProjection) 8 else 16 - side / 4).U
  val matrixRow =
    if (withProjection) hostAddress * (32 / side).U + ((hostInputBank - 8.U) / (side / 4).U)
    else hostAddress - 256.U
  val mLegal = matrixBank &&
    (if (withProjection) hostAddress < (10 * side * side / 32).U
     else hostAddress >= 256.U) && matrixRow < (10 * side).U &&
    matrixWritten(matrixRow(log2Ceil(10 * side) - 1, 0))
  val eLegal =
    if (withProjection)
      hostInputBank >= 8.U && hostInputBank < (8 + side / 2).U && hostAddress >= 128.U &&
      hostAddress < (128 + 8 * side).U && eWritten.get(
        (hostAddress - 128.U)(log2Ceil(8 * side) - 1, 0)
      )
    else false.B
  val hostLegal =
    if (withStateOutput)
      outputReady &&
      (!hostAddress(
        0
      ) || command.valueColumns > (4 * side).U) &&
      ((hostAddress >= 256.U && ((hostAddress - 256.U) >> 1) < command.keyColumns &&
        hostInputBank < side.U) ||
        (hostAddress < (8 * side).U && hostInputBank < (side / 2).U))
    else
      Mux(
        resultBank,
        hostAddress < 256.U && written(resultGroup)(hostAddress(7, 0)),
        mLegal || eLegal
      )
  hostRead.ready := !active && !io.start.valid && !hostPending && hostLegal &&
    workPorts
      .read(hostInputBank)
      .ready
  when(hostRead.fire) {
    hostPending := true.B
    hostBank := hostInputBank
  }
  hostResponse.valid := hostPending && workPorts.response(hostBank).valid
  hostResponse.bits := workPorts.response(hostBank).bits
  when(hostResponse.fire) { hostPending := false.B }
  transform.io.matrix.ready := active
  if (preparationLookahead)
    transform.io.matrix.ready := (active || futurePreparing.get) &&
      !(io.start.fire && consumePreparation) && (8 until 8 + side / 2)
        .map(b =>
          io.bankEnable(b) &&
            (!backend.get.io.workOwner ||
              (futurePreparing.get &&
                !backend.get.io
                  .workWrite(b)
                  .valid))
        )
        .reduce(_ && _)
  for (g <- 0 until 4) {
    io.result(g) <> transform.io.result(g)
    if (columnLookahead) io.result(g).bits.key := resultKey(g)
    if (directResidual) {
      val e = projection.get.io.residual.get.seedE(g)
      e.valid := transform.io.result(g).valid && io.result(g).ready
      e.bits.column := transform.io.result(g).bits.key
      for (r <- 0 until side) e.bits.values(r) := transform.io.result(g).bits.values(r)(15, 0)
      io.result(g).valid := transform.io.result(g).valid && e.ready
      transform.io.result(g).ready := io.result(g).ready && e.ready
    } else if (withProjection) {
      val u = projection.get.io.u(g)
      val isU = resultIsU(g)
      u.valid := transform.io.result(g).valid && io.result(g).ready && isU && !futureResults
      u.bits.column :=
        (if (mixedLookahead) resultColumn.get(g)
         else transform.io.result(g).bits.key)
      for (r <- 0 until side) u.bits.values(r) := transform.io.result(g).bits.values(r)(15, 0)
      val futureBanks = (0 until words).map(w => (g * words + w, 8 + g * words + w))
      val futureWritable =
        if (mixedLookahead)
          futureBanks
            .map { case (lo, hi) =>
              Mux(
                isU,
                io.bankEnable(hi) && !transform.io.matrix.valid &&
                  !backend.get.io
                    .workWrite(hi)
                    .valid,
                io.bankEnable(lo) && !io.projection.get.result.valid &&
                  !backend.get.io
                    .workWrite(lo)
                    .valid
              )
            }
            .reduce(_ && _)
        else true.B
      io.result(g).valid := transform.io.result(g).valid && Mux(
        futureResults,
        futureWritable,
        !isU || u.ready
      )
      transform.io.result(g).ready := io.result(g).ready && Mux(
        futureResults,
        futureWritable,
        !isU || u.ready
      )
      if (mixedLookahead) {
        val replay = preparedU.get.io.result(g)
        when(preparedLayout.get || earlyProjection) {
          u.valid := replay.valid
          u.bits.column := replay.bits.key
          for (r <- 0 until side) u.bits.values(r) := replay.bits.values(r)(15, 0)
        }
        replay.ready := u.ready && (preparedLayout.get || earlyProjection)
      }
      when(io.result(g).fire && !isU && !futureResults) {
        wWritten.get(g) := wWritten.get(g) | (1.U(128.W) << resultOffset(g))
      }
    }
    when(io.result(g).fire && !futureResults) {
      assert(!written(g)(resultKey(g)(7, 0)))
      written(g)(resultKey(g)(7, 0)) := true.B
    }
  }
  when(transform.io.matrix.fire) {
    val index = ((transform.io.matrix.bits.job - 10.U) * side.U + transform.io.matrix.bits.row)(
      log2Ceil(10 * side) - 1,
      0
    )
    if (preparationLookahead)
      when(futurePreparing.get) { futureMatrix.get(index) := true.B }.otherwise {
        matrixWritten(index) := true.B
      }
    else matrixWritten(index) := true.B
  }
  // M is only consumed as BF16 by H/O. Move the SAME narrowing to its
  // producer, and store one token column across the high bank half. No new
  // arithmetic: the backend's side-wide M narrowing bank is removed.
  val packedM =
    if (streamedStateOutput)
      Some(VecInit(transform.io.matrix.bits.values.map(FloatArithmetic.narrow)))
    else None
  for (b <- 0 until 16) {
    workPorts
      .read(b)
      .valid := hostRead.valid && !active && !io.start.valid && !hostPending && hostLegal &&
      hostInputBank === b.U
    if (compactOutputs) when(hostAddress < 256.U) { workPorts.read(b).valid := false.B }
    workPorts.read(b).bits :=
      (if (chunkLookahead && !compactOutputs)
         Mux(
           hostAddress < 256.U,
           512.U + Mux(
             io.lookahead.get.readPreviousOutput,
             0.U,
             Mux(sourceSlot.get, 128.U, 0.U)
           ) + hostAddress,
           hostAddress
         )
       else hostAddress)
    workPorts.response(b).ready := hostPending && hostBank === b.U && hostResponse.ready
    workPorts.write(b).valid := false.B
    workPorts.write(b).bits := 0.U.asTypeOf(workPorts.write(b).bits)
    if (withProjection) {
      val p = projection.get
      val h = io.projection.get.initH(b)
      p.io.read(b).ready := workPorts.read(b).ready && active
      p.io.response(b).valid := workPorts.response(b).valid && active
      p.io.response(b).bits := workPorts.response(b).bits
      when(active) {
        workPorts.read(b) <> p.io.read(b)
        workPorts.response(b).ready := p.io.response(b).ready
      }
      h.ready := !active && !io.start.valid && !hostPending && h.bits.address >= 256.U
      workPorts.write(b).valid := h.valid && h.ready
      workPorts.write(b).bits := h.bits
    }
    if (b < 4 * words) {
      val g = b / words
      val w = b % words
      if (directResidual) {
        val r = prep.io.rhs.get.result(g)
        when(r.fire && r.bits.key >= command.valueColumns) {
          workPorts.write(b).valid := true.B
          workPorts.write(b).bits.address := r.bits.key
          workPorts.write(b).bits.data := Cat(
            (0 until 8).reverse.map(k =>
              if (w * 8 + k < side) residualBk.get(g)(w * 8 + k) else 0.U(16.W)
            )
          )
          workPorts.write(b).bits.mask := ((BigInt(1) << math.min(16, 2 * side)) - 1).U
        }
      } else
        when(io.result(g).fire && !futureResults) {
          workPorts.write(b).valid := true.B
          workPorts.write(b).bits.address := resultKey(g)
          workPorts.write(b).bits.data := Cat(
            (0 until 8).reverse.map(k =>
              if (w * 8 + k < side) io.result(g).bits.values(w * 8 + k)(15, 0) else 0.U(16.W)
            )
          )
          workPorts.write(b).bits.mask := ((BigInt(1) << math.min(16, 2 * side)) - 1).U
        }
    }
    if (mixedLookahead && (b < 4 * words || (b >= 8 && b < 8 + 4 * words))) {
      val high = b >= 8
      val g = (if (high) b - 8 else b) / words
      val w = b % words
      val r = io.result(g)
      val isU = resultIsU(g)
      when(futureResults && r.fire && (if (high) isU else !isU)) {
        if (!high) assert(oldReadsRetired.get)
        workPorts.write(b).valid := true.B
        workPorts.write(b).bits.address :=
          (if (high) Cat(1.U(1.W), resultColumn.get(g))
           else resultColumn.get(g))
        workPorts.write(b).bits.mask := ((BigInt(1) << math.min(16, 2 * side)) - 1).U
        workPorts.write(b).bits.data := Cat(
          (0 until 8).reverse.map(k =>
            if (w * 8 + k < side) r.bits.values(w * 8 + k)(15, 0) else 0.U(16.W)
          )
        )
      }
    }
    if (!withProjection && b >= 16 - side / 4) {
      val r = transform.io.matrix
      val w = b - (16 - side / 4)
      workPorts.write(b).valid := r.fire
      workPorts.write(b).bits.address := 256.U + (r.bits.job - 10.U) * side.U + r.bits.row
      workPorts.write(b).bits.data := Cat((0 until 4).reverse.map(k => r.bits.values(w * 4 + k)))
      workPorts.write(b).bits.mask := 65535.U
    }
    if (streamedStateOutput && b >= 8 && b < 8 + side / 2) {
      val m = transform.io.matrix
      for (
        ((_, i, j), job) <- SharedGramLayout.jobs.zipWithIndex if job >= 10 &&
          (b - 8) * 8 <
          (i + 1) * side &&
          (b - 8) * 8 + 8 > i * side
      ) {
        val lanes = (0 until 8).filter(k =>
          (b - 8) * 8 + k >= i * side &&
            (b - 8) * 8 + k <
            (i + 1) * side
        )
        when(m.fire && m.bits.job === job.U) {
          workPorts.write(b).valid := true.B
          workPorts.write(b).bits.address := j.U * side.U + m.bits.row
          if (preparationLookahead)
            workPorts.write(b).bits.address := Mux(
              if (rollingLookahead) sourceSlot.get ^ futurePreparing.get
              else sourceSlot.get || futurePreparing.get,
              (4 * side).U,
              0.U
            ) + j.U * side.U + m.bits.row
          workPorts.write(b).bits.mask := lanes.map(k => BigInt(3) << (2 * k)).reduce(_ | _).U
          workPorts.write(b).bits.data := Cat(
            (0 until 8).reverse.map(k =>
              if (lanes.contains(k)) packedM.get((b - 8) * 8 + k - i * side) else 0.U(16.W)
            )
          )
        }
      }
    }
    if (streamedStateOutput && b < side / 2) {
      val e = io.projection.get.result
      when(e.fire) {
        // This page previously held U/W. The last projection frame must have
        // consumed every W column before E extraction can overwrite it.
        assert(
          projection.get.io.eReadsDone &&
            (transform.io.idle || futurePreparing.getOrElse(
              false.B
            )) && (rhs.io.idle || futureRhsStarted.getOrElse(false.B))
        )
        workPorts.write(b).valid := true.B
        workPorts
          .write(b)
          .bits
          .address := 128.U + ((e.bits.group * side.U + e.bits.row) << 1) + e.bits.half.asUInt
        workPorts.write(b).bits.data := Cat((0 until 8).reverse.map(k => e.bits.values(b * 8 + k)))
        workPorts.write(b).bits.mask := 65535.U
      }
    }
    if (withProjection && !streamedStateOutput && b >= 8) {
      val m = transform.io.matrix
      val mr = (m.bits.job - 10.U) * side.U + m.bits.row
      val part = (b - 8) / (side / 4)
      val w = (b - 8) % (side / 4)
      when(m.fire && (mr % (32 / side).U) === part.U) {
        workPorts.write(b).valid := true.B
        workPorts.write(b).bits.address := mr / (32 / side).U
        workPorts.write(b).bits.data := Cat((0 until 4).reverse.map(k => m.bits.values(w * 4 + k)))
        workPorts.write(b).bits.mask := 65535.U
      }
      if (b < 8 + side / 2) {
        val e = io.projection.get.result
        when(e.fire) {
          assert(!m.fire, "M and E share the high Work banks")
          workPorts.write(b).valid := true.B
          workPorts
            .write(b)
            .bits
            .address := 128.U + ((e.bits.group * side.U + e.bits.row) << 1) + e.bits.half.asUInt
          workPorts.write(b).bits.data := Cat(
            (0 until 8).reverse.map(k => e.bits.values((b - 8) * 8 + k))
          )
          workPorts.write(b).bits.mask := 65535.U
        }
      }
    }
    when(workPorts.write(b).valid) { assert(workPorts.write(b).ready) }
  }
  when(prep.io.done && !futureResults) { prepFinished := true.B }
  if (preparationLookahead) when(prep.io.done && !futureResults) {
    assert(!futurePreparing.get)
    currentDecay.get := prep.io.prefix(4 * side - 1)
    decayValid.get := true.B
  }
  if (mixedLookahead) when(prep.io.done && futureResults) { futureDone.get := true.B }
  // Once future preparation starts, its busy/Gram-cache state no longer
  // belongs to the current prefix. Old CS/RHS/Qbar and N retired at the lease.
  val finish = active && prepFinished && !gram.io.busy &&
    (transform.io.idle ||
      futurePreparing
        .getOrElse(false.B)) &&
    (rhs.io.idle || futureRhsStarted.getOrElse(false.B)) &&
    panels
      .map(p => Mux(futureRhsStarted.getOrElse(false.B), p.io.contextIdle.asUInt.andR, p.io.idle))
      .reduce(_ && _) && projection.map(_.io.idle).getOrElse(true.B) &&
    query
      .map(_.io.idle)
      .getOrElse(true.B) &&
    (cache.map(_.io.idle).getOrElse(true.B) || futurePreparing.getOrElse(
      false.B
    ))
  if (!withStateOutput) when(finish) { active := false.B }
  when(finish && !prefixRetired) { prefixRetired := true.B }
  io.done := finish
  io.busy := active
  io.coefficientsDone := prep.io.rhs.get.coefficientsDone
  io.preparationDone := prep.io.done
  io.gramDone := gram.io.done
  io.gatesReady := prep.io.gatesReady
  io.gramIssue := VecInit(gram.io.issued.map(_.valid)).asUInt
  io.gramJob := VecInit(gram.io.issued.map(_.bits))
  io.dispatchedCS := VecInit(
    panels.map(p => p.io.dispatched.valid && !p.io.dispatched.bits.os)
  ).asUInt
  io.dispatchedOS := VecInit(
    panels.map(p => p.io.dispatched.valid && p.io.dispatched.bits.os)
  ).asUInt
  io.ingressOccupancy := VecInit(panels.map(_.io.inputOccupancy))
  io.frozen := VecInit(panels.map(p => !p.io.advance)).asUInt
  io.csIssue := transform.io.reserved
  io.csReturned := VecInit(transform.io.returned.map(_.fire)).asUInt
  io.csFed := transform.io.fed
  io.captured := transform.io.captured
  io.occupied := transform.io.occupied
  io.loaded := transform.io.loaded
  io.coefficientLoad := VecInit(transform.io.load.map(_.fire)).asUInt
  io.coefficient := Mux(
    transform.io.load.map(_.fire).reduce(_ || _),
    (PriorityEncoder(io.coefficientLoad) * side.U + Mux1H(
      transform.io.load.map(_.fire),
      transform.io.load.map(_.bits.index)
    )).zext,
    (-1).S
  )
  io.matrix := Mux(
    transform.io.matrix.fire,
    (transform.io.matrix.bits.job * side.U + transform.io.matrix.bits.row).zext,
    (-1).S
  )
  io.gramReads := gram.io.allocated
  io.rhsReads := rhs.io.reads
  io.rhsFed := rhs.io.fed
  io.gateIssue := prep.io.gateIssue
  io.fmaIssue := prep.io.fmaIssue
  io.inverseIssue := prep.io.inverseIssue
  io.rhsIssue := prep.io.rhs.get.issued
  io.factorIssue := prep.io.rhs.get.factorIssued
  backend.foreach { b =>
    val o = io.output.get
    b.io.start.valid := io.start.fire
    b.io.start.bits := io.start.bits
    b.io.canPrepare := prepFinished && (rhs.io.idle || futureRhsStarted.getOrElse(false.B))
    b.io.prefixReady := prefixRetired
    b.io.decay := prep.io.prefix(4 * side - 1)
    b.io.scale <> prep.io.scale.get
    if (externalResources) {
      val x = io.recurrentScale.get
      val p = prep.io.scale.get
      val selected = io.recurrentEnable.get
      p.request.valid := Mux(selected, x.request.valid, b.io.scale.request.valid)
      p.request.bits := Mux(selected, x.request.bits, b.io.scale.request.bits)
      x.request.ready := p.request.ready && selected
      b.io.scale.request.ready := p.request.ready && !selected
      p.response.ready := Mux(selected, x.response.ready, b.io.scale.response.ready)
      x.response.valid := p.response.valid && selected
      x.response.bits := p.response.bits
      x.tailGate := p.tailGate
      // The prefill backend has no response reservation during native mode.
      // Deliver the scale response without a global mode mask: only its
      // existing seed/production reservation can consume it. This avoids a
      // mode -> response-valid -> H/O address -> SRAM collision control path.
      b.io.scale.response.valid := p.response.valid
      when(selected) {
        assert(!active && !prep.io.busy, "recurrent lanes require retired prefill ownership")
        assert(!b.io.scale.response.ready, "inactive prefill backend cannot consume native returns")
        assert(!b.io.scale.request.valid, "inactive prefill backend cannot issue native work")
      }
    }
    if (preparationLookahead) {
      b.io.decay := currentDecay.get
      when(b.io.canPrepare) { assert(decayValid.get) }
    }
    b.io.bankEnable := io.bankEnable
    b.io.panelEnable := Mux(io.projection.get.enable, 65535.U, 0.U)
    o.prefixDone := prefixRetired
    o.nDone := b.io.nDone
    o.owned := b.io.ownsPanels
    o.issued := b.io.issued
    o.scaleIssued := b.io.scaleIssued
    o.committed := b.io.committed
    o.operandReads := VecInit(b.io.operandRead.map(_.fire)).asUInt
    o.workReads := VecInit(b.io.workRead.map(_.fire)).asUInt
    io.done := b.io.done
    if (mixedLookahead) {
      when(b.io.done) { backendFinished.get := true.B }
      io.done := active && (b.io.done || backendFinished.get) && projection.get.io.idle
    }
    when(io.done) {
      active := false.B
      outputReady := true.B
      outputSlots.foreach(s => s := s | (1.U(2.W) << sourceSlot.get))
    }
    when(b.io.ownsPanels) {
      assert(
        prefixRetired &&
          (futureRunning
            .getOrElse(false.B) || gramOwner.get.map(_ === 0.U).reduce(_ && _))
      )
    }
    for (p <- 0 until 16) {
      val cell = panels(p)
      val old = panelPorts(p)
      val owned = b.io.ownsPanels
      if (residentProjection) {
        // No result read is needed to hand odd contexts to H/O. Admission
        // completion alone is insufficient: contextIdle includes both input
        // queue entries and every enabled-cycle arithmetic/link register.
        val initialized = projection.get.io.joint.get.initialized.get(p)
        when(oldReadsRetired.get && initialized(0) && cell.io.contextIdle(1)) {
          z0Released.get(p) := true.B
        }
        when(oldReadsRetired.get && initialized(1) && cell.io.contextIdle(3)) {
          zReleased.get(p) := true.B
        }
        when(io.start.fire) {
          z0Released.get(p) := false.B
          zReleased.get(p) := false.B
        }
      }
      val concurrent = (futureRunning.getOrElse(false.B) || earlyProjection) && owned
      val issue = Wire(Decoupled(new RetimedPanelInput(side, 2)))
      issue.valid := b.io.input(p).valid
      issue.bits := b.io.input(p).bits
      b.io.input(p).ready := issue.ready
      if (chunkLookahead) {
        // Prefer ready current-chunk work; use its four-cycle recurrence holes
        // for C1. Blind round robin can choose a cooling C1 and block ready H/O.
        val currentLease =
          if (residentProjection)
            !b.io.input(p).bits.context(0) ||
            Mux(b.io.input(p).bits.context(1), zReleased.get(p), z0Released.get(p))
          else if (mixedLookahead) b.io.input(p).bits.context =/= 3.U || zReleased.get(p)
          else true.B
        val currentReady = b.io.input(p).valid && currentLease && cell.io.osAdmissible.get(
          b.io.input(p).bits.context
        )
        val mixed = futureRhsStarted.getOrElse(false.B) && owned && !old.in.bits.os
        val futureReady = old.in.valid && ((concurrent && cell.io.osAdmissible.get(1)) || mixed)
        val locked = RegInit(false.B)
        val held = Reg(Bool())
        val chooseFuture = Mux(locked, held, !currentReady)
        issue.valid := Mux(
          locked,
          Mux(held, old.in.valid && (concurrent || mixed), b.io.input(p).valid),
          currentReady || futureReady
        )
        issue.bits := Mux(chooseFuture, old.in.bits, b.io.input(p).bits)
        b.io.input(p).ready := issue.ready && !chooseFuture && issue.valid
        old.in.ready := Mux(owned, issue.ready && chooseFuture && issue.valid, cell.io.in.ready)
        when(issue.valid && !issue.ready) {
          locked := true.B
          held := chooseFuture
        }
        // Once H/O releases ownership, old.in goes directly to the panel.
        // A held future CS may then fire on that bypass, not on issue.fire.
        // Do not carry its arbitration lock into the next H/O command.
        when(!owned || issue.fire) { locked := false.B }
        when(io.start.fire) {
          assert(!locked, "a backend selection cannot outlive its ownership epoch")
        }
      } else old.in.ready := cell.io.in.ready && !owned
      cell.io.in.valid := Mux(owned, issue.valid, old.in.valid)
      cell.io.in.bits := Mux(owned, issue.bits, old.in.bits)
      issue.ready := cell.io.in.ready && owned
      cell.io.seed.valid := Mux(owned && !earlyProjection, b.io.seed(p).valid, old.seed.valid)
      cell.io.seed.bits := Mux(owned && !earlyProjection, b.io.seed(p).bits, old.seed.bits)
      cell.io.seedColumn.get := !owned || earlyProjection
      old.seed.ready := cell.io.seed.ready && (!owned || earlyProjection)
      b.io.seed(p).ready := cell.io.seed.ready && owned && !earlyProjection
      val oldZ =
        if (mixedLookahead && !residentProjection)
          !zReleased.get(p) && old.read.bits.context === 3.U
        else false.B
      val futureRead = (concurrent || oldZ) && old.read.valid
      cell.io.read.valid := Mux(owned, b.io.rowRead(p).valid || futureRead, old.read.valid)
      cell.io.read.bits := Mux(
        owned,
        Mux(b.io.rowRead(p).valid, b.io.rowRead(p).bits, old.read.bits),
        old.read.bits
      )
      cell.io.readColumn.foreach(_ := !owned && old.readColumn.get)
      old.read.ready := cell.io.read.ready &&
        (!owned ||
          ((concurrent || oldZ) &&
            !b.io
              .rowRead(p)
              .valid))
      b.io.rowRead(p).ready := cell.io.read.ready && owned
      val futureResponse = (concurrent && cell.io.response.bits.context === 1.U) ||
        (if (mixedLookahead && !residentProjection)
           !zReleased.get(p) && cell.io.response.bits.context === 3.U
         else false.B)
      cell.io.response.ready := Mux(
        owned && !futureResponse,
        b.io.rowResponse(p).ready,
        old.response.ready
      )
      old.response.valid := cell.io.response.valid && (!owned || futureResponse)
      b.io.rowResponse(p).valid := cell.io.response.valid && owned && !futureResponse
      b.io.rowResponse(p).bits := cell.io.response.bits
      if (mixedLookahead) {
        when(b.io.input(p).fire && b.io.input(p).bits.context === 3.U) {
          assert(zReleased.get(p), "O1 cannot overwrite a live Z1 panel")
        }
        when(b.io.rowRead(p).fire && b.io.rowRead(p).bits.context === 3.U) {
          assert(zReleased.get(p))
        }
      }
      if (residentProjection) {
        when(b.io.input(p).fire && b.io.input(p).bits.context === 1.U) {
          assert(z0Released.get(p), "O0 must wait for resident Z0 retirement")
        }
        when(b.io.rowRead(p).fire && b.io.rowRead(p).bits.context === 1.U) {
          assert(z0Released.get(p))
        }
        when(owned && !earlyProjection) {
          assert(
            !projection.get.io.input(p).valid,
            "projection admissions must finish before H/O ownership"
          )
        }
      }
      when(owned) {
        cell.io.clear.valid := old.clear.valid && concurrent
        cell.io.coefficient.valid := false.B
      }
      old.clear.ready := cell.io.clear.ready && (!owned || concurrent)
      old.coefficient.ready := cell.io.coefficient.ready && !owned
      if (preparationLookahead) {
        when(owned) { cell.io.coefficient.valid := old.coefficient.valid && futurePreparing.get }
        old.coefficient.ready := cell.io.coefficient.ready && (!owned || futurePreparing.get)
      }
      if (stateEpilogue) {
        val zero = b.io.clear.get(p)
        zero.ready := cell.io.clear.ready && owned
        when(owned) {
          cell.io.clear.valid := zero.valid || (old.clear.valid && concurrent)
          cell.io.clear.bits := Mux(zero.valid, zero.bits, old.clear.bits)
          old.clear.ready := cell.io.clear.ready && concurrent && !zero.valid
        }
        when(zero.fire) { assert(zero.bits === 0.U || zero.bits === 2.U) }
      }
      if (chunkLookahead) when(concurrent) {
        when(old.clear.fire) { assert(old.clear.bits === 1.U) }
        when(old.in.fire && old.in.bits.os) { assert(old.in.bits.context === 1.U) }
        assert(b.io.c1Released)
      }
    }
    for (i <- 0 until 16) {
      val om = operandAccess
      val wm = workMemory.io
      val nop = b.io.operandOwner(i)
      val nw = b.io.workOwner
      om.read(i).valid := Mux(nop, b.io.operandRead(i).valid, operandPorts.read(i).valid)
      om.read(i).bits := Mux(nop, b.io.operandRead(i).bits, operandPorts.read(i).bits)
      operandPorts.read(i).ready := om.read(i).ready && !nop
      b.io.operandRead(i).ready := om.read(i).ready && nop
      om.response(i).ready := Mux(
        nop,
        b.io.operandResponse(i).ready,
        operandPorts.response(i).ready
      )
      operandPorts.response(i).valid := om.response(i).valid && !nop
      b.io.operandResponse(i).valid := om.response(i).valid && nop
      b.io.operandResponse(i).bits := om.response(i).bits
      // Original Z/Qbar writes retain priority. Only the unused write slot
      // belongs to N; its per-bank queue holds a rejected word unchanged.
      om.write(i).valid := operandPorts.write(i).valid || b.io.operandWrite(i).valid
      om.write(i).bits := Mux(
        operandPorts.write(i).valid,
        operandPorts.write(i).bits,
        b.io.operandWrite(i).bits
      )
      b.io.operandWrite(i).ready := om.write(i).ready && !operandPorts.write(i).valid
      wm.read(i).valid := Mux(nw, b.io.workRead(i).valid, workPorts.read(i).valid)
      wm.read(i).bits := Mux(nw, b.io.workRead(i).bits(8, 0), workPorts.read(i).bits)
      if (preparationLookahead && i >= 8) when(nw && b.io.workRead(i).bits < (4 * side).U) {
        wm.read(i).bits := b.io.workRead(i).bits(8, 0) + Mux(sourceSlot.get, (4 * side).U, 0.U)
      }
      workPorts.read(i).ready := wm.read(i).ready && !nw
      b.io.workRead(i).ready := wm.read(i).ready && nw
      wm.response(i).ready := Mux(nw, b.io.workResponse(i).ready, workPorts.response(i).ready)
      workPorts.response(i).valid := wm.response(i).valid && !nw
      b.io.workResponse(i).valid := wm.response(i).valid && nw
      b.io.workResponse(i).bits := wm.response(i).bits
      wm.write(i).valid := Mux(nw, b.io.workWrite(i).valid, workPorts.write(i).valid)
      wm.write(i).bits := workPorts.write(i).bits
      when(nw) {
        wm.write(i).bits := b.io.workWrite(i).bits
        // O is redirected to Operand below, so Work only consumes this H
        // address. Do not gate the address with return-valid arbitration.
        if (compactOutputs) wm.write(i).bits.address := b.io.stateAddress
      }
      if (chunkLookahead && !compactOutputs) when(nw && b.io.workWrite(i).bits.address < 256.U) {
        wm.write(i).bits.address :=
          512.U + Mux(sourceSlot.get, 128.U, 0.U) + b.io.workWrite(i).bits.address
      }
      workPorts.write(i).ready := wm.write(i).ready && !nw
      b.io.workWrite(i).ready := wm.write(i).ready && nw
      if (preparationLookahead) {
        // Current H commits retain priority. The second M page only uses
        // otherwise-free writes of the same 1R1W high banks.
        when(nw && futurePreparing.get && !b.io.workWrite(i).valid) {
          wm.write(i) <> workPorts.write(i)
        }
        workPorts
          .write(i)
          .ready := wm.write(i).ready && (!nw || (futurePreparing.get && !b.io.workWrite(i).valid))
      }
      if (compactOutputs) {
        val w = b.io.workWrite(i)
        val isOutput = nw && b.io.outputWrite
        // Future M kept the otherwise-free Work port above; valid O now
        // redirects to Operand, without selecting the Work H page address.
        when(isOutput && (if (preparationLookahead) w.valid else true.B)) {
          wm.write(i).valid := false.B
          w.ready := false.B
        }
        if (i < side / 2) {
          // Two value halves use disjoint bank halves of the SAME token row.
          // Existing per-source-bank retirement masks protect partial writes.
          val low = packedOutput.get(i)
          val high = packedOutput.get(i + side / 2)
          low.valid := isOutput && w.valid && !w.bits.address(0)
          high.valid := isOutput && w.valid && w.bits.address(0)
          for (p <- Seq(low, high)) {
            // O contains exactly 4*side token rows, at most 64. The fixed
            // page prefix and six row bits replace a wide address addition.
            // The engine holds its O row while H returns are arbitrated.
            // H selection gates valid, but must not select the O address.
            p.bits.address := GdnOutputAddress(b.io.outputAddress, sourceSlot.get)
            p.bits.data := w.bits.data
            p.bits.mask := w.bits.mask
            when(p.valid) { assert(w.bits.address < (8 * side).U, "O row exceeds its packed page") }
          }
          when(isOutput) { w.ready := Mux(w.bits.address(0), high.ready, low.ready) }
        }
      }
      if (preparationLookahead) when(futurePreparing.get && workPorts.write(i).fire) {
        assert(
          wm.write(i).fire && wm.write(i).bits.address === workPorts.write(i).bits.address,
          "future M must reach physical SRAM"
        )
      }
    }
  }
  if (mixedLookahead && !columnLookahead) {
    val u = preparedU.get
    for (i <- 0 until 16) {
      if (i < 4 * words) {
        val b = i + 8
        val access = Wire(Decoupled(UInt(10.W)))
        // Capture the pre-existing logical Work port before attaching the
        // physical arbiter (workPorts contains projection/host, backend owns
        // it only after this U reader has drained).
        val arb = Module(new QueuedReadArbiter(responseDepth = 2))
        arb.io.shared := preparedLayout.get && !u.io.idle
        access.valid := workPorts.read(b).valid
        access.bits := workPorts.read(b).bits
        arb.io.request(0) <> access
        workPorts.read(b).ready := access.ready && !backend.get.io.workOwner
        workPorts.response(b).valid := arb.io.response(0).valid && !backend.get.io.workOwner
        workPorts.response(b).bits := arb.io.response(0).bits
        arb.io.response(0).ready := workPorts.response(b).ready
        when(backend.get.io.workOwner) {
          arb.io.request(0) <> backend.get.io.workRead(b)
          backend.get.io.workResponse(b) <> arb.io.response(0)
          when(backend.get.io.workRead(b).bits < (4 * side).U) {
            arb.io.request(0).bits := backend.get.io.workRead(b).bits + Mux(
              sourceSlot.get,
              (4 * side).U,
              0.U
            )
          }
        }
        arb.io.request(1).valid := u.io.read(i).valid
        arb.io.request(1).bits := u.io.read(i).bits
        u.io.read(i).ready := arb.io.request(1).ready
        u.io.response(i) <> arb.io.response(1)
        workMemory.io.read(b) <> arb.io.read
        arb.io.returned <> workMemory.io.response(b)
        when(backend.get.io.workOwner) { assert(u.io.idle) }
      } else {
        u.io.read(i).ready := false.B
        u.io.response(i).valid := false.B
        u.io.response(i).bits := 0.U
      }
    }
    for (b <- 0 until 4 * words)
      when(preparedLayout.get && !backend.get.io.workOwner && workPorts.read(b).bits < 256.U) {
        workMemory.io.read(b).bits := workPorts.read(b).bits - command.valueColumns
      }
  }
  if (columnLookahead) {
    val u = preparedU.get
    val back = backend.get
    val projectionPort = projection.get.io
    for (i <- 0 until 16) {
      val uBank = i >= 8 && i < 8 + 4 * words
      val arb = Module(new QueuedReadArbiter(responseDepth = 2, clients = if (uBank) 3 else 2))
      val nw = back.io.workOwner
      // Queue occupancy itself selects arbitration. An empty request fabric
      // permits the current reader's existing direct path; secondary readers
      // always enqueue and thus disable bypass on the following cycle.
      // Do not force two queue stages merely because a future epoch is live.
      arb.io.shared := false.B
      arb.io.request(0).valid := Mux(
        nw,
        back.io.workRead(i).valid,
        workPorts.read(i).valid && !earlyProjection
      )
      val normalAddress =
        if (i < 4 * words)
          Mux(
            preparedLayout.get && workPorts.read(i).bits < 256.U,
            workPorts.read(i).bits - command.valueColumns,
            workPorts.read(i).bits
          )
        else workPorts.read(i).bits
      arb.io.request(0).bits := Mux(nw, back.io.workRead(i).bits, normalAddress)
      if (i >= 8) when(nw && back.io.workRead(i).bits < (4 * side).U) {
        arb.io.request(0).bits := back.io.workRead(i).bits + Mux(sourceSlot.get, (4 * side).U, 0.U)
      }
      back.io.workRead(i).ready := arb.io.request(0).ready && nw
      back.io.workResponse(i).valid := arb.io.response(0).valid && nw
      back.io.workResponse(i).bits := arb.io.response(0).bits
      workPorts.read(i).ready := Mux(
        earlyProjection,
        arb.io.request(1).ready,
        arb.io.request(0).ready && !nw
      )
      workPorts.response(i).valid := Mux(
        earlyProjection,
        arb.io.response(1).valid,
        arb.io.response(0).valid && !nw
      )
      workPorts.response(i).bits := Mux(
        earlyProjection,
        arb.io.response(1).bits,
        arb.io.response(0).bits
      )
      arb.io.response(0).ready := Mux(
        nw,
        back.io.workResponse(i).ready,
        workPorts.response(i).ready && !earlyProjection
      )
      arb.io.request(1).valid := earlyProjection && workPorts.read(i).valid
      arb.io.request(1).bits := workPorts.read(i).bits
      arb.io.response(1).ready := earlyProjection && workPorts.response(i).ready
      if (uBank) {
        val b = i - 8
        arb.io.request(2).valid := u.io.read(b).valid
        arb.io.request(2).bits := u.io.read(b).bits
        arb.io.response(2).ready := u.io.response(b).ready
        u.io.read(b).ready := arb.io.request(2).ready
        u.io.response(b).valid := arb.io.response(2).valid
        u.io.response(b).bits := arb.io.response(2).bits
      }
      workMemory.io.read(i) <> arb.io.read
      arb.io.returned <> workMemory.io.response(i)
      when(io.start.fire) {
        assert(arb.io.idle, "Work readers must drain before ownership switches")
      }
    }
    for (i <- 4 * words until 16) {
      u.io.read(i).ready := false.B
      u.io.response(i).valid := false.B
      u.io.response(i).bits := 0.U
    }
  }
  if (chunkLookahead) {
    val l = io.lookahead.get
    // A previous O slot is readable only after its actual SRAM writes retire.
    when(hostRead.valid && l.readPreviousOutput) {
      assert(outputSlots.get.andR && hostAddress < (8 * side).U)
    }
    val futureWriteReady = Wire(Vec(4, Bool()))
    val writePhase = if (earlyFullGram) Some(RegInit(false.B)) else None
    writePhase.foreach(x => when(futureRunning.get) { x := !x })
    for (g <- 0 until 4) {
      def free(base: Int) = (0 until side / 4)
        .map(i =>
          !operandAccess
            .write(base + i)
            .valid && !packedOutput.map(_(base + i).valid).getOrElse(false.B)
        )
        .reduce(_ && _)
      val tail = gram.io.result(g).bits.job >= 16.U
      val available =
        if (earlyFullGram && g < 2) Mux(tail, free(8 + 4 * g), free(4 * g)) else free(4 * g)
      val grant = if (earlyFullGram) {
        // Only groups 0/2 and 1/3 can share tail banks. A registered turn
        // avoids a four-way result-ready priority chain; normal groups remain
        // independent. A conflicting blocked row holds its original payload.
        if (g < 2) !(tail && gram.io.result(g + 2).valid) || !writePhase.get
        else
          !(gram.io.result(g - 2).valid && gram.io.result(g - 2).bits.job >= 16.U) || writePhase.get
      } else true.B
      futureWriteReady(g) := available && grant
    }
    for (g <- 0 until 4) when(futureRunning.get) { gram.io.result(g).ready := futureWriteReady(g) }
    if (compactOutputs) {
      val physicalBank = hostInputBank + Mux(hostAddress(0), (side / 2).U, 0.U)
      val outputRead = hostAddress < 256.U
      val mayRead = !active && !io.start.valid && !hostPending && hostLegal && !zHostPending.get &&
        !io.joint.get.hostRead.valid
      when(outputRead) { hostRead.ready := mayRead && operandAccess.read(physicalBank).ready }
      when(hostRead.fire) {
        hostInOperand.get := outputRead
        when(outputRead) { hostBank := physicalBank }
      }
      when(hostResponse.fire) { hostInOperand.get := false.B }
      when(hostInOperand.get) {
        hostResponse.valid := hostPending && operandAccess.response(hostBank).valid
        hostResponse.bits := operandAccess.response(hostBank).bits
      }
      val ownsRead = hostInOperand.get || (mayRead && hostRead.valid && outputRead)
      when(ownsRead) {
        assert(!active && !zHostPending.get)
        io.joint.get.hostRead.ready := false.B
        for (i <- 0 until 16) {
          operandAccess
            .read(i)
            .valid := mayRead && hostRead.valid && outputRead && physicalBank === i.U
          operandAccess.read(i).bits := Mux(
            l.readPreviousOutput || !sourceSlot.get,
            576.U,
            704.U
          ) + (hostAddress >> 1)
          operandAccess
            .response(i)
            .ready := hostPending && hostInOperand.get && hostBank === i.U && hostResponse.ready
          operandPorts.read(i).ready := false.B
          operandPorts.response(i).valid := false.B
        }
      }
      io.compact.get.writes := VecInit(packedOutput.get.map(_.fire)).asUInt
      io.compact.get.blocked := VecInit(packedOutput.get.map(p => p.valid && !p.ready)).asUInt
    }
    for (i <- 0 until 16) {
      val memory = operandMemory.io
      val arb = Module(new QueuedReadArbiter(responseDepth = if (preparationLookahead) 2 else 0))
      arb.io.shared := futureRunning.get || !cache.get.io.idle || futureRhsStarted.getOrElse(
        false.B
      )
      arb.io.request(0) <> operandAccess.read(i)
      when(sourceSlot.get && operandAccess.read(i).bits < 256.U) {
        arb.io.request(0).bits := 768.U + operandAccess.read(i).bits
      }
      operandAccess.response(i) <> arb.io.response(0)
      memory.read(i) <> arb.io.read
      arb.io.returned <> memory.response(i)
      arb.io.request(1).valid := Mux(
        futureRunning.get,
        gram.io.bankRead(i).valid,
        cache.get.io.read(i).valid
      )
      arb.io.request(1).bits := Mux(
        futureRunning.get,
        (if (rollingLookahead) Mux(sourceSlot.get, 0.U, 768.U) else 768.U) +
          gram.io
            .bankRead(i)
            .bits,
        cache.get.io.read(i).bits
      )
      cache.get.io.read(i).ready := arb.io.request(1).ready && !futureRunning.get
      cache.get.io.response(i).valid := arb.io.response(1).valid && !futureRunning.get
      cache.get.io.response(i).bits := arb.io.response(1).bits
      arb.io.response(1).ready := Mux(
        futureRunning.get,
        gram.io.bankResponse(i).ready,
        cache.get.io.response(i).ready
      )
      when(futureRunning.get) {
        gram.io.bankRead(i).ready := arb.io.request(1).ready
        gram.io.bankResponse(i).valid := arb.io.response(1).valid
        gram.io.bankResponse(i).bits := arb.io.response(1).bits
      }
      if (mixedLookahead && i < 4 * words) when(futureRhsStarted.get) {
        rhs.io.read(i).ready := false.B
        rhs.io.response(i).valid := false.B
        when(cache.get.io.released(i)) {
          arb.io.request(1).valid := rhs.io.read(i).valid
          arb.io.request(1).bits :=
            (if (rollingLookahead) Mux(sourceSlot.get, 0.U, 768.U)
             else 768.U) + rhs.io.read(i).bits
          rhs.io.read(i).ready := arb.io.request(1).ready
          rhs.io.response(i) <> arb.io.response(1)
          cache.get.io.read(i).ready := false.B
          cache.get.io.response(i).valid := false.B
        }
        // Disconnect the old exclusive RHS view from client 0. The current
        // backend remains free to use that client for N/E/Z/H/O.
        when(active && !backend.get.io.operandOwner(i)) {
          arb.io.request(0).valid := false.B
          arb.io.response(0).ready := false.B
        }
      }
      val init = l.init(i)
      init.ready := !active && !io.start.valid && !futurePending.get && !futureRunning.get &&
        !futureCached.get &&
        !operandAccess
          .write(i)
          .valid
      when(init.fire) {
        assert(init.bits.address < 256.U)
        memory.write(i).valid := true.B
        memory.write(i).bits := init.bits
        memory.write(i).bits.address := 768.U + init.bits.address
      }
      if (compactOutputs) {
        val p = packedOutput.get(i)
        p.ready := memory.write(i).ready && !operandAccess.write(i).valid && io.bankEnable(i)
        when(p.valid && !operandAccess.write(i).valid && io.bankEnable(i)) {
          memory.write(i).valid := true.B
          memory.write(i).bits := p.bits
        }
      }
      val g = i / 4
      val lane = i % 4
      val r = gram.io.result(g)
      if (lane < side / 4)
        when(
          futureRunning.get && r.fire && (if (earlyFullGram && g < 2) r.bits.job < 16.U else true.B)
        ) {
          assert(r.bits.job < (if (earlyFullGram) 20 else 16).U && r.bits.job(1, 0) === g.U)
          memory.write(i).valid := true.B
          memory.write(i).bits.address :=
            (if (earlyFullGram && g >= 2)
               Mux(
                 r.bits.job >= 16.U,
                 (896 + side).U + r.bits.row,
                 640.U + (r.bits.job >> 2) * side.U + r.bits.row
               )
             else 640.U + (r.bits.job >> 2) * side.U + r.bits.row)
          memory.write(i).bits.mask := 65535.U
          memory.write(i).bits.data := Cat(
            (0 until 4).reverse.map(j => r.bits.values(4 * lane + j))
          )
        }
      if (earlyFullGram && i >= 8 && lane < side / 4) {
        val first = gram.io.result(g - 2)
        when(futureRunning.get && first.fire && first.bits.job >= 16.U) {
          assert(!r.fire, "two Gram rows cannot write one physical bank")
          memory.write(i).valid := true.B
          memory.write(i).bits.address := 896.U + first.bits.row
          memory.write(i).bits.mask := 65535.U
          memory.write(i).bits.data := Cat(
            (0 until 4).reverse.map(j => first.bits.values(4 * lane + j))
          )
        }
      }
      if (residentProjection) when(memory.write(i).fire) {
        assert(
          memory.write(i).bits.address < 384.U || memory.write(i).bits.address >= 512.U,
          "resident Z must never write the unused snapshot page"
        )
      }
    }
  }
}
