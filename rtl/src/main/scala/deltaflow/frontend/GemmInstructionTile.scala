package deltaflow.frontend

import chisel3._
import chisel3.util._
import chisel3.experimental.hierarchy._
import deltaflow.isa._
import deltaflow.sa._
import deltaflow.memory._
import deltaflow.system._

class GemmWave(t: Int) extends Bundle {
  val context = UInt(2.W)
  val a = Vec(t, UInt(16.W))
  val b = Vec(t, UInt(16.W))
}

class GemmRowTag(side: Int) extends Bundle {
  val context = UInt(2.W)
  val group = UInt(2.W)
  val row = UInt(log2Ceil(side).W)
}

class GemmStoreTrace(t: Int) extends Bundle {
  val address = UInt(9.W)
  val values = Vec(t, UInt(32.W))
}

/** Four-context OS executor on the existing sixteen microarrays. GEMM completion releases input
  * storage; STORE/FENCE wait for retirement. SRAM buffers are caller initialized, and reset does
  * not clear memory. Input, CLEAR, STORE and SEED requests occupy mutually exclusive FSM phases.
  */
class GemmInstructionTile(
    val side: Int = 16,
    val tileId: Int = 0,
    val externalResources: Boolean = false,
    val independentSeedReads: Boolean = true
) extends Module {
  require(Set(4, 16).contains(side) && tileId >= 0 && tileId < 4)
  val operandDepth = if (externalResources) 1024 else 512
  val t = 4 * side
  val operandWords = t / 8
  val resultWords = t / 4

  val io = IO(new Bundle {
    val resources = if (externalResources) Some(new TileResources(side)) else None
    val invalidateContexts = if (externalResources) Some(Input(Bool())) else None
    val inputsConsumed = if (externalResources) Some(Output(Bool())) else None
    val inputLease = if (externalResources) Some(Output(new Bundle {
      val valid = Bool()
      val begin = UInt(10.W)
      val end = UInt(11.W)
    }))
    else None
    val instruction = Flipped(Decoupled(UInt(128.W)))
    val completion = Decoupled(new GemmCompletion)
    val load = Flipped(Vec(4, Decoupled(new GemmLoad)))
    val hostRead = Flipped(Decoupled(new GemmHostRead))
    val hostResponse = Decoupled(UInt(128.W))
    val storeEnable = Input(Bool())
    val issued = Output(Bool())
    val committed = Valid(new GemmStoreTrace(t))
    val idle = Output(Bool())
    val phase = Output(UInt(4.W))
  })

  val panel = if (externalResources) {
    Seq.tabulate(4, 4)((r, c) => new ResourceHandle(io.resources.get.panels(4 * r + c)))
  } else {
    val definition = Definition(new RetimedWavePanel(side))
    Seq.fill(4, 4)(new ResourceHandle(Instance(definition).io))
  }
  val operand = new ResourceHandle(
    if (externalResources) io.resources.get.operand
    else Module(new MacroScratchpad(16, 512, 16)).io
  )
  val work = new ResourceHandle(
    if (externalResources) io.resources.get.work
    else Module(new MacroScratchpad(16, 512, 16)).io
  )

  val commands = Module(new Queue(UInt(128.W), 4))
  commands.io.enq <> io.instruction
  val completions = Module(new Queue(new GemmCompletion, 2))
  io.completion <> completions.io.deq
  val idle :: decode :: clear :: run :: storeWait :: store :: seedWait :: seed :: finish :: Nil =
    Enum(9)
  val state = RegInit(idle)
  io.phase := state
  val cmd = Reg(new GemmFields)
  val error = RegInit(false.B)
  val initialized = RegInit(0.U(4.W))
  val ctx = RegInit(0.U(2.W))
  val fetch = RegInit(0.U(11.W))
  val issue = RegInit(0.U(11.W))
  val rowRequest = RegInit(0.U(log2Ceil(4 * t + 1).W))
  val rowCommit = RegInit(0.U(log2Ceil(4 * t + 1).W))
  val allIdle = panel.flatten.map(_.io.idle).reduce(_ && _)
  val hostPending = RegInit(false.B)
  val hostBank = Reg(UInt(4.W))

  // The unified resource contract has a two-entry registered input queue and
  // a non-bypassing row-response queue. These status outputs come only from
  // resident state. In a GEMM phase no other panel operation is requested, so
  // they reproduce the selected port's ready without traversing its request
  // mux, same-cycle input-valid checks, or read/clear/seed priority chain.
  def contextAvailable(p: RetimedPanelPort, context: UInt): Bool = {
    if (externalResources) p.residentContextIdle.get(context)
    else p.contextIdle(context)
  }

  def inputAvailable(p: RetimedPanelPort, context: UInt): Bool = {
    if (externalResources) {
      // GEMM emits only OS tokens, out.ready is always true, and ownership
      // switches require a fully idle panel. Therefore no CS retirement can
      // freeze this phase. Do not feed inactive GDN output backpressure into
      // GEMM admission; verify the invariant on the physical resource below.
      p.inputOccupancy < 2.U && p.osAdmissible.get(context)
    } else p.in.ready
  }

  def readAvailable(p: RetimedPanelPort, context: UInt): Bool = {
    if (externalResources) contextAvailable(p, context) && p.rowReadAvailable.get
    else p.read.ready
  }

  commands.io.deq.ready := state === idle && !hostPending && !io.hostRead.valid
  when(commands.io.deq.fire) {
    cmd := GemmFields.decode(commands.io.deq.bits)
    state := decode
    error := false.B
  }
  completions.io.enq.valid := state === finish
  completions.io.enq.bits.tile := tileId.U
  completions.io.enq.bits.tag := cmd.tag
  completions.io.enq.bits.opcode := cmd.opcode
  completions.io.enq.bits.error := error
  when(completions.io.enq.fire) {
    state := idle
  }
  val selectedIdle = (0 until 4)
    .map { c =>
      !cmd.contextMask(c) || panel.flatten.map(_.io.contextIdle(c)).reduce(_ && _)
    }
    .reduce(_ && _)

  when(state === decode) {
    val invalidHeader = cmd.reserved.orR || cmd.opcode > 4.U || !cmd.contextMask.orR ||
      !cmd.tileMask(tileId)
    val invalidShape = cmd.mValid === 0.U || cmd.mValid > (2 * t).U || cmd.nValid === 0.U ||
      cmd.nValid > (2 * t).U
    val invalidMultiply = cmd.opcode === 1.U && (
      cmd.contextMask =/= 15.U || initialized =/= 15.U || cmd.kCount === 0.U ||
        cmd.kCount > 256.U ||
        (cmd.inputBase +& (cmd.kCount << 1)) > operandDepth.U
    )
    val invalidResultRange =
      (cmd.opcode === 2.U || cmd.opcode === 4.U) && (cmd.outputBase +& (4 * t).U) > 512.U
    val invalidStoreContext =
      cmd.opcode === 2.U && (initialized & cmd.contextMask) =/= cmd.contextMask
    when(
      invalidHeader || invalidShape || invalidMultiply || invalidResultRange || invalidStoreContext
    ) {
      error := true.B
      state := finish
    }.otherwise {
      ctx := 0.U
      fetch := 0.U
      issue := 0.U
      rowRequest := 0.U
      rowCommit := 0.U
      switch(cmd.opcode) {
        is(0.U) { state := clear }
        is(1.U) { state := run }
        is(2.U) { state := storeWait }
        is(3.U) { state := storeWait }
        is(4.U) { state := seedWait }
      }
    }
  }

  // Port defaults. Every GEMM write covers its complete physical word;
  // keeping the mask constant also avoids a STORE-state fanout to 256 bits.
  for (p <- panel.flatten) {
    p.io.seedColumn.foreach(_ := false.B)
    p.io.in.valid := false.B
    p.io.in.bits := 0.U.asTypeOf(p.io.in.bits)
    p.io.coefficient.valid := false.B
    p.io.coefficient.bits := 0.U.asTypeOf(p.io.coefficient.bits)
    p.io.seed.valid := false.B
    p.io.seed.bits := 0.U.asTypeOf(p.io.seed.bits)
    p.io.clear.valid := false.B
    p.io.clear.bits := ctx
    p.io.read.valid := false.B
    p.io.read.bits := 0.U.asTypeOf(p.io.read.bits)
    p.io.response.ready := false.B
    p.io.out.ready := true.B
  }
  for (b <- 0 until 16) {
    operand.io.read(b).valid := false.B
    operand.io.read(b).bits := 0.U
    operand.io.response(b).ready := false.B
    operand.io.write(b).valid := false.B
    operand.io.write(b).bits := 0.U.asTypeOf(operand.io.write(b).bits)
    work.io.read(b).valid := false.B
    work.io.read(b).bits := 0.U
    work.io.response(b).ready := false.B
    work.io.write(b).valid := false.B
    work.io.write(b).bits := 0.U.asTypeOf(work.io.write(b).bits)
    work.io.write(b).bits.mask := "hffff".U
  }

  // Four physical payload ports per tile, 256 bits each; no hidden host bus.
  for (p <- 0 until 4) {
    val active = state === run && io.load(p).bits.address >= cmd.inputBase &&
      io.load(p).bits.address < (cmd.inputBase +& (cmd.kCount << 1))
    io.load(p).ready := !active
    for (g <- 0 until 2; l <- 0 until 2) {
      val b = g * 8 + p * 2 + l
      operand.io.write(b).valid := io.load(p).valid && !active && io.load(p).bits.group === g.U
      operand.io.write(b).bits.address := io.load(p).bits.address
      operand.io.write(b).bits.data := io.load(p).bits.data(128 * l + 127, 128 * l)
      operand.io.write(b).bits.mask := "hffff".U
    }
  }

  val clearReady = panel.flatten
    .map { p =>
      if (externalResources) contextAvailable(p.io, ctx) else p.io.clear.ready
    }
    .reduce(_ && _)
  when(state === clear) {
    for (p <- panel.flatten) {
      when(state === run) {
        assert(p.io.advance && !p.io.out.valid, "GEMM owns an OS-only advancing panel")
      }
      p.io.clear.valid := cmd.contextMask(ctx) && clearReady
    }
    when(!cmd.contextMask(ctx) || clearReady) {
      when(cmd.contextMask(ctx)) {
        initialized := initialized | (1.U(4.W) << ctx)
      }
      when(ctx === 3.U) { state := finish }
        .otherwise { ctx := ctx + 1.U }
    }
  }

  // Operand reads and four-context wave assembly keep their original queues.
  val tags = Module(new Queue(UInt(2.W), 4))
  val waves = Module(new Queue(new GemmWave(t), 4))
  val admission = Module(
    new FixedBankReadAdmission(2 * operandWords, registeredPending = externalResources)
  )
  val pendingFetch = Reg(UInt(11.W))
  admission.io.offer := state === run && fetch < (cmd.kCount << 2)
  admission.io.room := tags.io.enq.ready
  val doFetch = admission.io.admit
  val requestFetch = Mux(admission.io.pending, pendingFetch, fetch)
  tags.io.enq.valid := doFetch
  tags.io.enq.bits := fetch(1, 0)
  for (b <- 0 until 2 * operandWords) {
    admission.io.bankReady(b) := operand.io.read(b).ready
    operand.io.read(b).valid := admission.io.bankValid(b)
    operand.io.read(b).bits := cmd.inputBase + (requestFetch >> 2) * 2.U +
      (if (b < operandWords) requestFetch(1) else requestFetch(0))
  }
  when(doFetch) {
    pendingFetch := fetch
    fetch := fetch + 1.U
  }
  when(admission.io.pending) {
    assert(state === run, "GEMM cannot leave run with unfinished bank reads")
    assert(tags.io.deq.valid, "partial GEMM row needs its original tag reservation")
  }
  val responsesReady = (0 until 2 * operandWords)
    .map(b => operand.io.response(b).valid)
    .reduce(_ && _)
  waves.io.enq.valid := tags.io.deq.valid && responsesReady
  waves.io.enq.bits.context := tags.io.deq.bits
  for (i <- 0 until t) {
    val a = operand.io.response(i / 8).bits(16 * (i % 8) + 15, 16 * (i % 8))
    val b = operand.io.response(operandWords + i / 8).bits(16 * (i % 8) + 15, 16 * (i % 8))
    waves.io.enq.bits.a(i) := Mux((tags.io.deq.bits(1) * t.U + i.U) < cmd.mValid, a, 0.U)
    waves.io.enq.bits.b(i) := Mux((tags.io.deq.bits(0) * t.U + i.U) < cmd.nValid, b, 0.U)
  }
  tags.io.deq.ready := waves.io.enq.ready && responsesReady
  for (b <- 0 until 2 * operandWords) {
    operand.io.response(b).ready := waves.io.enq.ready && tags.io.deq.valid && responsesReady
  }
  val canIssue =
    panel.flatten.map(p => inputAvailable(p.io, waves.io.deq.bits.context)).reduce(_ && _)
  for (r <- 0 until 4; c <- 0 until 4) {
    val p = panel(r)(c)
    p.io.in.valid := state === run && waves.io.deq.valid && canIssue
    p.io.in.bits.os := true.B
    p.io.in.bits.context := waves.io.deq.bits.context
    p.io.in.bits.tag := waves.io.deq.bits.context
    for (i <- 0 until side) {
      p.io.in.bits.west(i) := waves.io.deq.bits.a(r * side + i)
      p.io.in.bits.north(i) := waves.io.deq.bits.b(c * side + i)
    }
  }
  waves.io.deq.ready := state === run && canIssue
  io.issued := waves.io.deq.fire
  when(waves.io.deq.fire) {
    issue := issue + 1.U
    when(issue + 1.U === (cmd.kCount << 2)) { state := finish }
  }
  when(state === storeWait && selectedIdle) {
    when(cmd.opcode === 3.U) { state := finish }.otherwise { state := store }
  }
  when(state === seedWait && selectedIdle) { state := seed }

  // Rotate requests through the four panel-row groups. Each group retains its
  // local row/context/address and has four cycles between requests, avoiding
  // the one-entry response FIFO's consecutive-read bubble without new storage.
  val rowTags = Module(new Queue(new GemmRowTag(side), 4))
  val requestTag = Wire(new GemmRowTag(side))
  requestTag.context := (rowRequest >> log2Ceil(t))(1, 0)
  requestTag.group := rowRequest(1, 0)
  requestTag.row := rowRequest(log2Ceil(side) + 1, 2)
  val requestAddress = cmd.outputBase +
    ((requestTag.context(1) * t.U + requestTag.group * side.U + requestTag.row) << 1) +
    requestTag.context(0)
  rowTags.io.enq.valid := false.B
  rowTags.io.enq.bits := requestTag
  rowTags.io.deq.ready := false.B
  val tag = rowTags.io.deq.bits
  val address =
    cmd.outputBase + ((tag.context(1) * t.U + tag.group * side.U + tag.row) << 1) + tag.context(0)
  io.committed.valid := false.B
  io.committed.bits.address := address
  io.committed.bits.values := 0.U.asTypeOf(io.committed.bits.values)
  val needRow = rowRequest < (4 * t).U
  val chosen = cmd.contextMask(requestTag.context)
  val skipped = (state === store || state === seed) && needRow && !chosen
  when(skipped) {
    rowRequest := rowRequest + 1.U
    rowCommit := rowCommit + 1.U
  }
  val panelReadReady = (0 until 4)
    .map { g =>
      requestTag.group =/= g.U || panel(g)
        .map(p => readAvailable(p.io, requestTag.context))
        .reduce(_ && _)
    }
    .reduce(_ && _)
  val takeRead = state === store && needRow && chosen && rowTags.io.enq.ready && panelReadReady
  for (g <- 0 until 4; c <- 0 until 4) {
    panel(g)(c).io.read.bits.context := requestTag.context
    panel(g)(c).io.read.bits.row := requestTag.row
    panel(g)(c).io.read.valid := takeRead && requestTag.group === g.U
  }
  val panelResponseValid = (0 until 4)
    .map { g =>
      tag.group =/= g.U || panel(g).map(_.io.response.valid).reduce(_ && _)
    }
    .reduce(_ && _)
  val storeFire = state === store && rowTags.io.deq.valid && panelResponseValid && io.storeEnable
  val rowValues = Wire(Vec(t, UInt(32.W)))
  for (i <- 0 until t) {
    rowValues(i) := Mux1H((0 until 4).map { g =>
      (tag.group === g.U) -> panel(g)(i / side).io.response.bits.values(i % side)
    })
  }
  when(state === store) {
    rowTags.io.enq.valid := takeRead
    rowTags.io.deq.ready := panelResponseValid && io.storeEnable
    for (g <- 0 until 4; c <- 0 until 4) {
      panel(g)(c).io.response.ready := storeFire && tag.group === g.U
    }
    for (b <- 0 until resultWords) {
      work.io.write(b).valid := storeFire
      work.io.write(b).bits.address := address
      work.io.write(b).bits.data := Cat((0 until 4).reverse.map(l => rowValues(4 * b + l)))
    }
    io.committed.valid := storeFire
    io.committed.bits.values := rowValues
  }

  // Reserve a row in the original four-entry tag queue before any physical
  // bank read. Each bank may accept independently, without its ready signal
  // controlling another bank's valid. Only an address and remaining mask are
  // retained; response payloads stay in the existing SRAM/owner queues.
  val seedOffer = state === seed && needRow && chosen
  val seedRead = Wire(Bool())
  val seedBankValid = Wire(Vec(resultWords, Bool()))
  val seedAddress = Wire(UInt(9.W))
  if (independentSeedReads) {
    val seedAdmission = Module(new FixedBankReadAdmission(resultWords))
    val heldSeedAddress = Reg(UInt(9.W))
    seedAdmission.io.offer := seedOffer
    seedAdmission.io.room := rowTags.io.enq.ready
    for (bank <- 0 until resultWords) {
      seedAdmission.io.bankReady(bank) := work.io.read(bank).ready
    }
    seedRead := seedAdmission.io.admit
    seedBankValid := seedAdmission.io.bankValid
    seedAddress := Mux(seedAdmission.io.pending, heldSeedAddress, requestAddress)
    when(seedRead) { heldSeedAddress := requestAddress }
    when(seedAdmission.io.pending) {
      assert(
        state === seed && rowTags.io.deq.valid,
        "pending GEMM seed reads must retain their reserved row tag"
      )
    }
  } else {
    val seedReadReady = work.io.read.take(resultWords).map(_.ready).reduce(_ && _)
    seedRead := seedOffer && rowTags.io.enq.ready && seedReadReady
    seedBankValid.foreach(_ := seedRead)
    seedAddress := requestAddress
  }
  val seedResponse = (0 until resultWords).map(b => work.io.response(b).valid).reduce(_ && _)
  val seedReady = (0 until 4)
    .map { g =>
      tag.group =/= g.U || panel(g)
        .map { p =>
          if (externalResources) contextAvailable(p.io, tag.context) else p.io.seed.ready
        }
        .reduce(_ && _)
    }
    .reduce(_ && _)
  val seedFire = state === seed && rowTags.io.deq.valid && seedResponse && seedReady
  when(state === seed) {
    rowTags.io.enq.valid := seedRead
    rowTags.io.deq.ready := seedResponse && seedReady
    for (b <- 0 until resultWords) {
      work.io.read(b).valid := seedBankValid(b)
      work.io.read(b).bits := seedAddress
      work.io.response(b).ready := seedFire
    }
    for (g <- 0 until 4; c <- 0 until 4) {
      val p = panel(g)(c)
      p.io.seed.valid := seedFire && tag.group === g.U
      p.io.seed.bits.context := tag.context
      p.io.seed.bits.row := tag.row
      for (i <- 0 until side) {
        val lane = c * side + i
        p.io.seed.bits
          .values(i) := work.io.response(lane / 4).bits(32 * (lane % 4) + 31, 32 * (lane % 4))
      }
    }
  }
  when(takeRead || seedRead) { rowRequest := rowRequest + 1.U }
  // A skipped context and an older row return can occur on the same edge.
  when(storeFire || seedFire) { rowCommit := rowCommit + 1.U + skipped.asUInt }
  when((state === store || state === seed) && rowCommit === (4 * t).U) {
    when(state === seed) { initialized := initialized | cmd.contextMask }
    state := finish
  }

  if (externalResources) {
    // Enforce the ownership proof at actual handshakes, including arbitrary
    // stalls. These are verification checks and are removed for synthesis.
    for (p <- panel.flatten) {
      when(p.io.in.valid) { assert(p.io.in.ready, "GEMM input reservation mismatch") }
      when(p.io.read.valid) { assert(p.io.read.ready, "GEMM row reservation mismatch") }
      when(p.io.clear.valid) { assert(p.io.clear.ready, "GEMM clear reservation mismatch") }
      when(p.io.seed.valid) { assert(p.io.seed.ready, "GEMM seed reservation mismatch") }
    }
  }

  io.hostRead.ready := state === idle && !commands.io.deq.valid && allIdle && !hostPending &&
    work.io.read(io.hostRead.bits.bank).ready
  io.hostResponse.valid := hostPending && work.io.response(hostBank).valid
  io.hostResponse.bits := work.io.response(hostBank).bits
  when(io.hostRead.fire) {
    hostPending := true.B
    hostBank := io.hostRead.bits.bank
  }
  when(io.hostResponse.fire) { hostPending := false.B }
  for (b <- 0 until 16) {
    when(state === idle) {
      work.io
        .read(b)
        .valid := io.hostRead.valid && io.hostRead.ready && io.hostRead.bits.bank === b.U
      work.io.read(b).bits := io.hostRead.bits.address
      work.io.response(b).ready := hostPending && hostBank === b.U && io.hostResponse.ready
    }
  }
  io.inputsConsumed.foreach(_ := state === idle && !commands.io.deq.valid && !hostPending)
  io.inputLease.foreach { lease =>
    lease.valid := state === run
    lease.begin := cmd.inputBase
    lease.end := cmd.inputBase +& (cmd.kCount << 1)
  }
  io.invalidateContexts.foreach { invalidate =>
    when(invalidate) {
      assert(state === idle && !commands.io.deq.valid && allIdle && !hostPending)
      initialized := 0.U
    }
  }
  io.idle := state === idle && !commands.io.deq.valid && allIdle && !hostPending
}
