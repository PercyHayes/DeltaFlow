package deltaflow.system
import chisel3._
import chisel3.util._
import chisel3.experimental.hierarchy._
import deltaflow.frontend._
import deltaflow.memory._
import deltaflow.sa._
import deltaflow.isa.GemmCompletion
import deltaflow.dma._

/** One physical array and one Operand/Work store, selected by instruction. The controllers are
  * external-resource clients of this shared hardware. Both modes overlap DMA using bounded
  * input/output slot leases. Future GDN preparation uses the same panels and fixed SRAM as state
  * work.
  */
class UnifiedComputeTile(val side: Int = 16, val tileId: Int = 0, val compactPe: Boolean = false)
    extends Module {
  val t = 4 * side
  val io = IO(new Bundle {
    val instruction = Flipped(Decoupled(UInt(128.W)))
    val completion = Decoupled(new GemmCompletion)
    val dma = Vec(4, Flipped(new DmaLocalPort))
    val dmaIdle = Input(Bool())
    val idle = Output(Bool())
    val buffersIdle = Output(Bool())
    val gdnMode = Output(Bool())
    val issued = Output(UInt(16.W))
  })
  val gemm = Module(new GemmInstructionTile(side, tileId, externalResources = true))
  val recurrent = Module(new RecurrentStateEngine(side))
  val gdn = Module(
    new ReadyTransformTile(
      side,
      registerIssue = true,
      withProjection = true,
      jointProjection = true,
      withStateOutput = true,
      returnFlow = true,
      eagerMulticast = true,
      chunkLookahead = true,
      compactOutputs = true,
      streamedStateOutput = true,
      stateEpilogue = true,
      earlyFullGram = true,
      preparationLookahead = true,
      mixedLookahead = true,
      residentProjection = true,
      pipelinedProjection = true,
      externalResources = true,
      rollingLookahead = true
    )
  )
  val definition = Definition(
    new RetimedWavePanel(
      side,
      rowResponseDepth = 1,
      registerInput = true,
      columnSeed = true,
      osAdmissionStatus = true,
      registerClear = true,
      compactWestMetadata = compactPe,
      sharedMagnitude = compactPe,
      freeInvalidPayload = compactPe,
      packedAligned = compactPe,
      compactClassification = compactPe,
      splitAlignment = compactPe,
      narrowAddition = compactPe,
      freeForwardPayload = compactPe
    )
  )
  val panels = Seq.fill(16)(Instance(definition))
  val operand = Module(
    new MacroScratchpad(16, 1024, registeredAdmission = true, earlyWriteConflict = true)
  )
  val work = Module(
    new MacroScratchpad(16, 512, registeredAdmission = true, earlyWriteConflict = true)
  )
  val mode = RegInit(false.B)
  io.gdnMode := mode
  val recurrentMode = RegInit(false.B)
  gdn.io.recurrentEnable.get := recurrentMode
  gdn.io.recurrentScale.get <> recurrent.io.scale
  val commands = Module(
    new RegisteredInstructionQueue(independentTailWrite = true, predecodeSide = Some(side))
  )
  commands.io.enq <> io.instruction
  val localCompletions = Module(new Queue(new GemmCompletion, 2))
  val completions = Module(new RRArbiter(new GemmCompletion, 2))
  completions.io.in(0) <> gemm.io.completion
  completions.io.in(1) <> localCompletions.io.deq
  io.completion <> completions.io.out
  val gateReader = Module(new GateBufferReader(side))
  val command = commands.io.deq.bits
  val opcode = command(7, 0)
  val slot = command(48)
  val prepared = command(49)
  val running = RegInit(false.B)
  val done = RegInit(false.B)
  val activeSlot = Reg(Bool())
  val hasState = RegInit(false.B)
  val reserveNext = RegInit(false.B)
  val futureRawReady = RegInit(false.B)
  val activeTag = Reg(UInt(16.W))
  val activeOpcode = Reg(UInt(8.W))
  val completionError = RegInit(false.B)
  val keyColumns = Reg(UInt(8.W))
  val valueColumns = Reg(UInt(8.W))
  val inputLive = RegInit(0.U(2.W))
  val outputLive = RegInit(0.U(2.W))
  val outputReady = RegInit(0.U(2.W))
  val futureLive = RegInit(false.B)
  val futureSlot = Reg(Bool())
  val readingGates = RegInit(false.B)
  val gatesSent = RegInit(false.B)
  val prepareSent = RegInit(false.B)
  val memoryIdle = Wire(Bool())
  val quiescent = gemm.io.idle && !running && !done && !gdn.io.busy && !futureLive &&
    gateReader.io.idle && !prepareSent && recurrent.io.idle && io.dmaIdle && memoryIdle &&
    panels
      .map(_.io.idle)
      .reduce(_ && _)
  val wantsGdn = opcode >= 33.U && opcode <= 35.U
  val wantsRecurrent = opcode === 36.U || opcode === 37.U
  val switchMode =
    commands.io.deq.valid && (mode =/= wantsGdn || recurrentMode =/= wantsRecurrent) && quiescent &&
      !outputLive.orR
  gemm.io.invalidateContexts.get := switchMode
  when(switchMode) {
    mode := wantsGdn
    recurrentMode := wantsRecurrent
    hasState := false.B
  }
  val selected = mode === wantsGdn && recurrentMode === wantsRecurrent
  val compute = opcode <= 4.U
  val run = opcode === 33.U
  val prepare = opcode === 34.U
  val release = opcode === 35.U
  val dims =
    command(39, 32) >= 1.U && command(39, 32) <= (2 * t).U && command(47, 40) >= 1.U && command(
      47,
      40
    ) <= (2 * t).U
  val reserved = command(127, 51).orR || command(15, 12).orR
  val runLegal = dims && !reserved &&
    (!prepared ||
      futureLive &&
      slot === futureSlot && command(39, 32) === keyColumns && command(47, 40) === valueColumns)
  val prepLegal = hasState && dims && !reserved && !command(50) && !prepared && command(
    39,
    32
  ) === keyColumns && command(47, 40) === valueColumns && slot =/= activeSlot
  val releaseLegal = !reserved && !command(50) && !prepared && !command(47, 32).orR
  val recurrentLegal = dims && !command(127, 64).orR && !command(15, 12).orR &&
    command(54, 48) > 0.U && command(54, 48) <= 64.U &&
    (command(63, 55) +&
      (command(
        39,
        32
      ) << 1)) <= 512.U
  val originalBad = !compute && Mux(
    wantsRecurrent,
    !recurrentLegal,
    !run && !prepare && !release || run && !runLegal || prepare && !prepLegal ||
      release && !releaseLegal
  )
  // Instruction-only checks were captured with this exact queue entry.
  // Resource ownership and dimensions of a prepared chunk remain live checks.
  val preparedMatches = futureLive && slot === futureSlot && command(39, 32) === keyColumns &&
    command(47, 40) === valueColumns
  val runStateLegal = !prepared || preparedMatches
  val prepareStateLegal = hasState && slot =/= activeSlot && command(39, 32) === keyColumns &&
    command(47, 40) === valueColumns
  val bad = commands.io.staticIllegal.get || (run && !runStateLegal) ||
    (prepare && !prepareStateLegal)
  when(commands.io.deq.valid) {
    assert(
      bad === originalBad,
      "predecoded instruction must preserve static and live legality checks"
    )
  }
  // Confirm the selected GDN head before it can affect SRAM clients. GEMM
  // bypasses this stage. Readiness and slot leases below remain live checks.
  val commandCheck = Module(new GdnCommandCheck)
  commandCheck.io.eligible := commands.io.deq.valid && !compute && selected
  commandCheck.io.invalidHead := !commands.io.deq.valid
  commandCheck.io.consumed := commands.io.deq.fire
  commandCheck.io.modeChanged := switchMode
  commandCheck.io.bad := bad
  recurrent.io.start.valid := false.B
  recurrent.io.start.bits.keys := command(39, 32)
  recurrent.io.start.bits.values := command(47, 40)
  recurrent.io.start.bits.tokens := command(54, 48)
  recurrent.io.start.bits.stateBase := command(63, 55)
  recurrent.io.start.bits.rebuild := opcode === 37.U
  commands.io.deq.ready := false.B
  gemm.io.instruction.valid := commands.io.deq.valid && compute && selected
  gemm.io.instruction.bits := command
  when(compute && selected) { commands.io.deq.ready := gemm.io.instruction.ready }
  gemm.io.storeEnable := true.B
  gemm.io.hostRead.valid := false.B
  gemm.io.hostRead.bits := 0.U.asTypeOf(gemm.io.hostRead.bits)
  gemm.io.hostResponse.ready := true.B
  for (p <- gemm.io.load) {
    p.valid := false.B
    p.bits := 0.U.asTypeOf(p.bits)
  }
  localCompletions.io.enq.valid := done
  localCompletions.io.enq.bits.tile := tileId.U
  localCompletions.io.enq.bits.tag := Mux(done, activeTag, command(31, 16))
  localCompletions.io.enq.bits.opcode := Mux(done, activeOpcode, opcode)
  localCompletions.io.enq.bits.error := done && completionError
  when(done && localCompletions.io.enq.fire) { done := false.B }
  when(gdn.io.done) {
    assert(running && !done)
    running := false.B
    done := true.B
    inputLive := inputLive & ~(1.U(2.W) << activeSlot)
    outputReady := outputReady | (1.U(2.W) << activeSlot)
  }
  when(recurrent.io.done) {
    assert(!done)
    done := true.B
    completionError := recurrent.io.error
  }
  val proj = gdn.io.projection.get
  val joint = gdn.io.joint.get
  val out = gdn.io.output.get
  val look = gdn.io.lookahead.get
  gateReader.io.start.valid := false.B
  gateReader.io.start.bits := slot
  gateReader.io.result.ready := false.B
  gdn.io.gates.valid := false.B
  gdn.io.gates.bits := gateReader.io.result.bits
  look.gates.get.valid := false.B
  look.gates.get.bits := gateReader.io.result.bits
  look.prepare.valid := running && reserveNext && !prepareSent && !futureLive
  look.prepare.bits := keyColumns
  look.rawReady.get := futureRawReady
  when(look.prepare.fire) { prepareSent := true.B }
  look.inputSlot := slot
  look.usePrepared := prepared
  look.overlap := true.B
  look.readPreviousOutput := false.B
  gdn.io.start.valid := false.B
  gdn.io.start.bits.keyColumns := command(39, 32)
  gdn.io.start.bits.valueColumns := command(47, 40)
  // The confirmation was captured only for a selected non-GEMM head. That
  // head cannot change without consumption, which clears the confirmation.
  // Reusing this admission avoids a second mode decode on the SRAM path.
  when(commands.io.deq.valid && commandCheck.io.checked) {
    assert(!compute && selected, "confirmed GDN head must retain its selected mode owner")
  }
  val commandEligible = commands.io.deq.valid && !compute && commandCheck.io.checked && !done &&
    !gdn.io.done
  when(commandEligible) {
    when(commandCheck.io.checkedBad) {
      localCompletions.io.enq.valid := true.B
      localCompletions.io.enq.bits.error := true.B
      commands.io.deq.ready := localCompletions.io.enq.ready
    }.elsewhen(wantsRecurrent) {
      // Decode mutually exclusive opcodes before testing their resource
      // readiness. A stalled native command cannot be a prefill command;
      // its full-system quiescence chain must not gate prefill/DMA requests.
      when(quiescent) {
        recurrent.io.start.valid := true.B
        commands.io.deq.ready := recurrent.io.start.ready
        when(recurrent.io.start.fire) {
          activeTag := command(31, 16)
          activeOpcode := opcode
          completionError := false.B
        }
      }
    }.elsewhen(release) {
      when(io.dmaIdle && outputReady(slot)) {
        localCompletions.io.enq.valid := true.B
        commands.io.deq.ready := localCompletions.io.enq.ready
        when(commands.io.deq.fire) {
          outputLive := outputLive & ~(1.U(2.W) << slot)
          outputReady := outputReady & ~(1.U(2.W) << slot)
        }
      }
    }.elsewhen(prepare) {
      when(!futureLive) {
        look.prepare.valid := !prepareSent
        look.prepare.bits := command(39, 32)
        futureRawReady := true.B
        inputLive := inputLive | (1.U(2.W) << slot)
        gateReader.io.start.valid := !readingGates && !gatesSent
        when(gateReader.io.start.fire) { readingGates := true.B }
        look.gates.get.valid := prepareSent && gateReader.io.result.valid && !gateReader.io.error
        gateReader.io.result.ready := look.gates.get.ready && prepareSent
        when(look.gates.get.fire) {
          // mayConfigure in ReadyTransformTile proves current Gram/RHS/query and
          // raw-K N reads retired. H/O use derived pages; release only raw input,
          // preserving the independently owned output slot until AXI B.
          gatesSent := true.B
          inputLive := inputLive & ~(1.U(2.W) << activeSlot)
        }
        when(gatesSent) {
          localCompletions.io.enq.valid := true.B
          commands.io.deq.ready := localCompletions.io.enq.ready
          when(commands.io.deq.fire) {
            futureLive := true.B
            futureSlot := slot
            readingGates := false.B
            gatesSent := false.B
          }
        }
      }
    }.elsewhen(run) {
      when(!running && !outputLive(slot)) {
        gateReader.io.start.valid := !prepared && !readingGates && !gatesSent
        when(gateReader.io.start.fire) { readingGates := true.B }
        gdn.io.gates.valid := !prepared && gateReader.io.result.valid && !gateReader.io.error
        gateReader.io.result.ready := gdn.io.gates.ready && !prepared
        when(gdn.io.gates.fire) { gatesSent := true.B }
        gdn.io.start.valid := prepared || gatesSent
        commands.io.deq.ready := gdn.io.start.ready && (prepared || gatesSent)
        when(gdn.io.start.fire) {
          hasState := true.B
          running := true.B
          activeSlot := slot
          reserveNext := command(50)
          futureRawReady := false.B
          prepareSent := false.B
          activeTag := command(31, 16)
          activeOpcode := 33.U
          completionError := false.B
          keyColumns := command(39, 32)
          valueColumns := command(47, 40)
          inputLive := inputLive | (1.U(2.W) << slot)
          outputLive := outputLive | (1.U(2.W) << slot)
          when(prepared) { futureLive := false.B }
          readingGates := false.B
          gatesSent := false.B
        }
      }
    }
    when(gateReader.io.result.valid && gateReader.io.error) {
      gdn.io.gates.valid := false.B
      look.gates.get.valid := false.B
      gdn.io.start.valid := false.B
      localCompletions.io.enq.valid := true.B
      localCompletions.io.enq.bits.error := true.B
      commands.io.deq.ready := localCompletions.io.enq.ready
      gateReader.io.result.ready := localCompletions.io.enq.ready
      when(commands.io.deq.fire) {
        readingGates := false.B
        gatesSent := false.B
      }
    }
  }
  gdn.io.bankEnable := 65535.U
  gdn.io.jobEnable := ((1 << 20) - 1).U
  gdn.io.transformEnable := true.B
  for (p <- gdn.io.result) p.ready := true.B
  proj.enable := true.B
  proj.result.ready := true.B
  joint.result.ready := true.B
  for (p <- gdn.io.init ++ proj.initH ++ look.init) {
    p.valid := false.B
    p.bits := 0.U.asTypeOf(p.bits)
  }
  for (
    (r, s) <- Seq(
      (gdn.io.hostRead, gdn.io.hostResponse),
      (joint.hostRead, joint.hostResponse),
      (out.hostRead, out.hostResponse)
    )
  ) {
    r.valid := false.B
    r.bits := 0.U
    s.ready := true.B
  }

  // Requests and responses share a registered mode owner. A mode switch is
  // permitted only after commands, arithmetic, memory and DMA have retired.
  def choose[T <: Data](
      physical: DecoupledIO[T],
      gemmPort: DecoupledIO[T],
      gdnPort: DecoupledIO[T]
  ): Unit = {
    physical.valid := Mux(mode, gdnPort.valid, gemmPort.valid)
    physical.bits := Mux(mode, gdnPort.bits, gemmPort.bits)
    gemmPort.ready := physical.ready && !mode
    gdnPort.ready := physical.ready && mode
  }
  def reply[T <: Data](
      physical: DecoupledIO[T],
      gemmPort: DecoupledIO[T],
      gdnPort: DecoupledIO[T]
  ): Unit = {
    physical.ready := Mux(mode, gdnPort.ready, gemmPort.ready)
    gemmPort.bits := physical.bits
    gdnPort.bits := physical.bits
    gemmPort.valid := physical.valid && !mode
    gdnPort.valid := physical.valid && mode
  }
  val gemmResources = gemm.io.resources.get
  val gdnResources = gdn.io.resources.get
  for (i <- 0 until 16) {
    val physical = panels(i).io
    val gemmPanel = gemmResources.panels(i)
    val gdnPanel = gdnResources.panels(i)
    choose(physical.in, gemmPanel.in, gdnPanel.in)
    choose(physical.coefficient, gemmPanel.coefficient, gdnPanel.coefficient)
    choose(physical.seed, gemmPanel.seed, gdnPanel.seed)
    choose(physical.clear, gemmPanel.clear, gdnPanel.clear)
    choose(physical.read, gemmPanel.read, gdnPanel.read)
    reply(physical.out, gemmPanel.out, gdnPanel.out)
    reply(physical.response, gemmPanel.response, gdnPanel.response)
    physical.seedColumn.get := Mux(mode, gdnPanel.seedColumn.get, gemmPanel.seedColumn.get)
    for (client <- Seq(gemmPanel, gdnPanel)) {
      client.contextIdle := physical.contextIdle
      client.csIdle := physical.csIdle
      client.idle := physical.idle
      client.configured := physical.configured
      client.advance := physical.advance
      client.inputOccupancy := physical.inputOccupancy
      client.osAdmissible.get := physical.osAdmissible.get
      client.residentContextIdle.get := physical.residentContextIdle.get
      client.rowReadAvailable.get := physical.rowReadAvailable.get
      client.dispatched.bits := physical.dispatched.bits
    }
    gemmPanel.dispatched.valid := physical.dispatched.valid && !mode
    gdnPanel.dispatched.valid := physical.dispatched.valid && mode
  }
  io.issued := VecInit(panels.map(_.io.dispatched.valid)).asUInt
  val computePorts = Seq(
    Wire(new ScratchpadPort(16, 1024)),
    Wire(new ScratchpadPort(16, 512))
  )
  val memoryClients = computePorts
    .zip(Seq(gemmResources.operand, gemmResources.work))
    .zip(Seq(gdnResources.operand, gdnResources.work))
  for (((compute, gemmPort), gdnPort) <- memoryClients) {
    for (i <- 0 until 16) {
      choose(compute.read(i), gemmPort.read(i), gdnPort.read(i))
      choose(compute.write(i), gemmPort.write(i), gdnPort.write(i))
      reply(compute.response(i), gemmPort.response(i), gdnPort.response(i))
    }
  }
  for ((c, n) <- computePorts.zip(Seq(recurrent.io.operand, recurrent.io.work)); i <- 0 until 16) {
    n.read(i).ready := c.read(i).ready && recurrentMode
    n.write(i).ready := c.write(i).ready && recurrentMode
    n.response(i).valid := c.response(i).valid && recurrentMode
    n.response(i).bits := c.response(i).bits
    when(recurrentMode) {
      c.read(i).valid := n.read(i).valid
      c.read(i).bits := n.read(i).bits
      c.write(i).valid := n.write(i).valid
      c.write(i).bits := n.write(i).bits
      c.response(i).ready := n.response(i).ready
    }
  }
  when(recurrentMode) {
    val inactivePorts =
      Seq(gemmResources.operand, gemmResources.work, gdnResources.operand, gdnResources.work)
    for (port <- inactivePorts; i <- 0 until 16) {
      port.read(i).ready := false.B
      port.write(i).ready := false.B
      port.response(i).valid := false.B
    }
  }
  val arbs = (0 until 2).map(r =>
    (0 until 16).map(i =>
      Module(
        new QueuedReadArbiter(
          responseDepth = 2,
          clients = if (r == 0 && i >= 8) 3 else 2,
          registeredOwnerReady = true
        )
      )
    )
  )
  val tags = Seq.fill(4)(Module(new Queue(UInt(2.W), 4)))
  for (p <- 0 until 4) {
    val port = io.dma(p)
    val tag = tags(p)
    val region = port.read.bits.region
    val group = port.read.bits.index(0)
    val ready = VecInit(
      (0 until 2).map(r =>
        VecInit(
          (0 until 2).map(h =>
            arbs(r)(8 * h + 2 * p).io.request(1).ready &&
              arbs(r)(8 * h + 2 * p + 1).io
                .request(1)
                .ready
          )
        )(group)
      )
    )(region)
    val readRow = port.read.bits.index >> 1
    val output0 = !region && readRow >= 576.U && readRow < 640.U
    val output1 = !region && readRow >= 704.U && readRow < 768.U
    val readAllowed = !mode || Mux(
      output0,
      outputReady(0),
      Mux(output1, outputReady(1), !running && !futureLive && !prepareSent)
    )
    port.read.ready :=
      readAllowed && (!recurrentMode || recurrent.io.idle) && ready && tag.io.enq.ready
    tag.io.enq.valid := port.read.fire
    tag.io.enq.bits := Cat(region, group)
    val responses = (0 until 2).map(r =>
      (0 until 2).map(h =>
        Seq(arbs(r)(8 * h + 2 * p).io.response(1), arbs(r)(8 * h + 2 * p + 1).io.response(1))
      )
    )
    port.response.valid := tag.io.deq.valid && VecInit(
      responses.flatten.map(v => v.map(_.valid).reduce(_ && _))
    )(tag.io.deq.bits)
    port.response.bits := VecInit(responses.flatten.map(v => Cat(v(1).bits, v(0).bits)))(
      tag.io.deq.bits
    )
    tag.io.deq.ready := port.response.fire
    val wr = port.write.bits.region
    val wg = port.write.bits.index(0)
    val writeAvailable = VecInit((0 until 2).flatMap { r =>
      (0 until 2).map { h =>
        val first = 8 * h + 2 * p
        !computePorts(r).write(first).valid && !computePorts(r).write(first + 1).valid
      }
    })
    val lease = gemm.io.inputLease.get
    val writeRow = port.write.bits.index >> 1
    val activeInput = !mode && !wr && lease.valid && writeRow >= lease.begin && writeRow < lease.end
    val raw0 = !wr && writeRow < 256.U
    val raw1 = !wr && writeRow >= 768.U
    val writeAllowed = !mode || Mux(
      raw0,
      !inputLive(0),
      Mux(raw1, !inputLive(1), !running && !futureLive && !prepareSent)
    )
    val dmaWrite = Module(new DmaLocalWriteAdmission)
    dmaWrite.io.valid := port.write.valid
    dmaWrite.io.permit := writeAllowed && (!recurrentMode || recurrent.io.idle) && !activeInput
    dmaWrite.io.region := wr
    dmaWrite.io.group := wg
    dmaWrite.io.available := writeAvailable
    port.write.ready := dmaWrite.io.ready
    when(port.write.fire) {
      assert(
        dmaWrite.io.accepted.asUInt === UIntToOH(Cat(wr, wg), 4),
        "DMA write must accept exactly its selected two-word target"
      )
    }
    for (r <- 0 until 2; h <- 0 until 2; l <- 0 until 2) {
      val i = 8 * h + 2 * p + l
      val arb = arbs(r)(i)
      val physical = if (r == 0) operand.io else work.io
      val c = computePorts(r)
      arb.io.request(0) <> c.read(i)
      c.response(i) <> arb.io.response(0)
      arb.io.read <> physical.read(i)
      arb.io.returned <> physical.response(i)
      arb.io.shared := port.read.valid || tag.io.deq.valid
      if (r == 0 && i >= 8) {
        arb.io.request(2) <> gateReader.io.read(i - 8)
        gateReader.io.response(i - 8) <> arb.io.response(2)
        arb.io.shared := port.read.valid || tag.io.deq.valid || !gateReader.io.idle
      }
      arb.io.request(1).valid := port.read.fire && region === r.U && group === h.U
      arb.io.request(1).bits := port.read.bits.index >> 1
      arb.io.response(1).ready := port.response.fire && tag.io.deq.bits === (2 * r + h).U
      // Local permission is algebraically the selected port.write.fire, but
      // cannot inherit an unavailable bank from a different SRAM region.
      val dmaAccepted = dmaWrite.io.accepted(2 * r + h)
      val dmaWord = Wire(chiselTypeOf(c.write(i).bits))
      dmaWord.address := port.write.bits.index >> 1
      dmaWord.data := port.write.bits.data(128 * l + 127, 128 * l)
      dmaWord.mask := "hffff".U
      val routedWrite = DmaWritePayload(c.write(i).valid, c.write(i).bits, dmaWord, dmaAccepted)
      physical.write(i).valid := routedWrite.valid
      physical.write(i).bits := routedWrite.bits
      physical.writeConflict.get(i) := DmaWriteConflict(
        c.write(i).valid,
        c.write(i).bits.address,
        dmaAccepted,
        dmaWord.address,
        physical.read(i).bits
      )
      c.write(i).ready := physical.write(i).ready
      when(dmaAccepted) {
        assert(port.write.fire, "SRAM write without an accepted DMA beat")
        assert(physical.write(i).ready)
      }
    }
  }
  memoryIdle := arbs.flatten.map(_.io.idle).reduce(_ && _) &&
    tags
      .map(q => !q.io.deq.valid)
      .reduce(_ && _)
  io.buffersIdle := !commands.io.deq.valid && !running && !done && !futureLive && !prepareSent &&
    gateReader.io.idle && gemm.io.inputsConsumed.get && !gdn.io.busy && recurrent.io.idle &&
    io.dmaIdle && memoryIdle && !io.completion.valid
  io.idle := !commands.io.deq.valid && !running && !done && !futureLive && !prepareSent &&
    gateReader.io.idle && quiescent && !io.completion.valid
}
