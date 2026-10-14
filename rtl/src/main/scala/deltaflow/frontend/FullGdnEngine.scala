package deltaflow.frontend

import chisel3._
import chisel3.util._
import easyfloat.FMA
import deltaflow.arithmetic.{FloatArithmetic, RowInverseController, FloatRow}
import deltaflow.sa.{NativeTile, Panel, StreamPipes}
import deltaflow.memory.{BankedScratchpad, CompactResultWriter}

/** Four-block GDN K/V/activated scalar gates -> U/W. One physical 4x4 tile is reused for OS Gram
  * and CS triangular application. Synchronous K/V memories and output banks have independent
  * token-group pointers. No host L/J/RHS.
  */
class FullGdnEngine(
    val side: Int = 16,
    val keyDim: Int = 128,
    val valueDim: Int = 128,
    val interleavedPreparation: Boolean = false,
    val blockReadyExecution: Boolean = false,
    val compactResults: Boolean = false
) extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  require(keyDim >= 2 && keyDim <= 128 && valueDim >= 2 && valueDim <= 128)
  require(!blockReadyExecution || interleavedPreparation)
  require(!compactResults || (side == 16 && keyDim == 128 && valueDim == 128))
  val resultDepth = if (compactResults) 1024 else keyDim + valueDim
  val tokens = 4 * side
  val width = keyDim + valueDim
  val tagBits = log2Ceil(width + 1)
  val banks = tokens / 4
  val gramCycles = math.max(keyDim + 2 * side, tokens)
  val io = IO(new Bundle {
    val key = Flipped(Decoupled(Vec(tokens, UInt(16.W))))
    val value = Flipped(Decoupled(Vec(tokens, UInt(16.W))))
    val gates = Flipped(Decoupled(Vec(tokens, new GateToken)))
    val start = Flipped(Decoupled(Bool()))
    val result = Vec(4, Decoupled(new Panel(side, 16, tagBits)))
    val read = Flipped(Vec(banks, Decoupled(UInt(log2Ceil(resultDepth).W))))
    val response = Vec(banks, Decoupled(UInt(128.W)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val phase = Output(UInt(3.W))
    val invalidGates = Output(Bool())
    val coefficientRows = Output(UInt(log2Ceil(10 * side + 1).W))
    val solverFma = Output(UInt(4.W))
    val tileInputValid = Output(UInt(4.W))
    val tileInputReady = Output(UInt(4.W))
    val tileStalled = Output(UInt(16.W))
    val tileTransfers = Output(UInt(16.W))
    val blockReady = Output(UInt(16.W))
    val tileLaunches = Output(UInt(16.W))
    val coefficientEvent = Output(UInt((5 + log2Ceil(side)).W))
    val rhsGrants = Output(UInt(4.W))
    val laneOwners = Output(UInt(12.W))
  })
  val idle :: gram :: scaleGate :: scaleBeta :: waitInverse :: rhsScale :: launch :: execute :: Nil =
    Enum(8)
  val state = RegInit(idle)
  val keys = Seq.fill(4)(SyncReadMem(keyDim, Vec(side, UInt(16.W))))
  val values = Seq.fill(4)(SyncReadMem(valueDim, Vec(side, UInt(16.W))))
  val keyCount = RegInit(0.U(log2Ceil(keyDim + 1).W))
  val valueCount = RegInit(0.U(log2Ceil(valueDim + 1).W))
  val gateLoaded = RegInit(false.B)
  val gates = Reg(Vec(tokens, new GateToken))
  val hostPending = RegInit(VecInit(Seq.fill(banks)(0.U(3.W))))
  val noHost = !hostPending.map(_ =/= 0.U).reduce(_ || _) && !io.read.map(_.valid).reduce(_ || _)
  io.key.ready := state === idle && !io.start.valid && keyCount < keyDim.U
  io.value.ready := state === idle && !io.start.valid && valueCount < valueDim.U
  def unitInterval(x: UInt): Bool = x <= "h3f800000".U || x === "h80000000".U
  val legal = io.gates.bits.map(g => unitInterval(g.decay) && unitInterval(g.beta)).reduce(_ && _)
  io.invalidGates := io.gates.valid && !legal
  io.gates.ready := state === idle && !io.start.valid && legal
  when(io.key.fire) { keyCount := keyCount + 1.U }
  when(io.value.fire) { valueCount := valueCount + 1.U }
  when(io.gates.fire) {
    gates := io.gates.bits
    gateLoaded := true.B
  }
  io.start.ready :=
    state === idle && keyCount === keyDim.U && valueCount === valueDim.U && gateLoaded && noHost

  val tile = Module(
    new NativeTile(
      side,
      maxColumns = width,
      withPreparation = true,
      earlyTriangular = blockReadyExecution
    )
  )
  val prep = tile.io.preparation.get
  prep.active := state =/= execute && state =/= launch
  prep.gram := state === gram
  prep.clear := io.start.fire
  prep.north := 0.U.asTypeOf(prep.north)
  prep.west := 0.U.asTypeOf(prep.west)
  val earlyBegun = RegInit(false.B)
  val streaming = if (blockReadyExecution) state =/= idle && state =/= gram else state === execute
  tile.io.begin.foreach { begin =>
    begin.valid := streaming && !earlyBegun
    begin.bits := width.U
    when(begin.fire) { earlyBegun := true.B }
    when(io.start.fire) { earlyBegun := false.B }
  }
  tile.io.command.valid := state === launch && (!blockReadyExecution).B
  tile.io.command.bits.triangular := true.B
  tile.io.command.bits.columns := width.U
  tile.io.coefficient.valid := false.B
  tile.io.coefficient.bits := 0.U.asTypeOf(tile.io.coefficient.bits)
  val gramTime = RegInit(0.U(log2Ceil(gramCycles + 1).W))
  val blockRow = RegInit(0.U(2.W))
  val blockCol = RegInit(0.U(2.W))
  val row = RegInit(0.U(log2Ceil(side).W))
  val order =
    if (interleavedPreparation)
      Some(Module(new InterleavedPreparationOrder(side, columnOrder = blockReadyExecution)))
    else None
  val rowQueues =
    if (interleavedPreparation) Seq.fill(4)(Module(new Queue(new FloatRow(side), 1))) else Seq.empty
  val selectedBlockRow = order.map(_.io.request.bits.blockRow).getOrElse(blockRow)
  val selectedBlockCol = order.map(_.io.request.bits.blockCol).getOrElse(blockCol)
  val selectedRow = order.map(_.io.request.bits.row).getOrElse(row)
  val producerValid = order.map(_.io.request.valid).getOrElse(true.B)
  order.foreach { o =>
    o.io.start := io.start.fire
    o.io.enable := state === scaleGate || state === scaleBeta
    for (i <- 0 until 4) o.io.diagonalReady(i) := rowQueues(i).io.enq.ready
    o.io.request.ready := false.B
  }
  val globalRow = Cat(selectedBlockRow, selectedRow)
  val gramRows = Wire(Vec(16, Vec(side, UInt(32.W))))
  for (b <- 0 until 16) gramRows(b) := prep.accumulators(b)(selectedRow)
  val gramRow = gramRows(Cat(selectedBlockRow, selectedBlockCol))
  val gateMatrix = Reg(Vec(tokens, Vec(tokens, UInt(32.W))))
  val gatePrevious = Reg(Vec(tokens, UInt(32.W)))
  val prefix = Reg(Vec(tokens, UInt(32.W)))
  val runningPrefix = Reg(UInt(32.W))
  val betaPrefix = Reg(Vec(tokens, UInt(32.W)))
  val scaled = Reg(Vec(side, UInt(32.W)))
  val coefficientRows = RegInit(0.U(log2Ceil(10 * side + 1).W))
  io.coefficientRows := coefficientRows

  val issue = RegInit(VecInit(Seq.fill(4)(0.U(tagBits.W))))
  val pending = RegInit(VecInit(Seq.fill(4)(false.B)))
  val responseTag = Reg(Vec(4, UInt(tagBits.W)))
  val rhs = Seq.fill(4)(Module(new Queue(new Panel(side, 32, tagBits), 2, pipe = true)))
  val raw =
    if (blockReadyExecution)
      Seq.fill(4)(Module(new Queue(new Panel(side, 16, tagBits), 2, pipe = true)))
    else Seq.empty
  val betaReady = RegInit(VecInit(Seq.fill(4)(false.B)))
  val laneRoundRobin = RegInit(0.U(2.W))
  val rhsRequests = Wire(Vec(4, Bool()))
  val betaGrants = Wire(Vec(4, Bool()))
  val rhsGrants = Wire(Vec(4, Bool()))
  val lanes = Seq.fill(tokens)(Module(new FMA(8, 23, 8, 23)))
  for (m <- lanes) {
    m.io.a := 0.U
    m.io.b := 0.U
    m.io.c := 0.U
  }
  val prefixLane = Module(new FMA(8, 23, 8, 23))
  val gateRow = gramTime(log2Ceil(tokens) - 1, 0)
  prefixLane.io.a := runningPrefix
  prefixLane.io.b := gates(gateRow).decay
  prefixLane.io.c := 0.U
  val solvers = Seq.fill(4)(Module(new RowInverseController(side)))
  val solverUsing = VecInit(solvers.map(_.io.usingFma))
  for (i <- 0 until 4) {
    rhsRequests(i) :=
      (if (blockReadyExecution)
         streaming && raw(i).io.deq.valid && rhs(i).io.enq.ready
       else false.B)
    betaGrants(i) := blockReadyExecution.B && streaming && !solverUsing(i) && !betaReady(i)
  }
  val free = VecInit((0 until 4).map(i => !solverUsing(i) && !betaGrants(i)))
  val spare = VecInit((0 until 4).map(i => free(i) && !rhsRequests(i)))
  val eligible = Mux(spare.asUInt.orR, spare.asUInt, free.asUInt)
  val indices = Wire(Vec(4, UInt(2.W)))
  val candidates = Wire(Vec(4, Bool()))
  for (i <- 0 until 4) {
    indices(i) := laneRoundRobin + i.U
    candidates(i) := eligible(indices(i))
  }
  val available = free.asUInt.orR
  val scaleGroup =
    if (blockReadyExecution) indices(PriorityEncoder(candidates.asUInt))
    else PriorityEncoder(~solverUsing.asUInt)
  val scaling = (state === scaleGate || state === scaleBeta) && producerValid && available
  when(blockReadyExecution.B && scaling) { laneRoundRobin := scaleGroup + 1.U }
  val owners = Wire(Vec(4, UInt(3.W)))
  for (i <- 0 until 4) {
    rhsGrants(i) := blockReadyExecution.B && rhsRequests(i) && betaReady(i) && free(
      i
    ) && !(scaling && scaleGroup === i.U)
    owners(i) := Mux(
      solverUsing(i),
      1.U,
      Mux(betaGrants(i), 2.U, Mux(scaling && scaleGroup === i.U, 3.U, Mux(rhsGrants(i), 4.U, 0.U)))
    )
  }
  io.rhsGrants := rhsGrants.asUInt
  io.laneOwners := owners.asUInt
  val scaleOutput = Wire(Vec(4, Vec(side, UInt(32.W))))
  for (i <- 0 until 4; j <- 0 until side) scaleOutput(i)(j) := lanes(i * side + j).io.out
  val scaleResult = scaleOutput(scaleGroup)
  for (i <- 0 until 4) {
    val unit = solvers(i)
    unit.io.start.valid := io.start.fire
    unit.io.start.bits := true.B
    when(io.start.fire) { assert(unit.io.start.ready) }
    val load = if (interleavedPreparation) rowQueues(i).io.enq else unit.io.load
    if (interleavedPreparation) { unit.io.load <> rowQueues(i).io.deq }
    load.valid := state === scaleBeta && producerValid && available && selectedBlockRow === i.U &&
      selectedBlockCol === i.U
    load.bits.row := selectedRow
    unit.io.result.ready := false.B
    for (j <- 0 until side) {
      load.bits.values(j) := Mux(selectedRow > j.U, scaleResult(j), 0.U)
      unit.io.fmaResult(j) := lanes(i * side + j).io.out
      when(unit.io.usingFma) {
        lanes(i * side + j).io.a := unit.io.operands(j).a
        lanes(i * side + j).io.b := unit.io.operands(j).b
        lanes(i * side + j).io.c := unit.io.operands(j).c
      }
      when(scaling && scaleGroup === i.U) {
        lanes(i * side + j).io.a := Mux(state === scaleGate, gramRow(j), scaled(j))
        lanes(i * side + j).io.b := Mux(
          state === scaleGate,
          gateMatrix(globalRow)(Cat(selectedBlockCol, j.U(log2Ceil(side).W))),
          gates(globalRow).beta
        )
        assert(!unit.io.usingFma)
      }
    }
    when(unit.io.usingFma) {
      assert(state === scaleGate || state === scaleBeta || state === waitInverse)
    }
  }
  // Single coefficient-row port: completed J rows have priority over -L rows.
  val publications = VecInit(solvers.map(_.io.result.valid)).asUInt
  val publishIndex = PriorityEncoder(publications)
  val resultRows = VecInit(solvers.map(_.io.result.bits))
  when(publications.orR) {
    tile.io.coefficient.valid := true.B
    tile.io.coefficient.bits.tileRow := publishIndex
    tile.io.coefficient.bits.tileCol := publishIndex
    tile.io.coefficient.bits.row := resultRows(publishIndex).row
    tile.io.coefficient.bits.values := VecInit(
      resultRows(publishIndex).values.map(FloatArithmetic.narrow)
    )
    for (i <- 0 until 4)
      solvers(i).io.result.ready := publishIndex === i.U && tile.io.coefficient.ready
  }.elsewhen(
    state === scaleBeta && producerValid && available && selectedBlockRow =/= selectedBlockCol
  ) {
    tile.io.coefficient.valid := true.B
    tile.io.coefficient.bits.tileRow := selectedBlockRow
    tile.io.coefficient.bits.tileCol := selectedBlockCol
    tile.io.coefficient.bits.row := selectedRow
    tile.io.coefficient.bits.values := VecInit(
      scaleResult.map(x => FloatArithmetic.narrow(x ^ "h80000000".U))
    )
  }
  when(tile.io.coefficient.fire) { coefficientRows := coefficientRows + 1.U }
  assert(coefficientRows <= (10 * side).U)

  val keyData = Wire(Vec(4, Vec(side, UInt(16.W))))
  for (i <- 0 until 4) {
    when(io.key.fire) {
      keys(i).write(keyCount, VecInit((0 until side).map(r => io.key.bits(i * side + r))))
    }
    when(io.value.fire) {
      values(i).write(valueCount, VecInit((0 until side).map(r => io.value.bits(i * side + r))))
    }
    val credit =
      if (blockReadyExecution) (raw(i).io.count +& pending(i).asUInt) < 2.U || raw(i).io.deq.fire
      else (rhs(i).io.count +& pending(i).asUInt) < 2.U || rhs(i).io.deq.fire
    val read = streaming && issue(i) < width.U && credit
    val readK = (state === gram && gramTime < keyDim.U) || (read && issue(i) >= valueDim.U)
    keyData(i) := keys(i).read(Mux(state === gram, gramTime, issue(i) - valueDim.U), readK)
    val vd = values(i).read(issue(i), read && issue(i) < valueDim.U)
    pending(i) := read
    when(read) {
      responseTag(i) := issue(i)
      issue(i) := issue(i) + 1.U
    }
    val data = Mux(responseTag(i) < valueDim.U, vd, keyData(i))
    if (blockReadyExecution) {
      raw(i).io.enq.valid := pending(i)
      raw(i).io.enq.bits.tag := responseTag(i)
      raw(i).io.enq.bits.values := data
      when(pending(i)) { assert(raw(i).io.enq.ready, "unreserved RAW SRAM response") }
      assert((raw(i).io.count +& pending(i).asUInt) <= 2.U)
      rhs(i).io.enq.valid := rhsGrants(i)
      rhs(i).io.enq.bits.tag := raw(i).io.deq.bits.tag
      raw(i).io.deq.ready := rhsGrants(i)
      for (r <- 0 until side) {
        val index = i * side + r
        rhs(i).io.enq.bits.values(r) := lanes(index).io.out
        when(betaGrants(i)) {
          lanes(index).io.a := gates(index).beta
          lanes(index).io.b := prefix(index)
          betaPrefix(index) := lanes(index).io.out
        }
        when(rhsGrants(i)) {
          lanes(index).io.a := Cat(raw(i).io.deq.bits.values(r), 0.U(16.W))
          lanes(index).io.b := Mux(
            raw(i).io.deq.bits.tag < valueDim.U,
            gates(index).beta,
            betaPrefix(index)
          )
        }
      }
      when(betaGrants(i)) { betaReady(i) := true.B }
      when(rhsGrants(i)) {
        assert(!solverUsing(i) && !betaGrants(i) && !(scaling && scaleGroup === i.U))
      }
    } else {
      rhs(i).io.enq.valid := pending(i)
      rhs(i).io.enq.bits.tag := responseTag(i)
      when(pending(i)) { assert(rhs(i).io.enq.ready, "unreserved independent RHS response") }
      for (r <- 0 until side) {
        val index = i * side + r
        rhs(i).io.enq.bits.values(r) := lanes(index).io.out
        when(state === execute) {
          lanes(index).io.a := Cat(data(r), 0.U(16.W))
          lanes(index).io.b := Mux(
            responseTag(i) < valueDim.U,
            gates(index).beta,
            betaPrefix(index)
          )
        }
      }
    }
    tile.io.rhs(i) <> rhs(i).io.deq
    tile.io.operand(i).valid := false.B
    tile.io.operand(i).bits := 0.U.asTypeOf(tile.io.operand(i).bits)
  }
  val gramValid = RegNext(state === gram && gramTime < keyDim.U, false.B)
  for (i <- 0 until 4; j <- 0 to i; r <- 0 until side) {
    val left = Wire(Valid(UInt(32.W)))
    val north = Wire(Valid(UInt(16.W)))
    left.valid := gramValid
    left.bits := Cat(0.U(16.W), keyData(i)(r))
    north.valid := gramValid
    north.bits := keyData(j)(r)
    val wl = StreamPipes.delay(left, r, true.B)
    val nn = StreamPipes.delay(north, r, true.B)
    when(state === gram) {
      prep.west(i * 4 + j)(r) := wl
      prep.north(i * 4 + j)(r) := nn
    }
  }
  when(state === gram && gramTime < tokens.U) {
    for (j <- 0 until tokens) {
      lanes(j).io.a := gatePrevious(j)
      lanes(j).io.b := gates(gateRow).decay
      val g = Mux(j.U < gateRow, lanes(j).io.out, Mux(j.U === gateRow, "h3f800000".U, 0.U))
      gateMatrix(gateRow)(j) := g
      gatePrevious(j) := g
    }
    prefix(gateRow) := prefixLane.io.out
    runningPrefix := prefixLane.io.out
  }

  val resultMemory = Module(new BankedScratchpad(banks, resultDepth))
  val compactWriter = if (compactResults) Some(Module(new CompactResultWriter)) else None
  val written = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(width)(false.B)))))
  for (i <- 0 until 4) {
    val sinkReady = compactWriter.map(_.io.result(i).ready).getOrElse(true.B)
    io.result(i).valid := streaming && tile.io.result(i).valid && sinkReady
    io.result(i).bits.tag := tile.io.result(i).bits.tag
    for (r <- 0 until side) io.result(i).bits.values(r) := tile.io.result(i).bits.values(r)(15, 0)
    tile.io.result(i).ready := streaming && io.result(i).ready && sinkReady
    compactWriter.foreach { writer =>
      // Atomic external consumer / SRAM fork. Scratchpad write-ready is
      // independent of valid, so this does not create a combinational loop.
      writer.io.result(i).valid := streaming && tile.io.result(i).valid && io.result(i).ready
      writer.io.result(i).bits := io.result(i).bits
      assert(!writer.io.invalid(i))
      when(writer.io.result(i).fire) { assert(io.result(i).fire) }
    }
    when(io.result(i).fire) {
      assert(io.result(i).bits.tag < width.U)
      written(i)(io.result(i).bits.tag(log2Ceil(width) - 1, 0)) := true.B
    }
  }
  for (b <- 0 until banks) {
    val group = b / (side / 4)
    val first = (b % (side / 4)) * 4
    if (compactResults) {
      resultMemory.io.write(b) <> compactWriter.get.io.write(b)
    } else {
      resultMemory.io.write(b).valid := io.result(group).fire
      resultMemory.io.write(b).bits.address := io.result(group).bits.tag
      resultMemory.io.write(b).bits.data := Cat(
        0.U(64.W),
        Cat((0 until 4).reverse.map(r => io.result(group).bits.values(first + r)))
      )
      resultMemory.io.write(b).bits.mask := "hffff".U
      when(io.result(group).fire) { assert(resultMemory.io.write(b).ready) }
    }
    val initialized = if (compactResults) {
      val address = io.read(b).bits
      val isU = address >= 256.U && address < 384.U
      val isW = address >= 512.U && address < 576.U
      val tag = Mux(isW, 128.U + ((address - 512.U) << 1) + ((b % 4) / 2).U, address - 256.U)
      (isU || isW) && written(group)(tag(7, 0))
    } else io.read(b).bits < width.U && written(group)(io.read(b).bits)
    resultMemory.io.read(b).valid := state === idle && initialized && io.read(b).valid
    resultMemory.io.read(b).bits := io.read(b).bits
    io.read(b).ready := state === idle && initialized && resultMemory.io.read(b).ready
    io.response(b) <> resultMemory.io.response(b)
    when(io.read(b).fire =/= io.response(b).fire) {
      when(io.read(b).fire) { hostPending(b) := hostPending(b) + 1.U }.otherwise {
        hostPending(b) := hostPending(b) - 1.U
      }
    }
  }
  val selectedReady =
    if (interleavedPreparation) VecInit(rowQueues.map(_.io.enq.ready))(selectedBlockRow)
    else VecInit(solvers.map(_.io.load.ready))(blockRow)
  val rowAccepted = state === scaleBeta && producerValid && available && Mux(
    selectedBlockRow === selectedBlockCol,
    selectedReady,
    !publications.orR && tile.io.coefficient.ready
  )
  order.foreach(_.io.request.ready := rowAccepted)
  when(io.start.fire) {
    state := gram
    gramTime := 0.U
    blockRow := 0.U
    blockCol := 0.U
    row := 0.U
    coefficientRows := 0.U
    keyCount := 0.U
    valueCount := 0.U
    gateLoaded := false.B
    laneRoundRobin := 0.U
    for (i <- 0 until 4) betaReady(i) := false.B
    runningPrefix := "h3f800000".U
    for (j <- 0 until tokens) gatePrevious(j) := 0.U
    for (i <- 0 until 4) {
      issue(i) := 0.U
      for (v <- 0 until width) written(i)(v) := false.B
    }
  }
  switch(state) {
    is(gram) {
      gramTime := gramTime + 1.U
      when(gramTime === (gramCycles - 1).U) { state := scaleGate }
    }
    is(scaleGate) {
      when(available && producerValid) {
        scaled := scaleResult
        state := scaleBeta
      }
      if (interleavedPreparation) { when(order.get.io.done) { state := waitInverse } }
    }
    is(scaleBeta) {
      when(rowAccepted) {
        if (interleavedPreparation) { state := scaleGate }
        else {
          when(row === (side - 1).U) {
            row := 0.U
            when(blockCol === blockRow) {
              blockCol := 0.U
              when(blockRow === 3.U) { state := waitInverse }.otherwise {
                blockRow := blockRow + 1.U
                state := scaleGate
              }
            }.otherwise {
              blockCol := blockCol + 1.U
              state := scaleGate
            }
          }.otherwise {
            row := row + 1.U
            state := scaleGate
          }
        }
      }
    }
    is(waitInverse) {
      when(!solvers.map(_.io.busy).reduce(_ || _)) {
        assert(coefficientRows === (10 * side).U)
        state := (if (blockReadyExecution) execute else rhsScale)
      }
    }
    is(rhsScale) {
      for (r <- 0 until tokens) {
        lanes(r).io.a := gates(r).beta
        lanes(r).io.b := prefix(r)
        betaPrefix(r) := lanes(r).io.out
      }
      state := launch
    }
    is(launch) { when(tile.io.command.fire) { state := execute } }
    is(execute) { when(tile.io.done) { state := idle } }
  }
  io.busy := state =/= idle
  io.done := RegNext(state === execute && tile.io.done, false.B)
  io.phase := state
  io.solverFma := solverUsing.asUInt
  io.tileInputValid := VecInit(tile.io.rhs.map(_.valid)).asUInt
  io.tileInputReady := VecInit(tile.io.rhs.map(_.ready)).asUInt
  io.tileStalled := tile.io.stalled
  io.tileTransfers := tile.io.transfers
  io.blockReady := tile.io.blockReady
  io.tileLaunches := tile.io.launches
  io.coefficientEvent := Cat(
    tile.io.coefficient.fire,
    tile.io.coefficient.bits.tileRow,
    tile.io.coefficient.bits.tileCol,
    tile.io.coefficient.bits.row
  )
}
