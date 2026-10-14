package deltaflow.frontend

import chisel3._
import chisel3.util._
import easyfloat.FMA
import deltaflow.arithmetic.{DiagonalInverse, FloatArithmetic, RowInverseController}
import deltaflow.sa.{TwoBlockCore, SolvedColumn, StreamPipes}
import deltaflow.memory.BankedScratchpad

class GateToken extends Bundle {
  val decay = UInt(32.W) // Activated scalar g in [0,1], not log-g or logits.
  val beta = UInt(32.W)
}

/** GDN K,V,g,beta -> U/W using the SAME three main microtiles for Gram and solve. Input columns are
  * accepted sequentially into synchronous wide memories. FP32 forward gate products avoid division
  * by tiny cumulative prefixes. Parameters and every loop/counter are statically bounded.
  */
class GdnTransformEngine(
    val side: Int = 4,
    val keyDim: Int = 8,
    val valueDim: Int = 7,
    val parallelPreparation: Boolean = false
) extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  require(keyDim >= 2 && keyDim <= 128 && valueDim >= 2 && valueDim <= 128)
  val tokens = 2 * side
  val width = keyDim + valueDim
  val tagBits = log2Ceil(width + 1)
  val bankCount = tokens / 4
  val io = IO(new Bundle {
    val key = Flipped(Decoupled(Vec(tokens, UInt(16.W))))
    val value = Flipped(Decoupled(Vec(tokens, UInt(16.W))))
    val gates = Flipped(Decoupled(Vec(tokens, new GateToken)))
    val start = Flipped(Decoupled(Bool()))
    val result = Decoupled(new SolvedColumn(side, tagBits))
    val read = Flipped(Vec(bankCount, Decoupled(UInt(log2Ceil(width).W))))
    val response = Vec(bankCount, Decoupled(UInt(128.W)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val phase = Output(UInt(4.W))
    val invalidGates = Output(Bool())
    val coefficientRows = Output(UInt(log2Ceil(3 * side + 1).W))
  })
  val (
    idle :: gram :: gate :: scaleGate :: scaleBeta :: inverseLoad :: inverseStart :: inverseWait ::
    publish :: rhsScale :: execute :: Nil
  ) = Enum(11)
  val state = RegInit(idle)
  val keyMemory = SyncReadMem(keyDim, Vec(tokens, UInt(16.W)))
  val valueMemory = SyncReadMem(valueDim, Vec(tokens, UInt(16.W)))
  val keyCount = RegInit(0.U(log2Ceil(keyDim + 1).W))
  val valueCount = RegInit(0.U(log2Ceil(valueDim + 1).W))
  val gateLoaded = RegInit(false.B)
  val gates = Reg(Vec(tokens, new GateToken))
  val hostPending = RegInit(VecInit(Seq.fill(bankCount)(0.U(3.W))))
  val noHost = !hostPending.map(_ =/= 0.U).reduce(_ || _) && !io.read.map(_.valid).reduce(_ || _)
  io.key.ready := state === idle && keyCount < keyDim.U && !io.start.valid
  io.value.ready := state === idle && valueCount < valueDim.U && !io.start.valid
  def unitInterval(x: UInt): Bool = x <= "h3f800000".U || x === "h80000000".U
  val gatesLegal =
    io.gates.bits.map(t => unitInterval(t.decay) && unitInterval(t.beta)).reduce(_ && _)
  io.invalidGates := io.gates.valid && !gatesLegal
  io.gates.ready := state === idle && !io.start.valid && gatesLegal
  when(io.key.fire) {
    keyMemory.write(keyCount, io.key.bits)
    keyCount := keyCount + 1.U
  }
  when(io.value.fire) {
    valueMemory.write(valueCount, io.value.bits)
    valueCount := valueCount + 1.U
  }
  when(io.gates.fire) {
    gates := io.gates.bits
    gateLoaded := true.B
  }
  io.start.ready :=
    state === idle && keyCount === keyDim.U && valueCount === valueDim.U && gateLoaded && noHost
  val core = Module(new TwoBlockCore(side, tagBits, withPreparation = true))
  val prep = core.io.preparation.get
  core.io.coefficient.valid := false.B
  core.io.coefficient.bits := 0.U.asTypeOf(core.io.coefficient.bits)
  prep.active := state =/= execute
  prep.gram := state === gram
  prep.clear := io.start.fire
  prep.rows := 0.U.asTypeOf(prep.rows)
  prep.north := 0.U.asTypeOf(prep.north)
  prep.west := 0.U.asTypeOf(prep.west)
  val gramTime = RegInit(0.U(log2Ceil(keyDim + 2 * side + 2).W))
  val block = RegInit(0.U(2.W))
  val row = RegInit(0.U(log2Ceil(side).W))
  val col = RegInit(0.U(log2Ceil(side).W))
  val gateRow = RegInit(0.U(log2Ceil(tokens).W))
  val gatePrevious = Reg(Vec(tokens, UInt(32.W)))
  val gateMatrix = Reg(Vec(tokens, Vec(tokens, UInt(32.W))))
  val prefix = Reg(Vec(tokens, UInt(32.W)))
  val runningPrefix = Reg(UInt(32.W))
  val scaled = Reg(Vec(side, UInt(32.W)))
  val diagonal = Reg(Vec(2, Vec(side, Vec(side, UInt(32.W)))))
  val betaPrefix = Reg(Vec(tokens, UInt(32.W)))
  val coefficients = RegInit(0.U(log2Ceil(3 * side + 1).W))
  io.coefficientRows := coefficients
  val issueColumn = RegInit(0.U(tagBits.W))
  val solver = if (!parallelPreparation) Some(Module(new DiagonalInverse(side))) else None
  solver.foreach { unit =>
    unit.io.load.valid := state === inverseLoad
    unit.io.load.bits.row := row
    unit.io.load.bits.col := col
    unit.io.load.bits.value := diagonal(block(0))(row)(col)
    unit.io.start.valid := state === inverseStart
    unit.io.start.bits := true.B
  }
  val rowSolvers =
    if (parallelPreparation) Seq.fill(2)(Module(new RowInverseController(side))) else Seq.empty
  val scaleUpper = if (parallelPreparation) block === 2.U else false.B
  val scaleEnabled =
    if (parallelPreparation)
      !Mux(scaleUpper, rowSolvers(0).io.usingFma, rowSolvers(1).io.usingFma)
    else true.B
  val scaleAccepted =
    if (parallelPreparation)
      Mux(
        block === 0.U,
        rowSolvers(0).io.load.ready,
        Mux(block === 2.U, rowSolvers(1).io.load.ready, true.B)
      )
    else true.B

  // Shared 2b FP32 lanes plus prefix; only the serial baseline adds a scalar solver FMA.
  val mul = Seq.fill(tokens)(Module(new FMA(8, 23, 8, 23)))
  for (m <- mul) {
    m.io.a := 0.U
    m.io.b := 0.U
    m.io.c := 0.U
  }
  val scaleA = WireDefault(VecInit(Seq.fill(side)(0.U(32.W))))
  val scaleB = WireDefault(VecInit(Seq.fill(side)(0.U(32.W))))
  val scaleResult = Wire(Vec(side, UInt(32.W)))
  for (j <- 0 until side) {
    scaleResult(j) := Mux(scaleUpper, mul(side + j).io.out, mul(j).io.out)
    when((state === scaleGate || state === scaleBeta) && scaleEnabled) {
      when(scaleUpper) {
        mul(side + j).io.a := scaleA(j)
        mul(side + j).io.b := scaleB(j)
      }
        .otherwise {
          mul(j).io.a := scaleA(j)
          mul(j).io.b := scaleB(j)
        }
    }
  }
  val prefixMul = Module(new FMA(8, 23, 8, 23))
  prefixMul.io.a := runningPrefix
  prefixMul.io.b := gates(gateRow).decay
  prefixMul.io.c := 0.U
  val globalRow = Cat(block =/= 0.U, row)
  val accumulatorRows = Wire(Vec(3, Vec(side, UInt(32.W))))
  for (b <- 0 until 3) accumulatorRows(b) := prep.accumulators(b)(row)
  val gramRow = accumulatorRows(block)

  val rhsQueue = Module(new Queue(new deltaflow.sa.RhsColumn(side, tagBits), 2, pipe = true))
  val rhsPending = RegInit(false.B)
  val responseTag = Reg(UInt(tagBits.W))
  val rhsRead = state === execute && issueColumn < width.U &&
    ((rhsQueue.io.count +& rhsPending.asUInt) < 2.U || rhsQueue.io.deq.fire)
  val readKey = (state === gram && gramTime < keyDim.U) || (rhsRead && issueColumn >= valueDim.U)
  val readValue = rhsRead && issueColumn < valueDim.U
  val keyAddress = Mux(state === gram, gramTime, issueColumn - valueDim.U)
  val keyData = keyMemory.read(keyAddress, readKey)
  val valueData = valueMemory.read(issueColumn, readValue)
  val gramValid = RegNext(state === gram && gramTime < keyDim.U, false.B)
  rhsPending := rhsRead
  when(rhsRead) {
    responseTag := issueColumn
    issueColumn := issueColumn + 1.U
  }
  val rhsData = Mux(responseTag < valueDim.U, valueData, keyData)
  for (r <- 0 until side; b <- 0 until 3) {
    val left = Wire(Valid(UInt(32.W)))
    val right = Wire(Valid(UInt(16.W)))
    left.valid := gramValid
    right.valid := gramValid
    left.bits := Cat(0.U(16.W), keyData((if (b == 0) 0 else side) + r))
    right.bits := keyData((if (b == 2) side else 0) + r)
    val skewLeft = StreamPipes.delay(left, r, true.B)
    val skewRight = StreamPipes.delay(right, r, true.B)
    when(state === gram) {
      prep.west(b)(r) := skewLeft
      prep.north(b)(r) := skewRight
    }
  }
  rhsQueue.io.enq.valid := rhsPending
  rhsQueue.io.enq.bits.tag := responseTag
  core.io.in <> rhsQueue.io.deq
  when(rhsPending) { assert(rhsQueue.io.enq.ready, "unreserved RHS response") }
  for (r <- 0 until tokens) {
    val value = mul(r).io.out
    if (r < side) rhsQueue.io.enq.bits.r0(r) := value
    else rhsQueue.io.enq.bits.r1(r - side) := value
    when(state === execute) {
      mul(r).io.a := Cat(rhsData(r), 0.U(16.W))
      mul(r).io.b := Mux(responseTag < valueDim.U, gates(r).beta, betaPrefix(r))
    }
  }

  val resultMemory = Module(new BankedScratchpad(bankCount, width))
  io.result.valid := state === execute && core.io.out.valid
  io.result.bits := core.io.out.bits
  core.io.out.ready := state === execute && io.result.ready
  for (b <- 0 until bankCount) {
    val values = if (b < side / 4) core.io.out.bits.x0 else core.io.out.bits.x1
    val first = (b % (side / 4)) * 4
    resultMemory.io.write(b).valid := io.result.fire
    resultMemory.io.write(b).bits.address := core.io.out.bits.tag
    resultMemory.io.write(b).bits.data := Cat(
      0.U(64.W),
      Cat((0 until 4).reverse.map(i => values(first + i)))
    )
    resultMemory.io.write(b).bits.mask := "hffff".U
    when(io.result.fire) { assert(resultMemory.io.write(b).ready) }
    resultMemory.io.read(b).valid := state === idle && io.read(b).valid
    resultMemory.io.read(b).bits := io.read(b).bits
    io.read(b).ready := state === idle && resultMemory.io.read(b).ready
    io.response(b) <> resultMemory.io.response(b)
    when(io.read(b).fire =/= io.response(b).fire) {
      when(io.read(b).fire) { hostPending(b) := hostPending(b) + 1.U }
        .otherwise { hostPending(b) := hostPending(b) - 1.U }
    }
  }
  if (parallelPreparation) {
    for (i <- 0 until 2) {
      val unit = rowSolvers(i)
      val offset = if (i == 0) side else 0
      unit.io.start.valid := io.start.fire
      unit.io.start.bits := true.B
      when(io.start.fire) { assert(unit.io.start.ready) }
      unit.io.load.valid := state === scaleBeta && scaleEnabled && block === (2 * i).U
      unit.io.load.bits.row := row
      for (j <- 0 until side) {
        unit.io.load.bits.values(j) := Mux(row > j.U, scaleResult(j), 0.U)
        unit.io.fmaResult(j) := mul(offset + j).io.out
        when(unit.io.usingFma) {
          mul(offset + j).io.a := unit.io.operands(j).a
          mul(offset + j).io.b := unit.io.operands(j).b
          mul(offset + j).io.c := unit.io.operands(j).c
        }
        prep.rows(2 * i).bits.values(j) := FloatArithmetic.narrow(unit.io.result.bits.values(j))
      }
      unit.io.result.ready := prep.active
      prep.rows(2 * i).valid := unit.io.result.valid
      prep.rows(2 * i).bits.row := unit.io.result.bits.row
      when(unit.io.usingFma) { assert(state =/= gram && state =/= execute && state =/= rhsScale) }
    }
  }
  val finalResult = io.result.fire && core.io.out.bits.tag === (width - 1).U
  io.done := RegNext(finalResult, false.B)
  io.busy := state =/= idle
  io.phase := state
  when(io.start.fire) {
    state := gram
    gramTime := 0.U
    coefficients := 0.U
    keyCount := 0.U
    valueCount := 0.U
    gateLoaded := false.B
    runningPrefix := "h3f800000".U
    gateRow := 0.U
    block := 0.U
    row := 0.U
    for (i <- 0 until tokens) gatePrevious(i) := 0.U
  }
  val gateActive =
    if (parallelPreparation) state === gram && gramTime < tokens.U else state === gate
  when(gateActive) {
    for (j <- 0 until tokens) {
      mul(j).io.a := gatePrevious(j)
      mul(j).io.b := gates(gateRow).decay
      val g = Mux(j.U < gateRow, mul(j).io.out, Mux(j.U === gateRow, "h3f800000".U, 0.U))
      gateMatrix(gateRow)(j) := g
      gatePrevious(j) := g
    }
    prefix(gateRow) := prefixMul.io.out
    runningPrefix := prefixMul.io.out
    when(gateRow === (tokens - 1).U) {
      if (!parallelPreparation) {
        state := scaleGate
        block := 0.U
        row := 0.U
      }
    }.otherwise { gateRow := gateRow + 1.U }
  }
  switch(state) {
    is(gram) {
      gramTime := gramTime + 1.U
      // Includes synchronous memory response and complete wavefront drainage.
      when(gramTime === (keyDim + 2 * side - 1).U) {
        if (parallelPreparation) { state := scaleGate }
        else {
          state := gate
          gateRow := 0.U
        }
      }
    }
    is(gate) {} // Serial baseline gate work is driven above.
    is(scaleGate) {
      when(scaleEnabled) {
        for (j <- 0 until side) {
          scaleA(j) := gramRow(j)
          scaleB(j) := gateMatrix(globalRow)(Cat(block === 2.U, j.U(log2Ceil(side).W)))
          scaled(j) := scaleResult(j)
        }
        state := scaleBeta
      }
    }
    is(scaleBeta) {
      when(scaleEnabled) {
        for (j <- 0 until side) {
          scaleA(j) := scaled(j)
          scaleB(j) := gates(globalRow).beta
        }
      }
      when(scaleEnabled && scaleAccepted) {
        for (j <- 0 until side) {
          val masked =
            Mux(globalRow > Cat(block === 2.U, j.U(log2Ceil(side).W)), scaleResult(j), 0.U)
          when(block === 1.U) {
            prep.rows(1).bits.values(j) := FloatArithmetic.narrow(masked ^ "h80000000".U)
          }.otherwise {
            if (!parallelPreparation) diagonal(Mux(block === 0.U, 0.U, 1.U))(row)(j) := masked
          }
        }
        when(block === 1.U) {
          prep.rows(1).valid := true.B
          prep.rows(1).bits.row := row
          coefficients := coefficients + 1.U
        }
        when(row === (side - 1).U) {
          row := 0.U
          when(block === 2.U) {
            block := 0.U
            row := 1.U
            col := 0.U
            if (parallelPreparation) { state := inverseWait }
            else { state := inverseLoad }
          }
            .otherwise {
              block := block + 1.U
              state := scaleGate
            }
        }.otherwise {
          row := row + 1.U
          state := scaleGate
        }
      }
    }
    is(inverseLoad) {
      solver.foreach { unit =>
        when(unit.io.load.fire) {
          when(col === row - 1.U) {
            col := 0.U
            when(row === (side - 1).U) { state := inverseStart }
              .otherwise { row := row + 1.U }
          }.otherwise { col := col + 1.U }
        }
      }
    }
    is(inverseStart) {
      solver.foreach { unit => when(unit.io.start.fire) { state := inverseWait } }
    }
    is(inverseWait) {
      if (parallelPreparation) {
        when(!rowSolvers.map(_.io.busy).reduce(_ || _)) { state := rhsScale }
      } else
        solver.foreach { unit =>
          when(unit.io.done) {
            state := publish
            row := 0.U
          }
        }
    }
    is(publish) {
      solver.foreach { unit =>
        for (b <- Seq(0, 2)) {
          prep.rows(b).valid := block === (b / 2).U
          prep.rows(b).bits.row := row
          for (j <- 0 until side)
            prep.rows(b).bits.values(j) := FloatArithmetic.narrow(unit.io.result(row)(j))
        }
        coefficients := coefficients + 1.U
        when(row === (side - 1).U) {
          when(block === 0.U) {
            block := 1.U
            row := 1.U
            col := 0.U
            state := inverseLoad
          }
            .otherwise { state := rhsScale }
        }.otherwise { row := row + 1.U }
      }
    }
    is(rhsScale) {
      for (r <- 0 until tokens) {
        mul(r).io.a := gates(r).beta
        mul(r).io.b := prefix(r)
        betaPrefix(r) := mul(r).io.out
      }
      issueColumn := 0.U
      state := execute
    }
    is(execute) { when(finalResult) { state := idle } }
  }
  if (parallelPreparation) {
    val commits = PopCount(prep.rows.map(_.valid))
    when(commits =/= 0.U) { coefficients := coefficients + commits }
    when(io.start.fire) { coefficients := 0.U }
    assert(coefficients <= (3 * side).U)
  }

}
