package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.memory._
import deltaflow.arithmetic.FloatArithmetic

class RecurrentCommand extends Bundle {
  val keys = UInt(8.W)
  val values = UInt(8.W)
  val tokens = UInt(7.W)
  val stateBase = UInt(9.W)
  val rebuild = Bool()
}

/** Native operations read the same row in every selected SRAM bank. */
class RecurrentRowRequest extends Bundle {
  val address = UInt(10.W)
  val mask = UInt(16.W)
  val tag = UInt(10.W)
}

/** Reserve one of the existing three tags before issuing a row to its banks. Each bank can accept
  * independently; only the address and unfinished mask are held locally. Payload stays in the
  * existing SRAM return queues. No later row overtakes an unfinished row in any bank.
  */
class RecurrentVectorReader extends Module {
  val io = IO(new Bundle {
    val request = Flipped(Decoupled(new RecurrentRowRequest))
    val result = Decoupled(new BankVectorResult)
    val read = Vec(16, Decoupled(UInt(10.W)))
    val response = Flipped(Vec(16, Decoupled(UInt(128.W))))
    val idle = Output(Bool())
  })
  val tags = Module(new Queue(UInt(26.W), 3, pipe = false, flow = false))
  val remaining = RegInit(0.U(16.W))
  val pendingAddress = Reg(UInt(10.W))
  val partial = remaining.orR

  // Admission uses registered credits, never the other SRAM's bank ready.
  io.request.ready := tags.io.enq.ready && !partial
  tags.io.enq.valid := io.request.fire
  tags.io.enq.bits := Cat(io.request.bits.mask, io.request.bits.tag)
  val issue = partial || io.request.fire
  val issueMask = Mux(partial, remaining, io.request.bits.mask)
  val issueAddress = Mux(partial, pendingAddress, io.request.bits.address)
  for (b <- 0 until 16) {
    io.read(b).valid := issue && issueMask(b)
    io.read(b).bits := issueAddress
  }
  val accepted = VecInit(io.read.map(_.fire)).asUInt
  when(issue) { remaining := issueMask & ~accepted }
  when(io.request.fire) { pendingAddress := io.request.bits.address }

  val mask = tags.io.deq.bits(25, 10)
  val returned = (0 until 16).map(b => !mask(b) || io.response(b).valid).reduce(_ && _)
  io.result.valid := tags.io.deq.valid && returned
  io.result.bits.tag := tags.io.deq.bits(9, 0)
  tags.io.deq.ready := io.result.ready && returned
  for (b <- 0 until 16) {
    io.result.bits.words(b) := Mux(mask(b), io.response(b).bits, 0.U)
    io.response(b).ready := io.result.fire && mask(b)
  }
  io.idle := !tags.io.deq.valid && !partial
  when(partial) {
    assert(tags.io.deq.valid, "unfinished bank reads require an admitted tag")
    assert(!io.request.fire, "a later row cannot overtake unfinished reads")
  }
}

/** Short recurrence and corrected-E replay, sharing the existing FP32 lanes. No PE, MAC, R16 or C32
  * instances. Operand rows 0..383: six rows/token Q,K,V,g/beta,Elo,Ehi; rows 384..399: eight
  * interleaved dot partials; rows 512..703: O(BF16),Elo/Ehi(FP32). Work contains FP32 H. Dot
  * partials live in SRAM, with a scoreboard until the FMA write retires. Rebuild consumes cached E,
  * never V, and does not recompute projections.
  */
class RecurrentStateEngine(val side: Int = 16) extends Module {
  require(Set(4, 16).contains(side))
  val lanes = 4 * side
  val width = 2 * lanes
  val bankCount = lanes / 4
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new RecurrentCommand))
    val operand = Flipped(new ScratchpadPort(16, 1024))
    val work = Flipped(new ScratchpadPort(16, 512))
    val scale = Flipped(new BorrowedScalePorts(side))
    val idle = Output(Bool())
    val done = Output(Bool())
    val error = Output(Bool())
    val fmaIssued = Output(Bool())
  })
  val (
    sIdle :: sLoad :: sLoadWait :: sClear :: sDot :: sReduceRead :: sReduceIssue :: sReduceWait ::
    sE1 :: sE1Issue :: sE1Wait :: sE2 :: sE2Wait :: sScale :: sOuter :: sWrite :: sDone :: Nil
  ) = Enum(17)
  val state = RegInit(sIdle)
  val cmd = Reg(new RecurrentCommand)
  val token = RegInit(0.U(7.W))
  val inputBase = RegInit(0.U(10.W))
  val outputBase = RegInit(512.U(10.W))
  val loadIndex = Reg(UInt(3.W))
  val error = RegInit(false.B)
  // Two transient operand registers, 512 B total at b16. All E, partials,
  // input windows and state remain in the existing SRAM, not new data queues.
  val coefficient = Reg(Vec(width, UInt(16.W)))
  val decay = Reg(UInt(32.W))
  val beta = Reg(UInt(32.W))
  val accumulator = Reg(Vec(lanes, UInt(32.W)))
  val query = RegInit(false.B)
  val clearIndex = Reg(UInt(4.W))
  val half = RegInit(false.B)
  val reduceIndex = Reg(UInt(3.W))
  val send = RegInit(0.U(9.W))
  val committed = RegInit(0.U(9.W))
  val partialBusy = RegInit(VecInit(Seq.fill(16)(false.B)))
  val tags = Module(new Queue(UInt(9.W), 16, pipe = false, flow = false))
  val operand = Module(new RecurrentVectorReader)
  val work = Module(new RecurrentVectorReader)
  for ((reader, port) <- Seq((operand, io.operand), (work, io.work))) {
    for (b <- 0 until 16) {
      port.read(b) <> reader.io.read(b)
      reader.io.response(b) <> port.response(b)
    }
    reader.io.request.valid := false.B
    reader.io.request.bits := 0.U.asTypeOf(reader.io.request.bits)
    reader.io.result.ready := false.B
  }
  for (port <- Seq(io.operand, io.work); b <- 0 until 16) {
    port.write(b).valid := false.B
    port.write(b).bits := 0.U.asTypeOf(port.write(b).bits)
    port.write(b).bits.mask := 65535.U
  }
  def request(
      reader: RecurrentVectorReader,
      address: UInt,
      tag: UInt = 0.U,
      all: Boolean = false
  ): Unit = {
    reader.io.request.bits.mask := (if (all) 65535 else (1 << bankCount) - 1).U
    reader.io.request.bits.tag := tag
    reader.io.request.bits.address := address
  }
  def fp(words: Vec[UInt], j: Int): UInt = words(j / 4)(32 * (j % 4) + 31, 32 * (j % 4))
  def narrow(words: Vec[UInt], j: Int): UInt = words(j / 8)(16 * (j % 8) + 15, 16 * (j % 8))
  val one = "h3f800000".U(32.W)
  io.start.ready := state === sIdle
  io.idle := state === sIdle
  io.done := state === sDone
  io.error := error
  io.scale.request.valid := false.B
  io.scale.response.ready := false.B
  for (j <- 0 until lanes) io.scale.request.bits(j) := 0.U.asTypeOf(io.scale.request.bits(j))
  io.fmaIssued := io.scale.request.fire
  tags.io.enq.valid := false.B
  tags.io.enq.bits := 0.U
  tags.io.deq.ready := false.B
  val operandWriteReady = io.operand.write.map(_.ready).reduce(_ && _)
  val workWriteReady = io.work.write.take(bankCount).map(_.ready).reduce(_ && _)
  def phase(next: UInt): Unit = {
    state := next
    send := 0.U
    committed := 0.U
  }
  def clearDot(isQuery: Bool): Unit = {
    query := isQuery
    clearIndex := 0.U
    state := sClear
    for (j <- 0 until lanes) accumulator(j) := 0.U
  }
  def nextToken(): Unit = {
    when(token + 1.U === cmd.tokens) { state := sDone }.otherwise {
      token := token + 1.U
      inputBase := inputBase + 6.U
      outputBase := outputBase + 3.U
      loadIndex := 1.U
      state := sLoad
    }
  }
  when(io.start.fire) {
    assert(io.start.bits.keys > 0.U && io.start.bits.keys <= width.U)
    assert(io.start.bits.values > 0.U && io.start.bits.values <= width.U)
    assert(io.start.bits.tokens > 0.U && io.start.bits.tokens <= 64.U)
    assert(io.start.bits.stateBase +& (io.start.bits.keys << 1) <= 512.U)
    assert(operand.io.idle && work.io.idle && !tags.io.deq.valid)
    cmd := io.start.bits
    token := 0.U
    inputBase := 0.U
    outputBase := 512.U
    error := false.B
    loadIndex := 1.U
    state := sLoad
    for (i <- 0 until 16) partialBusy(i) := false.B
  }
  when(state === sDone) { state := sIdle }
  when(state === sLoad) {
    request(operand, inputBase + loadIndex, all = true)
    operand.io.request.valid := true.B
    when(operand.io.request.fire) { state := sLoadWait }
  }
  when(state === sLoadWait) {
    operand.io.result.ready := true.B
    when(operand.io.result.fire) {
      for (j <- 0 until width) {
        when(loadIndex <= 1.U) { coefficient(j) := narrow(operand.io.result.bits.words, j) }
      }
      when(loadIndex === 3.U) {
        val g = fp(operand.io.result.bits.words, 0)
        val b = fp(operand.io.result.bits.words, 1)
        decay := g
        beta := b
        when(g > one || (!cmd.rebuild && b > one)) {
          error := true.B
          state := sDone
        }
          .elsewhen(cmd.rebuild) { phase(sScale) }
          .otherwise { clearDot(false.B) }
      }.elsewhen(loadIndex === 0.U) {
        clearDot(true.B)
      }.otherwise {
        loadIndex := 3.U
        state := sLoad
      }
    }
  }
  when(state === sClear) {
    for (b <- 0 until 16) {
      io.operand.write(b).valid := true.B
      io.operand.write(b).bits.address := 384.U + clearIndex
    }
    when(operandWriteReady) {
      when(clearIndex === 15.U) { phase(sDot) }.otherwise { clearIndex := clearIndex + 1.U }
    }
  }
  val streaming = state === sDot || state === sScale || state === sOuter
  when(streaming) {
    val dot = state === sDot
    val readsOperand = dot || state === sOuter
    val canRead = send < (cmd.keys << 1) && (!dot || !partialBusy(send(3, 0)))
    request(work, cmd.stateBase + send, send)
    request(
      operand,
      Mux(dot, 384.U + send(3, 0), Mux(cmd.rebuild, inputBase + 4.U, outputBase + 1.U) + send(0)),
      send
    )
    work.io.request.valid := canRead && (!readsOperand || operand.io.request.ready)
    operand.io.request.valid := canRead && readsOperand && work.io.request.ready
    when(work.io.request.fire) {
      send := send + 1.U
      when(dot) {
        assert(operand.io.request.fire)
        partialBusy(send(3, 0)) := true.B
      }
    }
    val wr = work.io.result
    val op = operand.io.result
    val row = wr.bits.tag(8, 0)
    val available = wr.valid && (!readsOperand || op.valid)
    io.scale.request.valid := available && tags.io.enq.ready
    tags.io.enq.valid := io.scale.request.fire
    tags.io.enq.bits := row
    wr.ready := io.scale.request.ready && tags.io.enq.ready && (!readsOperand || op.valid)
    op.ready := readsOperand && wr.valid && io.scale.request.ready && tags.io.enq.ready
    when(io.scale.request.fire && readsOperand) { assert(wr.bits.tag === op.bits.tag) }
    val scalar = Cat(coefficient(row(8, 1)), 0.U(16.W))
    for (j <- 0 until lanes) {
      val valid = (row(0).asUInt * lanes.U + j.U) < cmd.values
      val h = fp(wr.bits.words, j)
      io.scale.request.bits(j).a := Mux(dot, scalar, Mux(state === sScale, decay, scalar))
      io.scale.request.bits(j).b := Mux(valid, Mux(state === sOuter, fp(op.bits.words, j), h), 0.U)
      io.scale.request.bits(j).c := Mux(
        dot,
        fp(op.bits.words, j),
        Mux(state === sOuter && valid, h, 0.U)
      )
    }
    val target = tags.io.deq.bits
    val ready = Mux(dot, operandWriteReady, workWriteReady)
    io.scale.response.ready := tags.io.deq.valid && ready
    tags.io.deq.ready := io.scale.response.valid && ready
    for (b <- 0 until 16) {
      val data =
        if (b < bankCount) Cat((0 until 4).reverse.map(j => io.scale.response.bits(4 * b + j)))
        else 0.U(128.W)
      io.operand.write(b).valid := dot && io.scale.response.valid && tags.io.deq.valid
      io.operand.write(b).bits.address := 384.U + target(3, 0)
      io.operand.write(b).bits.data := data
      if (b < bankCount) {
        io.work.write(b).valid := !dot && io.scale.response.valid && tags.io.deq.valid
        io.work.write(b).bits.address := cmd.stateBase + target
        io.work.write(b).bits.data := data
      }
    }
    when(io.scale.response.fire) {
      assert(tags.io.deq.valid)
      committed := committed + 1.U
      when(dot) { partialBusy(target(3, 0)) := false.B }
      when(committed + 1.U === (cmd.keys << 1)) {
        assert(send === (cmd.keys << 1))
        when(dot) {
          half := false.B
          reduceIndex := 0.U
          state := sReduceRead
        }
          .elsewhen(state === sScale) { phase(sOuter) }
          .otherwise {
            when(cmd.rebuild) { nextToken() }.otherwise {
              loadIndex := 0.U
              state := sLoad
            }
          }
      }
    }
  }
  when(state === sReduceRead) {
    request(operand, 384.U + Cat(reduceIndex, half))
    operand.io.request.valid := true.B
    when(operand.io.request.fire) { state := sReduceIssue }
  }
  when(state === sReduceIssue) {
    io.scale.request.valid := operand.io.result.valid
    operand.io.result.ready := io.scale.request.ready
    for (j <- 0 until lanes) {
      io.scale.request.bits(j).a := one
      io.scale.request.bits(j).b := fp(operand.io.result.bits.words, j)
      io.scale.request.bits(j).c := accumulator(j)
    }
    when(io.scale.request.fire) { state := sReduceWait }
  }
  when(state === sReduceWait) {
    io.scale.response.ready := true.B
    when(io.scale.response.fire) {
      accumulator := io.scale.response.bits
      when(reduceIndex === 7.U) { state := Mux(query, sWrite, sE1) }
        .otherwise {
          reduceIndex := reduceIndex + 1.U
          state := sReduceRead
        }
    }
  }
  when(state === sE1) {
    request(operand, inputBase + 2.U, all = true)
    operand.io.request.valid := true.B
    when(operand.io.request.fire) { state := sE1Issue }
  }
  when(state === sE1Issue || state === sE2) {
    io.scale.request.valid := state === sE2 || operand.io.result.valid
    operand.io.result.ready := state === sE1Issue && io.scale.request.ready
    val values = VecInit((0 until width).map(j => narrow(operand.io.result.bits.words, j)))
    for (j <- 0 until lanes) {
      io.scale.request.bits(j).a := Mux(state === sE1Issue, decay ^ "h80000000".U, beta)
      io.scale.request.bits(j).b := accumulator(j)
      io.scale.request.bits(j).c := Mux(
        state === sE1Issue && (half.asUInt * lanes.U + j.U) < cmd.values,
        Cat(values(half.asUInt * lanes.U + j.U), 0.U(16.W)),
        0.U
      )
    }
    when(io.scale.request.fire) { state := Mux(state === sE1Issue, sE1Wait, sE2Wait) }
  }
  when(state === sE1Wait || state === sE2Wait) {
    io.scale.response.ready := state === sE1Wait || operandWriteReady
    for (b <- 0 until 16) {
      io.operand.write(b).valid := state === sE2Wait && io.scale.response.valid
      io.operand.write(b).bits.address := outputBase + 1.U + half
      io.operand
        .write(b)
        .bits
        .data :=
        (if (b < bankCount)
           Cat((0 until 4).reverse.map(j => io.scale.response.bits(4 * b + j)))
         else 0.U(128.W))
    }
    when(io.scale.response.fire) {
      accumulator := io.scale.response.bits
      when(state === sE1Wait) { state := sE2 }
        .elsewhen(!half) {
          half := true.B
          reduceIndex := 0.U
          for (j <- 0 until lanes) accumulator(j) := 0.U
          state := sReduceRead
        }
        .otherwise { phase(sScale) }
    }
  }
  when(state === sWrite) {
    val rounded = VecInit(accumulator.map(FloatArithmetic.narrow))
    for (b <- 0 until 16) {
      val out =
        if (b < width / 8) Cat((0 until 8).reverse.map(j => rounded(8 * (b % (lanes / 8)) + j)))
        else 0.U(128.W)
      val selected = if (b < width / 8) (b / (lanes / 8)).U === half.asUInt else false.B
      io.operand.write(b).valid := !half || selected
      io.operand.write(b).bits.address := outputBase
      io.operand.write(b).bits.data := Mux(selected, out, 0.U)
    }
    when(operandWriteReady) {
      when(half) { nextToken() }.otherwise {
        half := true.B
        reduceIndex := 0.U
        for (j <- 0 until lanes) accumulator(j) := 0.U
        state := sReduceRead
      }
    }
  }
}
