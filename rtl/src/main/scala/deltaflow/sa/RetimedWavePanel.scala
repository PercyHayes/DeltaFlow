package deltaflow.sa
import chisel3._
import chisel3.util._
import chisel3.experimental.hierarchy._

class RetimedPanelInput(side: Int, tagBits: Int) extends RetimedMeta(tagBits) {
  val north = Vec(side, UInt(16.W))
  val west = Vec(side, UInt(32.W))
}
class RetimedResult(side: Int, tagBits: Int) extends Bundle {
  val context = UInt(2.W)
  val tag = UInt(tagBits.W)
  val values = Vec(side, UInt(32.W))
}
class RetimedRow(side: Int) extends Bundle {
  val context = UInt(2.W)
  val row = UInt(log2Ceil(side).W)
}
class RetimedRowData(side: Int) extends RetimedRow(side) { val values = Vec(side, UInt(32.W)) }

/** Select registered ownership first, then compare this request's hazards. This is the same
  * permission as selecting one element of contextIdle, but avoids placing a four-way mux after the
  * offered-wave collision checks.
  */
object RetimedContextPermission {
  def apply(
      residentIdle: Vec[Bool],
      requestedContext: UInt,
      offeredOs: Bool,
      offeredContext: UInt,
      dispatchedOs: Bool,
      dispatchedContext: UInt
  ): Bool = {
    require(residentIdle.length == 4)
    val offeredConflict = offeredOs && offeredContext === requestedContext
    val dispatchedConflict = dispatchedOs && dispatchedContext === requestedContext
    residentIdle(requestedContext) && !offeredConflict && !dispatchedConflict
  }
}

class RetimedPanelPort(
    side: Int,
    tagBits: Int = 2,
    columnSeed: Boolean = false,
    coefficientBanks: Int = 1,
    columnRead: Boolean = false,
    osAdmissionStatus: Boolean = false
) extends Bundle {
  val in = Flipped(Decoupled(new RetimedPanelInput(side, tagBits)))
  val out = Decoupled(new RetimedResult(side, tagBits))
  val coefficient = Flipped(Decoupled(new WaveCoefficient(side)))
  val coefficientBank = if (coefficientBanks == 2) Some(Input(Bool())) else None
  val coefficientIdle = if (coefficientBanks == 2) Some(Output(Vec(2, Bool()))) else None
  val coefficientConfigured = if (coefficientBanks == 2) Some(Output(Vec(2, Bool()))) else None
  val seed = Flipped(Decoupled(new RetimedRowData(side)))
  val seedColumn = if (columnSeed) Some(Input(Bool())) else None
  val clear = Flipped(Decoupled(UInt(2.W)))
  val read = Flipped(Decoupled(new RetimedRow(side)))
  val readColumn = if (columnRead) Some(Input(Bool())) else None
  val response = Decoupled(new RetimedRowData(side))
  val contextIdle = Output(Vec(4, Bool()))
  val csIdle = Output(Bool())
  val idle = Output(Bool())
  val configured = Output(Bool())
  val advance = Output(Bool())
  val dispatched = Output(Valid(new RetimedPanelInput(side, tagBits)))
  val inputOccupancy = Output(UInt(2.W))
  val osAdmissible = if (osAdmissionStatus) Some(Output(Vec(4, Bool()))) else None
  // Registered resource state, independent of this cycle's request valids.
  // A phase-exclusive owner may combine these with its own request context;
  // normal ports retain their same-cycle read/clear/seed collision checks.
  val residentContextIdle = if (osAdmissionStatus) Some(Output(Vec(4, Bool()))) else None
  val rowReadAvailable = if (osAdmissionStatus) Some(Output(Bool())) else None
}

/** Complete four-context microarray boundary: 5*side+1 enabled cycles. CS backpressure freezes
  * every arithmetic/link/control register. Row read responses have an independent one/two-slot
  * queue (default two). All payloads count as logic; callers changing depth must recalibrate row
  * extraction.
  */
@instantiable
class RetimedWavePanel(
    val side: Int = 16,
    tagBits: Int = 2,
    val rowResponseDepth: Int = 2,
    val registerInput: Boolean = false,
    val columnSeed: Boolean = false,
    val coefficientBanks: Int = 1,
    val columnRead: Boolean = false,
    val osAdmissionStatus: Boolean = false,
    val registerClear: Boolean = false,
    val compactWestMetadata: Boolean = false,
    val sharedMagnitude: Boolean = false,
    val freeInvalidPayload: Boolean = false,
    val packedAligned: Boolean = false,
    val compactClassification: Boolean = false,
    val splitAlignment: Boolean = false,
    val narrowAddition: Boolean = false,
    val freeForwardPayload: Boolean = false
) extends Module {
  require(Set(4, 8, 16, 32).contains(side))
  require(Set(1, 2).contains(coefficientBanks))
  require(Set(1, 2).contains(rowResponseDepth))
  require(!osAdmissionStatus || registerInput)
  require(!registerClear || registerInput)
  val latency = 5 * side + 1
  @public val io = IO(
    new RetimedPanelPort(side, tagBits, columnSeed, coefficientBanks, columnRead, osAdmissionStatus)
  )
  val pe = Seq.fill(side, side)(
    Module(
      new RetimedWavePE(
        tagBits,
        coefficientBanks,
        compactWestMetadata,
        sharedMagnitude,
        freeInvalidPayload,
        packedAligned,
        compactClassification,
        splitAlignment,
        narrowAddition,
        freeForwardPayload
      )
    )
  )
  // Accepted clear owns its context until the following edge. This separates
  // controller phase/retirement checks from the full PE reset fanout. The input
  // queue prevents a newly accepted wave reaching that context before clear.
  val clearPending = if (registerClear) RegNext(io.clear.fire, false.B) else false.B
  val clearContext = if (registerClear) RegEnable(io.clear.bits, io.clear.fire) else io.clear.bits
  val advance = !io.out.valid || io.out.ready
  io.advance := advance
  // Two non-bypassing entries cut arbitration/data selection from the first
  // multiplier. Admission and arithmetic dispatch have separate RAW tracking.
  // Credits are reserved by the caller at admission, before either entry is used.
  val queuedCS = RegInit(0.U(2.W))
  val queuedOS = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val configured = Wire(Vec(coefficientBanks, Bool()))
  val loadBank = io.coefficientBank.getOrElse(false.B)
  def bank(context: UInt): UInt = if (coefficientBanks == 2) context(0) else 0.U
  def loadingRequested(context: UInt): Bool = io.coefficient.valid && loadBank === bank(context)
  val wave = if (registerInput) {
    val q = Module(new RegisteredWaveInput(side, tagBits))
    val acceptedCooldown = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
    io.osAdmissible.foreach(_ := VecInit(acceptedCooldown.map(_ === 0.U)))
    val allowed = advance && Mux(
      io.in.bits.os,
      acceptedCooldown(io.in.bits.context) === 0.U,
      configured(bank(io.in.bits.context)) && !loadingRequested(io.in.bits.context)
    )
    q.io.enq.valid := io.in.valid && allowed
    q.io.enq.bits := io.in.bits
    io.in.ready := q.io.enq.ready && allowed
    io.inputOccupancy := q.io.count
    val csEnq = q.io.enq.fire && !q.io.enq.bits.os
    val csDeq = q.io.deq.fire && !q.io.deq.bits.os
    when(csEnq =/= csDeq) { queuedCS := Mux(csEnq, queuedCS + 1.U, queuedCS - 1.U) }
    for (i <- 0 until 4) {
      val enq = q.io.enq.fire && q.io.enq.bits.os && q.io.enq.bits.context === i.U
      val deq = q.io.deq.fire && q.io.deq.bits.os && q.io.deq.bits.context === i.U
      when(enq =/= deq) { queuedOS(i) := Mux(enq, queuedOS(i) + 1.U, queuedOS(i) - 1.U) }
      when(advance) {
        when(acceptedCooldown(i) =/= 0.U) { acceptedCooldown(i) := acceptedCooldown(i) - 1.U }
        when(enq) { acceptedCooldown(i) := 3.U }
      }
    }
    assert(queuedCS +& queuedOS.reduce(_ +& _) === q.io.count)
    q.io.deq
  } else {
    io.inputOccupancy := 0.U
    io.in
  }
  io.dispatched.valid := wave.fire
  io.dispatched.bits := wave.bits
  if (registerClear) {
    when(clearPending) {
      assert(
        !(wave.fire && wave.bits.os && wave.bits.context === clearContext),
        "clear context dispatched before commit"
      )
      assert(
        !(io.seed.fire && io.seed.bits.context === clearContext),
        "seed raced registered clear"
      )
      assert(
        !(io.read.fire && io.read.bits.context === clearContext),
        "read raced registered clear"
      )
    }
  }
  val os = wave.bits.os
  val ctx = wave.bits.context
  val cooldown = RegInit(VecInit(Seq.fill(4)(0.U(2.W))))
  val mask = RegInit(VecInit(Seq.fill(coefficientBanks)(0.U(side.W))))
  val loading = RegInit(VecInit(Seq.fill(coefficientBanks)(false.B)))
  val column = RegInit(VecInit(Seq.fill(coefficientBanks)(false.B)))
  for (i <- 0 until coefficientBanks) configured(i) := mask(i).andR && !loading(i)
  io.configured := configured.asUInt.andR
  io.coefficientConfigured.foreach(_ := configured)
  wave.ready := advance && Mux(
    os,
    cooldown(ctx) === 0.U,
    configured(bank(ctx)) && (if (registerInput) true.B else !loadingRequested(ctx))
  )
  val token = Wire(Valid(new RetimedMeta(tagBits)))
  token.valid := wave.fire
  token.bits.os := os
  token.bits.context := ctx
  token.bits.tag := wave.bits.tag
  val retired = StreamPipes.delay(token, latency, advance, freeForwardPayload)
  io.out.valid := retired.valid && !retired.bits.os
  io.out.bits.context := retired.bits.context
  io.out.bits.tag := retired.bits.tag
  val csCount = RegInit(0.U(log2Ceil(latency + 2).W))
  val counts = RegInit(VecInit(Seq.fill(4)(0.U(log2Ceil(latency + 2).W))))
  val residentIdle = Wire(Vec(4, Bool()))
  def contextPermission(context: UInt): Bool = RetimedContextPermission(
    residentIdle,
    context,
    io.in.valid && io.in.bits.os,
    io.in.bits.context,
    wave.valid && os,
    ctx
  )
  val csStart = wave.fire && !os
  val csFinish = retired.valid && !retired.bits.os && advance
  when(csStart =/= csFinish) { csCount := Mux(csStart, csCount + 1.U, csCount - 1.U) }
  when(csFinish) { assert(csCount =/= 0.U) }
  io.csIdle :=
    csCount === 0.U && queuedCS === 0.U && !(io.in.valid && !io.in.bits.os) && !(wave.valid && !os)
  for (i <- 0 until 4) {
    val start = wave.fire && os && ctx === i.U
    val finish = retired.valid && retired.bits.os && retired.bits.context === i.U && advance
    when(start =/= finish) { counts(i) := Mux(start, counts(i) + 1.U, counts(i) - 1.U) }
    when(finish) { assert(counts(i) =/= 0.U) }
    when(advance) {
      when(cooldown(i) =/= 0.U) { cooldown(i) := cooldown(i) - 1.U }
      when(start) { cooldown(i) := 3.U }
    }
    residentIdle(i) :=
      counts(i) === 0.U && queuedOS(i) === 0.U && !(clearPending && clearContext === i.U)
    io.residentContextIdle.foreach(_(i) := residentIdle(i))
    io.contextIdle(i) := contextPermission(i.U(2.W))
  }
  io.idle := io.csIdle && io.contextIdle.asUInt.andR
  if (coefficientBanks == 2) {
    // Lifetime begins on admission, including the two registered input slots.
    // Reload is conservative through full result retirement, also under freeze.
    val reserved = RegInit(VecInit(Seq.fill(2)(0.U(log2Ceil(latency + 4).W))))
    for (i <- 0 until 2) {
      val accept = io.in.fire && !io.in.bits.os && bank(io.in.bits.context) === i.U
      val release = csFinish && bank(retired.bits.context) === i.U
      when(accept =/= release) { reserved(i) := Mux(accept, reserved(i) + 1.U, reserved(i) - 1.U) }
      when(release) { assert(reserved(i) =/= 0.U) }
      io.coefficientIdle.get(i) := reserved(i) === 0.U &&
        !(io.in.valid && !io.in.bits.os && bank(io.in.bits.context) === i.U) &&
        !(wave.valid && !os && bank(ctx) === i.U)
    }
    assert(reserved.reduce(_ +& _) === csCount +& queuedCS)
    io.coefficient.ready := io.coefficientIdle.get(loadBank)
  } else { io.coefficient.ready := io.csIdle }
  when(io.coefficient.fire) {
    val bit = (1.U(side.W) << io.coefficient.bits.index)(side - 1, 0)
    val next = Mux(loading(loadBank), mask(loadBank) | bit, bit)
    when(loading(loadBank)) {
      assert(!(mask(loadBank) & bit).orR)
      assert(column(loadBank) === io.coefficient.bits.column)
    }
    column(loadBank) := io.coefficient.bits.column
    mask(loadBank) := next
    loading(loadBank) := !next.andR
  }
  val rows = Module(
    new Queue(new RetimedRowData(side), rowResponseDepth, pipe = false, flow = false)
  )
  io.rowReadAvailable.foreach(_ := rows.io.enq.ready)
  io.response <> rows.io.deq
  val readPermission = contextPermission(io.read.bits.context)
  io.read.ready := readPermission && rows.io.enq.ready
  rows.io.enq.valid := io.read.valid && readPermission
  rows.io.enq.bits.context := io.read.bits.context
  rows.io.enq.bits.row := io.read.bits.row
  for (c <- 0 until side) {
    val rowValue = VecInit((0 until side).map(r => pe(r)(c).io.accumulators(io.read.bits.context)))(
      io.read.bits.row
    )
    rows.io.enq.bits
      .values(c) :=
      (if (columnRead)
         Mux(
           io.readColumn.get,
           VecInit(
             (0 until side).map(r => pe(c)(r).io.accumulators(io.read.bits.context))
           )(io.read.bits.row),
           rowValue
         )
       else rowValue)
  }
  io.seed.ready := contextPermission(io.seed.bits.context) && !io.read.valid && !io.clear.valid
  io.clear.ready := contextPermission(io.clear.bits) && !io.read.valid
  for (r <- 0 until side; c <- 0 until side) {
    val cell = pe(r)(c)
    cell.io.advance := advance
    cell.io.coefficient.valid := io.coefficient.fire && Mux(
      io.coefficient.bits.column,
      c.U === io.coefficient.bits.index,
      r.U === io.coefficient.bits.index
    )
    cell.io.coefficient.bits := Mux(
      io.coefficient.bits.column,
      io.coefficient.bits.values(r),
      io.coefficient.bits.values(c)
    )
    cell.io.coefficientBank.foreach(_ := loadBank)
    val seedByColumn = io.seedColumn.getOrElse(false.B)
    cell.io.seed.valid := io.seed.fire && Mux(
      seedByColumn,
      io.seed.bits.row === c.U,
      io.seed.bits.row === r.U
    )
    cell.io.seed.bits.context := io.seed.bits.context
    cell.io.seed.bits.value := Mux(seedByColumn, io.seed.bits.values(r), io.seed.bits.values(c))
    cell.io.clear.valid := (if (registerClear) clearPending else io.clear.fire)
    cell.io.clear.bits := clearContext
    if (r > 0) cell.io.north := pe(r - 1)(c).io.south
    else {
      val n = Wire(Valid(new CapacityToken(16, 4, tagBits)))
      n.valid := wave.fire
      n.bits.value := wave.bits.north(c)
      n.bits.os := os
      n.bits.context := ctx
      n.bits.tag := wave.bits.tag
      cell.io.north := StreamPipes.delay(n, 4 * c, advance, freeForwardPayload)
    }
    if (c > 0) {
      cell.io.westOperand := pe(r)(c - 1).io.eastOperand
      cell.io.westSum := pe(r)(c - 1).io.eastSum
    } else {
      val w = Wire(Valid(new CapacityToken(16, 4, tagBits)))
      val sum = Wire(Valid(new CapacityToken(32, 4, tagBits)))
      w.valid := wave.fire && os
      w.bits.value := wave.bits.west(r)(15, 0)
      w.bits.os := true.B
      w.bits.context := ctx
      w.bits.tag := wave.bits.tag
      sum.valid := wave.fire && !os
      sum.bits.value := wave.bits.west(r)
      sum.bits.os := false.B
      sum.bits.context := ctx
      sum.bits.tag := wave.bits.tag
      cell.io.westOperand := StreamPipes.delay(w, r, advance, freeForwardPayload)
      cell.io.westSum := StreamPipes.delay(sum, r + 1, advance, freeForwardPayload)
    }
  }
  for (r <- 0 until side) {
    val result =
      StreamPipes.delay(pe(r)(side - 1).io.eastSum, side - r, advance, freeForwardPayload)
    io.out.bits.values(r) := result.bits.value
    when(retired.valid && !retired.bits.os) {
      assert(
        result.valid && result.bits.tag === retired.bits.tag &&
          result.bits.context === retired.bits.context
      )
    }
  }
}
