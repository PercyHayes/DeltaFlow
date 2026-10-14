package deltaflow.frontend

import chisel3._
import chisel3.util._
import deltaflow.sa._
import deltaflow.memory.RegisteredPayloadQueue

class StateOutputCommand extends Bundle {
  val epoch = UInt(32.W)
  val half = Bool()
}
class StateOutputSeed(side: Int) extends Bundle {
  val index = UInt(log2Ceil(12 * side).W)
  val values = Vec(4 * side, UInt(32.W))
}
class StateOutputColumn(side: Int) extends Bundle {
  val token = UInt(log2Ceil(4 * side).W)
  val n = Vec(8 * side, UInt(16.W))
  val m = Vec(4 * side, UInt(16.W))
  val e = Vec(4 * side, UInt(16.W))
}
class StateOutputRow(side: Int) extends StateOutputSeed(side) {
  val epoch = UInt(32.W)
  val half = Bool()
}

/** One value slab: [Hlo; Hhi; O] = [D*Hlo; D*Hhi; Z] + [Nlo; Nhi; M]*E. Borrows the existing
  * sixteen panels; contains NO MAC, C32 or SRAM instance. Default C32 0/1/2 own the targets. With
  * residentZ, H uses C0/C2 and O continues the existing C1/C3 for the selected value half. R16 and
  * the other half's odd C32 are untouched. No separate Z seed is then accepted. Two bounded column
  * frames retain E until all three consumers of all panels have captured it. Each panel advances
  * independently, in increasing token order; the panel's registered admission enforces the
  * four-cycle C32 RAW. Inputs are already rounded/scaled. Seed rows and tokens arrive in order.
  * Old-state projection readers must release their lease before start.
  */
class StateOutputController(
    val side: Int = 16,
    val residentZ: Boolean = false,
    val groupedSeeds: Boolean = false,
    val zeroState: Boolean = false,
    val outputFirst: Boolean = false,
    val registeredResultHead: Boolean = false
) extends Module {
  require(Set(4, 16).contains(side))
  require(!groupedSeeds || residentZ)
  require(!zeroState || residentZ)
  require(!outputFirst || zeroState)
  val t = 4 * side
  val rowBits = log2Ceil(3 * t)
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new StateOutputCommand))
    val oldStateReleased = Input(Bool())
    val seed = Flipped(Decoupled(new StateOutputSeed(side)))
    val column = Flipped(Decoupled(new StateOutputColumn(side)))
    val panelEnable = Input(UInt(16.W))
    val input = Vec(16, Decoupled(new RetimedPanelInput(side, 2)))
    val panelSeed = Vec(16, Decoupled(new RetimedRowData(side)))
    val clear = if (zeroState) Some(Vec(16, Decoupled(UInt(2.W)))) else None
    val rowRead = Vec(16, Decoupled(new RetimedRow(side)))
    val rowResponse = Flipped(Vec(16, Decoupled(new RetimedRowData(side))))
    val result = Decoupled(new StateOutputRow(side))
    val busy = Output(Bool())
    val done = Output(Bool())
    val issued = Output(UInt(16.W))
    val frames = Output(UInt(2.W))
    val released = Output(Valid(UInt(log2Ceil(t).W)))
  })
  val active = RegInit(false.B)
  val command = Reg(new StateOutputCommand)
  val seedRows = (if (zeroState) 0 else if (residentZ) 2 else 3) * t
  def physicalContext(x: UInt): UInt =
    if (residentZ) Mux(x === 2.U, Cat(command.half, 1.U(1.W)), Cat(x(0), 0.U(1.W))) else x
  val seedCount = RegInit(0.U(log2Ceil(3 * t + 1).W))
  val tokenCount = RegInit(0.U(log2Ceil(t + 1).W))
  val seeds = Module(new Queue(new StateOutputSeed(side), 2, pipe = false, flow = false))
  val seedCaptured = RegInit(0.U(4.W))
  val seeded = RegInit(VecInit(Seq.fill(16)(0.U(3.W))))
  val zeroPending = if (zeroState) Some(RegInit(VecInit(Seq.fill(16)(0.U(2.W))))) else None
  val live = RegInit(VecInit(Seq.fill(2)(false.B)))
  val data = Reg(Vec(2, new StateOutputColumn(side)))
  val captured = RegInit(VecInit(Seq.fill(2)(0.U(16.W))))
  val writeHead = RegInit(false.B)
  val heads = RegInit(VecInit(Seq.fill(16)(false.B)))
  val phases = RegInit(VecInit(Seq.fill(16)(0.U(2.W))))
  val finished = RegInit(VecInit(Seq.fill(16)(0.U(3.W))))
  val request = RegInit(0.U(log2Ceil(3 * t + 1).W))
  val requested = RegInit(0.U(4.W))
  val rowTags = Module(new Queue(UInt(rowBits.W), 4, pipe = false, flow = false))
  // Retain both result entries and all FP32 payload bits. A registered head
  // removes the read-pointer/data-mux prefix from downstream SRAM control.
  val results = if (registeredResultHead) {
    val queue = Module(new RegisteredPayloadQueue(new StateOutputRow(side), 2))
    queue.suggestName("results")
    queue.io
  } else {
    val queue = Module(new Queue(new StateOutputRow(side), 2, pipe = false, flow = false))
    queue.suggestName("results")
    queue.io
  }
  io.result <> results.deq
  io.start.ready := !active && io.oldStateReleased && !results.deq.valid && !seeds.io.deq.valid &&
    !rowTags.io.deq.valid
  io.busy := active
  io.done := false.B
  when(io.start.fire) {
    active := true.B
    command := io.start.bits
    seedCount := 0.U
    tokenCount := 0.U
    seedCaptured := 0.U
    writeHead := false.B
    request := 0.U
    requested := 0.U
    for (p <- 0 until 16) {
      seeded(p) := (if (residentZ) 4 else 0).U
      heads(p) := false.B
      phases(p) := 0.U
      finished(p) := 0.U
    }
    zeroPending.foreach(x => for (p <- 0 until 16) x(p) := 3.U)
    for (f <- 0 until 2) {
      live(f) := false.B
      captured(f) := 0.U
    }
  }
  seeds.io.enq.valid := active && seedCount < seedRows.U && io.seed.valid
  seeds.io.enq.bits := io.seed.bits
  io.seed.ready := active && seedCount < seedRows.U && seeds.io.enq.ready
  val expectedSeed =
    if (groupedSeeds)
      Cat(
        seedCount(log2Ceil(side)),
        seedCount(log2Ceil(2 * t) - 1, log2Ceil(side) + 1),
        seedCount(log2Ceil(side) - 1, 0)
      )
    else seedCount
  when(io.seed.fire) {
    assert(io.seed.bits.index === expectedSeed)
    seedCount := seedCount + 1.U
  }
  val seed = seeds.io.deq
  val seedGroup = seed.bits.index(log2Ceil(t) - 1, log2Ceil(side))
  val seedContext = seed.bits.index(rowBits - 1, log2Ceil(t))
  val seedRow = seed.bits.index(log2Ceil(side) - 1, 0)
  val seedFires = Wire(Vec(4, Bool()))
  for (c <- 0 until 4)
    seedFires(c) := (0 until 4).map(r => io.panelSeed(4 * r + c).fire).reduce(_ || _)
  seed.ready := (seedCaptured | seedFires.asUInt).andR
  when(seed.valid) { seedCaptured := seedCaptured | seedFires.asUInt }
  when(seed.fire) { seedCaptured := 0.U }

  io.column.ready := active && tokenCount < t.U && !live(writeHead)
  when(io.column.fire) {
    assert(io.column.bits.token === tokenCount)
    data(writeHead) := io.column.bits
    live(writeHead) := true.B
    captured(writeHead) := 0.U
    tokenCount := tokenCount + 1.U
    writeHead := !writeHead
  }
  io.released.valid := false.B
  io.released.bits := 0.U
  for (f <- 0 until 2) when(live(f) && captured(f).andR) {
    live(f) := false.B
    io.released.valid := true.B
    io.released.bits := data(f).token
  }
  // At most one token per panel can finish each cycle, in identical token order.
  assert(!(live(0) && captured(0).andR && live(1) && captured(1).andR))
  io.frames := PopCount(live)
  io.issued := VecInit(io.input.map(_.fire)).asUInt
  val captureFires = Wire(Vec(2, UInt(16.W)))
  for (f <- 0 until 2) {
    captureFires(f) := VecInit(
      (0 until 16).map(p => io.input(p).fire && phases(p) === 2.U && heads(p) === f.U)
    ).asUInt
    when(live(f) && !captured(f).andR) { captured(f) := captured(f) | captureFires(f) }
  }
  for (p <- 0 until 16) {
    val r = p / 4
    val c = p % 4
    val h = heads(p)
    val phase = phases(p)
    val ps = io.panelSeed(p)
    ps.valid := active && seed.valid && seedGroup === r.U && !seedCaptured(c)
    ps.bits.context := physicalContext(seedContext)
    ps.bits.row := seedRow
    for (j <- 0 until side) ps.bits.values(j) := seed.bits.values(c * side + j)
    when(ps.fire && seedRow === (side - 1).U) { seeded(p) := seeded(p) | (1.U(3.W) << seedContext) }
    if (zeroState) {
      val clear = io.clear.get(p)
      val first = zeroPending.get(p)(0)
      clear.valid := active && zeroPending.get(p).orR
      clear.bits := Mux(first, 0.U, 2.U)
      when(clear.fire) {
        zeroPending.get(p) := Mux(first, 2.U, 0.U)
        seeded(p) := seeded(p) | Mux(first, 1.U, 2.U)
      }
    }
    val in = io.input(p)
    in.valid := active && live(h) && !captured(h)(p) && seeded(p)(phase) && io.panelEnable(p)
    in.bits.os := true.B
    in.bits.context := physicalContext(phase)
    in.bits.tag := data(h).token(1, 0)
    for (i <- 0 until side) {
      in.bits.north(i) := data(h).e(c * side + i)
      in.bits.west(i) := Mux(
        phase === 0.U,
        data(h).n(r * side + i),
        Mux(phase === 1.U, data(h).n(t + r * side + i), data(h).m(r * side + i))
      )
    }
    when(in.fire) {
      when(data(h).token === (t - 1).U) { finished(p) := finished(p) | (1.U(3.W) << phase) }
      when(phase === 2.U) {
        phases(p) := 0.U
        heads(p) := !h
      }.otherwise { phases(p) := phase + 1.U }
    }
  }
  // Four independent row requests reserve one tag until their last acceptance.
  // Returned rows stay in each panel's bounded queue until the full row can be
  // joined. No combinational ready path from result consumption to MAC issue.
  // The integrated panel has one response slot. Rotate row groups so a
  // different panel supplies each successive row; no extra queue or C32.
  // The counter remains dense, while tags carry the actual target row.
  val requestRole = request(rowBits - 1, log2Ceil(t))
  val orderedRole =
    if (outputFirst) Mux(requestRole === 0.U, 2.U, Mux(requestRole === 1.U, 0.U, 1.U))
    else requestRole
  val readIndex =
    if (residentZ) Cat(orderedRole, request(1, 0), request(log2Ceil(t) - 1, 2))
    else request(rowBits - 1, 0)
  val requestGroup = readIndex(log2Ceil(t) - 1, log2Ceil(side))
  val requestContext = readIndex(rowBits - 1, log2Ceil(t))
  val readFires = Wire(Vec(4, Bool()))
  for (c <- 0 until 4)
    readFires(c) := (0 until 4).map(r => io.rowRead(4 * r + c).fire).reduce(_ || _)
  rowTags.io.enq.valid := active && request < (3 * t).U && (requested | readFires.asUInt).andR
  rowTags.io.enq.bits := readIndex
  when(active) { requested := requested | readFires.asUInt }
  when(rowTags.io.enq.fire) {
    request := request + 1.U
    requested := 0.U
  }
  val tag = rowTags.io.deq.bits
  val group = tag(log2Ceil(t) - 1, log2Ceil(side))
  val returned = VecInit(
    (0 until 4).map(c => io.rowResponse(Cat(group, c.U(2.W))).valid)
  ).asUInt.andR
  results.enq.valid := rowTags.io.deq.valid && returned
  results.enq.bits.index := tag
  results.enq.bits.epoch := command.epoch
  results.enq.bits.half := command.half
  for (c <- 0 until 4; j <- 0 until side)
    results.enq.bits.values(c * side + j) := io.rowResponse(Cat(group, c.U(2.W))).bits.values(j)
  rowTags.io.deq.ready := results.enq.ready && returned
  for (p <- 0 until 16) {
    val r = p / 4
    val c = p % 4
    io.rowRead(p).valid := active && request < (3 * t).U && rowTags.io.enq.ready &&
      requestGroup === r.U && !requested(c) && finished(p)(requestContext)
    io.rowRead(p).bits.context := physicalContext(requestContext)
    io.rowRead(p).bits.row := readIndex(log2Ceil(side) - 1, 0)
    io.rowResponse(p).ready :=
      rowTags.io.deq.valid && group === r.U && returned && results.enq.ready
    when(io.rowResponse(p).fire) {
      assert(io.rowResponse(p).bits.context === physicalContext(tag(rowBits - 1, log2Ceil(t))))
      assert(io.rowResponse(p).bits.row === tag(log2Ceil(side) - 1, 0))
    }
  }
  when(io.result.fire && io.result.bits.index === ((if (outputFirst) 2 else 3) * t - 1).U) {
    assert(tokenCount === t.U && seedCount === seedRows.U && !live.asUInt.orR)
    active := false.B
    io.done := true.B
  }
}
