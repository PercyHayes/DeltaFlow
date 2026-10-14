package deltaflow.sa

import chisel3._
import chisel3.util._
import deltaflow.arithmetic.FloatArithmetic

class TileCommand(maxColumns: Int) extends Bundle {
  val triangular = Bool()
  val columns = UInt(log2Ceil(maxColumns + 1).W)
}
class TileCoefficient(side: Int) extends RowCoefficient(side) {
  val tileRow = UInt(2.W)
  val tileCol = UInt(2.W)
}

class TilePreparation(side: Int) extends Bundle {
  val active = Input(Bool())
  val gram = Input(Bool())
  val clear = Input(Bool())
  val north = Input(Vec(16, Vec(side, Valid(UInt(16.W)))))
  val west = Input(Vec(16, Vec(side, Valid(UInt(32.W)))))
  val accumulators = Output(Vec(16, Vec(side, Vec(side, UInt(32.W)))))
}

/** Four by four physical microtiles: full dense GEMM or four-block triangular solve. Every
  * inter-tile edge is bounded, tagged and independently backpressured. Mode/frame changes require
  * all work drained. Optional early frames permit one-time coefficient publication to unstarted
  * blocks while peers execute.
  */
class NativeTile(
    val side: Int = 16,
    val depth: Int = 2,
    val maxColumns: Int = 256,
    val withPreparation: Boolean = false,
    val earlyTriangular: Boolean = false
) extends Module {
  require(side >= 4 && side <= 16 && isPow2(side))
  require(depth >= 1 && depth <= 8 && maxColumns >= 1 && maxColumns <= 256)
  val tagBits = log2Ceil(maxColumns + 1)
  val io = IO(new Bundle {
    val coefficient = Flipped(Decoupled(new TileCoefficient(side)))
    val begin = if (earlyTriangular) Some(Flipped(Decoupled(UInt(tagBits.W)))) else None
    val blockReady = Output(UInt(16.W))
    val launches = Output(UInt(16.W))
    val command = Flipped(Decoupled(new TileCommand(maxColumns)))
    val operand = Flipped(Vec(4, Decoupled(new Panel(side, 16, tagBits))))
    val rhs = Flipped(Vec(4, Decoupled(new Panel(side, 32, tagBits))))
    val result = Vec(4, Decoupled(new Panel(side, 32, tagBits)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val invalidCommand = Output(Bool())
    val stalled = Output(UInt(16.W))
    val transfers = Output(UInt(16.W))
    val preparation = if (withPreparation) Some(new TilePreparation(side)) else None
  })
  val tiles = Seq.fill(4, 4)(Module(new ElasticMicroTile(side, tagBits, withPreparation)))
  val forks = Seq.fill(4)(Module(new PanelBroadcast(side, 4, depth, tagBits)))
  // Residuals arrive 2*side cycles before the next solved operand. Include
  // one credit-return cycle; shallow queues here periodically stall the wave.
  val partial = Seq.tabulate(4, 3)((i, j) =>
    Module(
      new Queue(
        new Panel(side, 32, tagBits),
        if (i > j) math.max(depth, 2 * side + 1) else depth,
        pipe = true
      )
    )
  )
  val rhsQueues = Seq.fill(4)(Module(new Queue(new Panel(side, 32, tagBits), depth, pipe = true)))
  val busy = RegInit(false.B)
  val triangular = RegInit(false.B)
  val columns = Reg(UInt(tagBits.W))
  val earlyFrame = RegInit(false.B)
  val rowLoaded = RegInit(VecInit(Seq.fill(16)(0.U(side.W))))
  val started = RegInit(VecInit(Seq.fill(16)(false.B)))
  val localReady = VecInit((0 until 16).map(i => rowLoaded(i).andR))
  io.blockReady := localReady.asUInt
  io.launches := VecInit(tiles.flatten.map(_.io.in.fire)).asUInt
  val issued = RegInit(VecInit(Seq.fill(4)(0.U(tagBits.W))))
  val retired = RegInit(VecInit(Seq.fill(4)(0.U(tagBits.W))))
  val drained =
    tiles.flatten.map(_.io.idle).reduce(_ && _) && forks.map(_.io.idle).reduce(_ && _) &&
      partial.flatten.map(_.io.count === 0.U).reduce(_ && _) &&
      rhsQueues
        .map(_.io.count === 0.U)
        .reduce(_ && _)
  val allConfigured = tiles.flatten.map(_.io.configured).reduce(_ && _)
  val triangularConfigured =
    (for (i <- 0 until 4; j <- 0 to i) yield tiles(i)(j).io.configured).reduce(_ && _)
  val preparing = io.preparation.map(_.active).getOrElse(false.B)
  val gramActive = io.preparation.map(p => p.active && p.gram).getOrElse(false.B)
  val legal = io.command.bits.columns > 0.U && io.command.bits.columns <= maxColumns.U
  io.invalidCommand := io.command.valid && !legal
  val destination = Cat(io.coefficient.bits.tileRow, io.coefficient.bits.tileCol)
  val rowBit = (1.U(side.W) << io.coefficient.bits.row)(side - 1, 0)
  val freshRow = !(rowLoaded(destination) & rowBit).orR
  io.coefficient.ready := !busy && drained && !gramActive
  if (earlyTriangular) {
    when(earlyFrame && busy) {
      io.coefficient.ready := !gramActive &&
        io.coefficient.bits.tileCol <= io.coefficient.bits.tileRow && !started(destination) &&
        freshRow && VecInit(tiles.flatten.map(_.io.idle))(destination)
    }
    val begin = io.begin.get
    begin.ready := !busy && drained && !gramActive && !io.coefficient.valid && begin.bits > 0.U &&
      begin.bits <= maxColumns.U
    when(begin.fire) {
      busy := true.B
      triangular := true.B
      earlyFrame := true.B
      columns := begin.bits
      for (i <- 0 until 4) {
        issued(i) := 0.U
        retired(i) := 0.U
      }
      for (i <- 0 until 16) {
        rowLoaded(i) := 0.U
        started(i) := false.B
      }
    }
  }
  when(io.coefficient.fire) { rowLoaded(destination) := rowLoaded(destination) | rowBit }
  io.command.ready := !busy && drained && !preparing && !io.coefficient.valid && legal &&
    Mux(io.command.bits.triangular, triangularConfigured, allConfigured) &&
    !io.begin
      .map(_.valid)
      .getOrElse(false.B)
  when(io.command.fire) {
    busy := true.B
    earlyFrame := false.B
    triangular := io.command.bits.triangular
    columns := io.command.bits.columns
    for (i <- 0 until 4) {
      issued(i) := 0.U
      retired(i) := 0.U
    }
  }
  for (i <- 0 until 4; j <- 0 until 4) {
    val tile = tiles(i)(j)
    io.preparation.foreach { p =>
      val q = tile.io.preparation.get
      q.active := p.active && !(earlyFrame && busy && localReady(i * 4 + j))
      q.gram := p.gram
      q.clear := p.clear
      q.north := p.north(i * 4 + j)
      q.west := p.west(i * 4 + j)
      p.accumulators(i * 4 + j) := q.accumulators
      when(p.active && (!earlyFrame || p.gram || p.clear)) { assert(!busy && drained) }
      when(q.active) { assert(tile.io.idle) }
    }
    tile.io.coefficient.valid := io.coefficient.fire && io.coefficient.bits.tileRow === i.U &&
      io.coefficient.bits.tileCol === j.U
    tile.io.coefficient.bits.row := io.coefficient.bits.row
    tile.io.coefficient.bits.values := io.coefficient.bits.values
    when(tile.io.coefficient.valid) { assert(tile.io.coefficient.ready) }
    tile.io.in.valid := false.B
    tile.io.in.bits := 0.U.asTypeOf(tile.io.in.bits)
    tile.io.out.ready := false.B
    when(tile.io.in.fire) {
      started(i * 4 + j) := true.B
      when(earlyFrame) { assert(localReady(i * 4 + j)) }
    }
  }
  for (i <- 0 until 4; j <- 0 until 3) {
    partial(i)(j).io.enq.valid := false.B
    partial(i)(j).io.enq.bits := tiles(i)(j).io.out.bits
    partial(i)(j).io.deq.ready := false.B
  }
  for (i <- 0 until 4) {
    val fork = forks(i)
    fork.io.mask := Mux(triangular, ((15 << i) & 15).U, 15.U)
    fork.io.in.valid := false.B
    fork.io.in.bits := 0.U.asTypeOf(fork.io.in.bits)
    for (r <- 0 until 4) fork.io.out(r).ready := false.B
    io.operand(i).ready := false.B
    rhsQueues(i).io.enq.valid := busy && triangular && issued(i) < columns && io.rhs(i).valid
    rhsQueues(i).io.enq.bits := io.rhs(i).bits
    io.rhs(i).ready := busy && triangular && issued(i) < columns && rhsQueues(i).io.enq.ready
    rhsQueues(i).io.deq.ready := false.B
    io.result(i).valid := false.B
    io.result(i).bits := 0.U.asTypeOf(io.result(i).bits)
    when(busy && !triangular) {
      fork.io.in.valid := issued(i) < columns && io.operand(i).valid
      fork.io.in.bits := io.operand(i).bits
      io.operand(i).ready := issued(i) < columns && fork.io.in.ready
    }
    val inputFire = Mux(triangular, io.rhs(i).fire, io.operand(i).fire)
    val inputTag = Mux(triangular, io.rhs(i).bits.tag, io.operand(i).bits.tag)
    when(inputFire) {
      assert(inputTag === issued(i), "input columns must be ordered")
      issued(i) := issued(i) + 1.U
    }
    when(io.result(i).fire) {
      assert(io.result(i).bits.tag === retired(i), "output column order changed")
      assert(retired(i) < columns)
      retired(i) := retired(i) + 1.U
    }
  }
  for (i <- 0 until 4; j <- 0 until 4) {
    val tile = tiles(i)(j)
    val op = forks(j).io.out(i)
    // Dense GEMM: B_j is multicast down column j, partial sums move east.
    when(busy && !triangular) {
      if (j == 0) {
        tile.io.in.valid := op.valid
        tile.io.in.bits.tag := op.bits.tag
        tile.io.in.bits.operand := op.bits.values
        tile.io.in.bits.initial := VecInit(Seq.fill(side)(0.U(32.W)))
        op.ready := tile.io.in.ready
      } else {
        val west = partial(i)(j - 1).io.deq
        tile.io.in.valid := op.valid && west.valid
        tile.io.in.bits.tag := op.bits.tag
        tile.io.in.bits.operand := op.bits.values
        tile.io.in.bits.initial := west.bits.values
        op.ready := tile.io.in.ready && west.valid
        west.ready := tile.io.in.ready && op.valid
        when(tile.io.in.fire) { assert(op.bits.tag === west.bits.tag) }
      }
    }
    if (j <= i) {
      val west = if (j == 0) rhsQueues(i).io.deq else partial(i)(j - 1).io.deq
      when(busy && triangular && (!earlyFrame || localReady(i * 4 + j))) {
        if (i == j) {
          tile.io.in.valid := west.valid
          tile.io.in.bits.tag := west.bits.tag
          tile.io.in.bits.operand := VecInit(west.bits.values.map(FloatArithmetic.narrow))
          tile.io.in.bits.initial := VecInit(Seq.fill(side)(0.U(32.W)))
          west.ready := tile.io.in.ready
        } else {
          tile.io.in.valid := west.valid && op.valid
          tile.io.in.bits.tag := west.bits.tag
          tile.io.in.bits.operand := op.bits.values
          tile.io.in.bits.initial := west.bits.values
          west.ready := tile.io.in.ready && op.valid
          op.ready := tile.io.in.ready && west.valid
          when(tile.io.in.fire) { assert(op.bits.tag === west.bits.tag) }
        }
      }
    }
    if (j < 3) {
      when(busy && (!triangular || (j < i).B)) {
        partial(i)(j).io.enq.valid := tile.io.out.valid
        tile.io.out.ready := partial(i)(j).io.enq.ready
      }
    } else {
      when(busy && !triangular) {
        io.result(i).valid := tile.io.out.valid
        io.result(i).bits := tile.io.out.bits
        tile.io.out.ready := io.result(i).ready
      }
    }
    if (i == j) {
      when(busy && triangular) {
        forks(i).io.in.valid := tile.io.out.valid
        forks(i).io.in.bits.tag := tile.io.out.bits.tag
        forks(i).io.in.bits.values := VecInit(tile.io.out.bits.values.map(FloatArithmetic.narrow))
        tile.io.out.ready := forks(i).io.in.ready
        io.result(i).valid := forks(i).io.out(i).valid
        io.result(i).bits.tag := forks(i).io.out(i).bits.tag
        for (r <- 0 until side) io.result(i).bits.values(r) := forks(i).io.out(i).bits.values(r)
        forks(i).io.out(i).ready := io.result(i).ready
      }
    }
  }
  val finish = busy && retired.map(_ === columns).reduce(_ && _) && drained
  when(finish) { busy := false.B }
  io.done := RegNext(finish, false.B)
  io.busy := busy
  io.stalled := VecInit(tiles.flatten.map(_.io.stalled)).asUInt
  io.transfers := VecInit(tiles.flatten.map(_.io.out.fire)).asUInt
}
