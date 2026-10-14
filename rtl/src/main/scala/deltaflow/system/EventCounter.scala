package deltaflow.system

import chisel3._
import chisel3.util._

/** Exact modulo counter arithmetic for increments smaller than one byte.
  *
  * Only the low byte receives the event count. A higher byte increments when the low addition
  * overflows and every intervening old byte is all ones. Those carry qualifications are parallel
  * reductions, not a serial chain of byte additions. No pipeline latency or extra counter state is
  * introduced.
  */
object EventCounterAdd {
  def apply(value: UInt, increment: UInt): UInt = {
    val width = value.getWidth
    require(width > 0 && width % 8 == 0)
    require(increment.getWidth > 0 && increment.getWidth <= 8)

    val low = value(7, 0) +& increment
    val bytes = Wire(Vec(width / 8, UInt(8.W)))
    bytes(0) := low(7, 0)
    for (index <- 1 until width / 8) {
      val propagate =
        if (index == 1) true.B
        else value(8 * index - 1, 8).andR
      val carry = low(8) && propagate
      bytes(index) := value(8 * index + 7, 8 * index) + carry.asUInt
    }
    bytes.asUInt
  }
}

/** Array-external statistics counter; its value changes on the original edge.
  *
  * With registered increments, accumulated excludes the most recent edge and pending contains that
  * edge's events. Exposing accumulated + pending keeps the original visible count exactly, while
  * event reduction ends at a small register instead of feeding the counter addition on the same
  * path.
  */
class EventCounter(val maxIncrement: Int, val registerIncrement: Boolean = false) extends Module {
  require(maxIncrement >= 1 && maxIncrement <= 255)
  val io = IO(new Bundle {
    val increment = Input(UInt(log2Ceil(maxIncrement + 1).W))
    val value = Output(UInt(64.W))
  })

  val value = RegInit(0.U(64.W))
  if (registerIncrement) {
    val pending = RegInit(0.U(log2Ceil(maxIncrement + 1).W))
    pending := io.increment
    val visible = EventCounterAdd(value, pending)
    value := visible
    io.value := visible
  } else {
    value := EventCounterAdd(value, io.increment)
    io.value := value
  }
}

/** Capture each tile's sixteen issue events before combining tile counts.
  *
  * All four small counts describe the same edge. The accumulated count excludes that edge, so
  * adding the captured counts preserves the original visible modulo-64-bit count without changing
  * computation or MMIO sampling cycles.
  */
class GroupedEventCounter extends Module {
  val io = IO(new Bundle {
    val events = Input(Vec(64, Bool()))
    val value = Output(UInt(64.W))
  })

  val pending = RegInit(VecInit(Seq.fill(4)(0.U(5.W))))
  for (group <- 0 until 4) {
    pending(group) := PopCount(io.events.slice(16 * group, 16 * (group + 1)))
  }
  val first = pending(0) +& pending(1)
  val second = pending(2) +& pending(3)
  val increment = first +& second
  val accumulated = RegInit(0.U(64.W))
  val visible = EventCounterAdd(accumulated, increment)
  accumulated := visible
  io.value := visible
}

/** Capture DMA handshakes before reducing all sixteen ports.
  *
  * The two counts refer to the same edge. Their sum is added to accumulated both for the next state
  * and for the visible value, preserving the original count on every edge. This adds three
  * statistics bits per counter, with no payload buffering or change to DMA admission or MMIO
  * sampling.
  */
class GroupedDmaEventCounter extends Module {
  val io = IO(new Bundle {
    val events = Input(Vec(16, Bool()))
    val value = Output(UInt(64.W))
  })

  val pending = RegInit(VecInit(Seq.fill(2)(0.U(4.W))))
  for (group <- 0 until 2) {
    pending(group) := PopCount(io.events.slice(8 * group, 8 * (group + 1)))
  }
  val increment = pending(0) +& pending(1)
  val accumulated = RegInit(0.U(64.W))
  val visible = EventCounterAdd(accumulated, increment)
  accumulated := visible
  io.value := visible
}
