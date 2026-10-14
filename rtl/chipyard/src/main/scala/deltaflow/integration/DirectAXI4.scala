package deltaflow.integration

import chisel3._
import chisel3.util._
import chisel3.reflect.DataMirror
import org.chipsalliance.cde.config.{Config, Field, Parameters}
import org.chipsalliance.diplomacy.lazymodule.{LazyModule, LazyModuleImp}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.diplomacy.{AddressSet, IdRange, RegionType, TransferSizes, InModuleBody}
import freechips.rocketchip.regmapper.{RegField, RegReadFn, RegWriteFn}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.prci.{ClockSinkNode, ClockSinkParameters}
import testchipip.util.ClockedIO
import chipyard._
import chipyard.iobinders._
import chipyard.harness._
import deltaflow.system.{EventCounter, FourTileSystem, GroupedDmaEventCounter, GroupedEventCounter}

case object DeltaFlowSide extends Field[Int](16)
class Monitor extends Bundle {
  val cycles = UInt(64.W)
  val waves = UInt(64.W)
  val reads = UInt(64.W)
  val writes = UInt(64.W)
  val issued = UInt(64.W)
  val completion = UInt(32.W)
  val completionFire = Bool()
  val idle = Bool()
  val fault = UInt(4.W)
  val gdn = UInt(4.W)
  val submitted = UInt(32.W)
}

/** Real register bus endpoint; all numerical payloads still use the 16 DMA masters. */
class AXI4DeltaFlow(implicit p: Parameters) extends LazyModule {
  val configNode = AXI4RegisterNode(address = AddressSet(0x8000, 0xfff), beatBytes = 4)
  val memNode = AXI4MasterNode(
    (0 until 16).map(i =>
      AXI4MasterPortParameters(
        masters = Seq(AXI4MasterParameters(name = s"deltaflow_$i", id = IdRange(0, 2)))
      )
    )
  )
  lazy val module: LazyModuleImp { val monitor: Monitor } = new LazyModuleImp(this) {
    val monitor = IO(Output(new Monitor))
    // The release uses the selected compact PE implementation.
    val core = Module(new FourTileSystem(p(DeltaFlowSide), compactPe = true))
    val staging = Reg(Vec(4, UInt(32.W)))
    val present = RegInit(0.U(4.W))
    val badSubmit = RegInit(false.B)
    val submitted = RegInit(0.U(32.W))
    val completions = Module(new Queue(UInt(32.W), 64))
    val c = core.io.completion.bits
    val event = Cat(1.U(1.W), 0.U(1.W), c.error, c.dma, c.port, c.tile, c.opcode, c.tag)
    completions.io.enq.valid := core.io.completion.valid
    completions.io.enq.bits := event
    core.io.completion.ready := completions.io.enq.ready
    core.io.instruction.bits := staging.asUInt
    core.io.instruction.valid := false.B
    completions.io.deq.ready := false.B
    // Statistics retain their original visible count. Registered event counts
    // end the handshake/reduction path; accumulated + pending exposes the
    // original count without delaying MMIO or changing compute/DMA admission.
    val cycleCounter = Module(new EventCounter(1))
    val waveCounter = Module(new GroupedEventCounter)
    val readCounter = Module(new GroupedDmaEventCounter)
    val writeCounter = Module(new GroupedDmaEventCounter)
    val issued = core.io.issued.asUInt
    cycleCounter.io.increment := 1.U
    waveCounter.io.events := issued.asTypeOf(waveCounter.io.events)
    readCounter.io.events := VecInit(core.io.hbm.map(_.r.fire))
    writeCounter.io.events := VecInit(core.io.hbm.map(_.w.fire))
    val cycles = cycleCounter.io.value
    val waves = waveCounter.io.value
    val reads = readCounter.io.value
    val writes = writeCounter.io.value
    val status = Cat(
      0.U(18.W),
      badSubmit,
      core.io.faultMask,
      completions.io.deq.valid,
      present,
      core.io.instruction.ready,
      core.io.idle
    )
    val stageFields = (0 until 4).map(i =>
      (4 * i) -> Seq(
        RegField.w(
          32,
          RegWriteFn((valid: Bool, data: UInt) => {
            when(valid) {
              staging(i) := data
              present := present | (1 << i).U
            }
            true.B
          })
        )
      )
    )
    configNode.regmap(
      (stageFields ++ Seq(
        0x10 -> Seq(
          RegField.w(
            32,
            RegWriteFn((valid: Bool, data: UInt) => {
              val legal = present.andR && data === 1.U
              core.io.instruction.valid := valid && legal
              when(valid && !legal) { badSubmit := true.B }
              when(core.io.instruction.fire) {
                present := 0.U
                submitted := submitted + 1.U
              }
              !legal || core.io.instruction.ready
            })
          )
        ),
        0x14 -> Seq(RegField.r(32, status)),
        0x18 -> Seq(RegField.r(32, submitted)),
        0x20 -> Seq(
          RegField.r(
            32,
            RegReadFn((ready: Bool) => {
              completions.io.deq.ready := ready
              (true.B, Mux(completions.io.deq.valid, completions.io.deq.bits, 0.U(32.W)))
            })
          )
        ),
        0x28 -> Seq(RegField.r(32, cycles(31, 0))),
        0x2c -> Seq(RegField.r(32, cycles(63, 32))),
        0x30 -> Seq(RegField.r(32, waves(31, 0))),
        0x34 -> Seq(RegField.r(32, waves(63, 32))),
        0x38 -> Seq(RegField.r(32, reads(31, 0))),
        0x3c -> Seq(RegField.r(32, reads(63, 32))),
        0x40 -> Seq(RegField.r(32, writes(31, 0))),
        0x44 -> Seq(RegField.r(32, writes(63, 32)))
      )): _*
    )
    monitor.cycles := cycles
    monitor.waves := waves
    monitor.reads := reads
    monitor.writes := writes
    monitor.issued := issued
    monitor.completion := event
    monitor.completionFire := core.io.completion.fire
    monitor.idle := core.io.idle
    monitor.fault := core.io.faultMask
    monitor.gdn := core.io.gdnMode
    monitor.submitted := submitted
    for (((a, edge), i) <- memNode.out.zipWithIndex) {
      val b = core.io.hbm(i)
      // Disjoint 4GiB windows. Current DMA offsets are <4GiB; no channel striping.
      val base = BigInt(i + 1) << 32
      a.ar.valid := b.ar.valid
      b.ar.ready := a.ar.ready
      a.aw.valid := b.aw.valid
      b.aw.ready := a.aw.ready
      for ((x, y) <- Seq((a.ar.bits, b.ar.bits), (a.aw.bits, b.aw.bits))) {
        x := 0.U.asTypeOf(x)
        x.addr := base.U + y.addr
        x.id := y.id
        x.len := y.len
        x.size := y.size
        x.burst := y.burst
        x.lock := y.lock
        x.cache := y.cache
        x.prot := y.prot
        x.qos := y.qos
      }
      when(b.ar.valid) { assert(!b.ar.bits.addr(47, 32).orR, "HBM offset exceeds 4GiB window") }
      when(b.aw.valid) { assert(!b.aw.bits.addr(47, 32).orR, "HBM offset exceeds 4GiB window") }
      a.w.valid := b.w.valid
      b.w.ready := a.w.ready
      a.w.bits := 0.U.asTypeOf(a.w.bits)
      a.w.bits.data := b.w.bits.data
      a.w.bits.strb := b.w.bits.strb
      a.w.bits.last := b.w.bits.last
      b.r.valid := a.r.valid
      a.r.ready := b.r.ready
      b.r.bits.data := a.r.bits.data
      b.r.bits.id := a.r.bits.id
      b.r.bits.resp := a.r.bits.resp
      b.r.bits.last := a.r.bits.last
      b.b.valid := a.b.valid
      a.b.ready := b.b.ready
      b.b.bits.id := a.b.bits.id
      b.b.bits.resp := a.b.bits.resp
    }
  }
}

trait CanHaveDeltaFlowDirectAXI4 { this: BaseSubsystem =>
  val fbus = locateTLBusWrapper(FBUS)
  val dfDomain = fbus.generateSynchronousDomain("deltaflow")
  val df = dfDomain { LazyModule(new AXI4DeltaFlow) }
  fbus.coupleTo("deltaflow") {
    df.configNode := AXI4UserYanker() := TLToAXI4() :=
      TLFragmenter(4, fbus.blockBytes, holdFirstDeny = true) := TLWidthWidget(fbus.beatBytes) := _
  }
  val dfSlave = AXI4SlaveNode(
    (0 until 16).map(i =>
      AXI4SlavePortParameters(
        slaves = Seq(
          AXI4SlaveParameters(
            address = Seq(AddressSet(BigInt(i + 1) << 32, BigInt("ffffffff", 16))),
            regionType = RegionType.UNCACHED,
            executable = false,
            supportsRead = TransferSizes(1, 512),
            supportsWrite = TransferSizes(1, 512),
            interleavedId = Some(0)
          )
        ),
        beatBytes = 32
      )
    )
  )
  dfSlave :=* df.memNode
  val dfPorts = InModuleBody { dfSlave.makeIOs() }
  val domainMonitor = dfDomain {
    InModuleBody {
      val x = IO(Output(new Monitor))
      x := df.module.monitor
      x
    }
  }
  val dfMonitor = InModuleBody {
    val x = IO(Output(new Monitor))
    x := domainMonitor
    x
  }
}
case class DeltaMemoryPort(getIO: () => ClockedIO[AXI4Bundle], index: Int)
    extends Port[ClockedIO[AXI4Bundle]]
case class DeltaMonitorPort(getIO: () => Monitor) extends Port[Monitor]
class WithDeltaFlowIO
    extends OverrideLazyIOBinder({ (system: CanHaveDeltaFlowDirectAXI4) =>
      {
        implicit val p: Parameters = GetSystemParameters(system)
        val clk = ClockSinkNode(Seq(ClockSinkParameters()))
        clk := system.asInstanceOf[HasTileLinkLocations].locateTLBusWrapper(FBUS).fixedClockNode
        InModuleBody {
          val ports = system.dfPorts.zipWithIndex.map { case (m, i) =>
            val out = IO(new ClockedIO(DataMirror.internal.chiselTypeClone[AXI4Bundle](m)))
              .suggestName(s"delta_hbm_$i")
            out.bits <> m
            out.clock := clk.in.head._1.clock
            DeltaMemoryPort(() => out, i)
          }
          val mon = IO(Output(new Monitor)).suggestName("delta_monitor")
          mon := system.dfMonitor
          (ports.toSeq :+ DeltaMonitorPort(() => mon), Nil)
        }
      }
    })

/** External AXI host and external memory model: both cross actual ChipTop IOs. */
class WithDeltaFlowHarnessIO
    extends HarnessBinder({
      case (th: HasHarnessInstantiators, port: DeltaMemoryPort, _) => {
        val io = IO(DataMirror.internal.chiselTypeClone[AXI4Bundle](port.io.bits))
          .suggestName(s"hbm_${port.index}")
        io <> port.io.bits
      }
      case (th: HasHarnessInstantiators, port: AXI4InPort, _) => {
        val io =
          IO(DataMirror.internal.chiselTypeClone[AXI4Bundle](port.io.bits)).suggestName("host")
        port.io.bits <> io
      }
      case (th: HasHarnessInstantiators, port: DeltaMonitorPort, _) => {
        val io = IO(Output(new Monitor)).suggestName("monitor")
        io := port.io
      }
    })
class WithDeltaFlow(side: Int)
    extends Config((site, here, up) => {
      case DeltaFlowSide => side
      case BuildSystem   => (p: Parameters) => new DigitalTop()(p) with CanHaveDeltaFlowDirectAXI4
    })
class DeltaFlowConfig(side: Int = 16)
    extends Config(
      new WithDeltaFlowHarnessIO ++ new WithDeltaFlowIO ++ new WithDeltaFlow(side) ++
        new WithAllClocksFromHarnessClockInstantiator ++ new WithHarnessBinderClockFreqMHz(1000) ++
        new chipyard.config.WithUniformBusFrequencies(1000) ++ new chipyard.WithFbusBeatBytes(4) ++
        new WithCustomSlavePort(32, 1) ++ new testchipip.serdes.WithNoSerialTL ++
        new chipyard.WithMBusErrorDevice ++ new chipyard.WithMBusZeroDevice ++ new WithNoMemPort ++
        new NoCoresConfig
    )
class DeltaFlow16Config extends DeltaFlowConfig(16)
class DeltaFlow4Config extends DeltaFlowConfig(4)

class DeltaFlowHarness(implicit val p: Parameters) extends Module with HasHarnessInstantiators {
  val success = WireInit(false.B)
  def referenceClockFreqMHz = 1000.0
  def referenceClock = clock
  def referenceReset = reset
  val lazyDuts = instantiateChipTops()
}
