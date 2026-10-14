package deltaflow.frontend
import chisel3._
import chisel3.util._
import deltaflow.memory._

/** One physical Gram tile plus the shared 4*side+1 auxiliary lanes. No second main array. The
  * coefficient stream is the boundary for the next UW stage.
  */
class PreparedGramTile(val side: Int = 16) extends Module {
  val io = IO(new Bundle {
    val start = Flipped(Decoupled(UInt(8.W)))
    val gates = Flipped(Decoupled(Vec(4 * side, new GateToken)))
    val init = Flipped(Vec(16, Decoupled(new BankWrite(512, 16))))
    val bankEnable = Input(UInt(16.W))
    val jobEnable = Input(UInt(20.W))
    val result = Vec(4, Decoupled(new GramRow(side)))
    val prefix = Output(Vec(4 * side, UInt(32.W)))
    val busy = Output(Bool())
    val done = Output(Bool())
    val gramDone = Output(Bool())
    val gatesReady = Output(Bool())
    val gramCommit = Output(UInt(4.W))
    val fmaIssue = Output(UInt(4.W))
    val inverseIssue = Output(UInt(4.W))
    val gateIssue = Output(Bool())
    val mainIssues = Output(UInt(5.W))
    val reads = Output(UInt(16.W))
  })
  val gram = Module(new SharedGramTile(side))
  val preparation = Module(new GramPreparation(side))
  preparation.io.gates <> io.gates
  gram.io.init <> io.init
  gram.io.bankEnable := io.bankEnable
  gram.io.jobEnable := io.jobEnable
  gram.io.start.valid := io.start.valid && preparation.io.start.ready
  gram.io.start.bits := io.start.bits
  preparation.io.start.valid := io.start.valid && gram.io.start.ready
  preparation.io.start.bits := true.B
  io.start.ready := gram.io.start.ready && preparation.io.start.ready
  preparation.io.gram <> gram.io.result
  io.result <> preparation.io.result
  gram.io.hostRead.valid := false.B
  gram.io.hostRead.bits := 0.U
  gram.io.hostResponse.ready := true.B
  io.prefix := preparation.io.prefix
  io.busy := preparation.io.busy
  io.done := preparation.io.done
  when(io.done) { assert(!gram.io.busy) }
  io.gramDone := gram.io.done
  io.gatesReady := preparation.io.gatesReady
  io.gramCommit := VecInit(gram.io.result.map(_.fire)).asUInt
  io.fmaIssue := preparation.io.fmaIssue
  io.inverseIssue := preparation.io.inverseIssue
  io.gateIssue := preparation.io.gateIssue
  io.mainIssues := PopCount(VecInit(gram.io.issued.map(_.valid)))
  io.reads := gram.io.bankReads
}
