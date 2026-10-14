package deltaflow

import circt.stage.ChiselStage
import deltaflow.system.FourTileSystem
import deltaflow.sa.RetimedWavePE

/** Small arithmetic probes and the unified four-tile core share the same RTL. */
object PublicElaborate extends App {
  require(args.length == 2, "usage: PublicElaborate <core|pe> <output-directory>")
  require(Set("core", "pe").contains(args(0)))
  ChiselStage.emitSystemVerilogFile(
    if (args(0) == "core") new FourTileSystem(16, compactPe = true)
    else
      new RetimedWavePE(
        tagBits = 3,
        compactWestMetadata = true,
        sharedMagnitude = true,
        freeInvalidPayload = true,
        packedAligned = true,
        compactClassification = true,
        splitAlignment = true,
        narrowAddition = true,
        freeForwardPayload = true
      ),
    Array("--target-dir", args(1)),
    Array("--disable-all-randomization", "--strip-debug-info")
  )
}
