# Chipyard Direct AXI4 integration

DeltaFlow uses [Chipyard](https://github.com/ucb-bar/chipyard) through [FSA's integration](https://github.com/VCA-EPFL/chipyard-fsa), following [FSA](https://github.com/VCA-EPFL/FSA)'s Direct AXI4 organization. There is one accelerator system for all operation modes. The payload path does not route sixteen HBM streams through the host register bus.

## Regenerate the complete system

Install the dependency using [Environment](environment.md), then:

```bash
source scripts/env.sh
bash scripts/elaborate_chipyard.sh
bash scripts/build_sim.sh .local/chipyard/DeltaFlowHarness.sv .local/sim_fresh
python scripts/run_sim.py .local/examples/gemm --binary .local/sim_fresh/chipyard_hbm
```

`setup_chipyard.py` adds the two release Scala source directories to the dependency's `lazy val chipyard` generator project. It does not rewrite FSA source files. The dependency build uses `DELTAFLOW_REPO_ROOT` to select this release, so export that environment before elaborating. The selected compact PE is explicit in `DirectAXI4.scala`.

The generator uses `deltaflow.integration.DeltaFlow16Config` and `deltaflow.integration.DeltaFlowHarness`. The former selects four physical 64 × 64 tiles. The standalone `PublicElaborate core` entry emits only the four-tile core; it does not replace complete-system integration or STA.

The supplied `generated/DeltaFlowHarness.sv` is the retained full-system RTL. It includes the host bus, MMIO, direct-port binders and all four tiles. Fresh elaboration is required after changing Scala or the integration configuration. Always build and test the new generated file explicitly.

## Interfaces

`AXI4DeltaFlow` has an AXI4RegisterNode at 0x8000 with a 4-KiB window and 4-byte register beats. The external 32-bit host AXI interface enters Chipyard's FBUS, then the register bridge. There is no RISC-V CPU in this accelerator harness.

Sixteen AXI4MasterNode payload lanes are exported directly using Chipyard IO and harness binders. Each lane carries 256-bit data and burst control. Tile t owns global lanes `4*t` through `4*t+3`. Each lane's 4-GiB address window starts at `(lane+1)<<32`; instruction addresses are offsets within these independent windows. Four heads can use the same offsets with different lane contents.

The harness supplies one clock/reset domain and reports a nominal 1,000 MHz reference clock. The clock declaration is a configuration parameter, not a timing-closure directive or a physical frequency guarantee. Synthesis and STA constraints are provided by [the timing flow](synthesis.md).

## Board integration

Connect the external host AXI interface to the host/CPU register master and the sixteen payload ports to compatible HBM controller channels or an AXI interconnect. Preserve the tile bandwidth budget, ordering, backpressure and write responses. Provide the clock/reset and any physical clock-domain bridges required by the platform. Upload and download tensors through HBM; the register transport only submits metadata and drains completions.

The release does not include a vendor HBM controller, PHY or board-specific pinout. For simulation, the C++ harness supplies the sixteen AXI service models and validates actual payload transfers through the same external interfaces.
