# Chipyard integration

DeltaFlow uses Chipyard's AXI4 register nodes, I/O binders and harness binders to connect the accelerator to the host and external memory. The host interface submits instructions and reads completions. Sixteen separate AXI4 ports carry tensor data through DMA. Chipyard describes these mechanisms in its [register-router guide](https://chipyard.readthedocs.io/en/stable/TileLink-Diplomacy-Reference/Register-Router.html) and [I/O and harness binder guide](https://chipyard.readthedocs.io/en/stable/Customization/IOBinders.html).

## Generate the complete system

Install the build environment from [Environment](environment.md#chisel-and-chipyard), then run these commands from the project root:

```bash
source scripts/env.sh
bash scripts/elaborate_chipyard.sh
bash scripts/build_sim.sh .local/chipyard/DeltaFlowHarness.sv .local/sim_fresh
python examples/generate.py gemm --m 256 --n 256 --k 2048 --out .local/examples/gemm
python scripts/run_sim.py .local/examples/gemm --binary .local/sim_fresh/chipyard_hbm
```

`scripts/elaborate_chipyard.sh` calls `scripts/setup_chipyard.py` to add `rtl/src/main/scala` and `rtl/chipyard/src/main/scala` to Chipyard's Scala build. `DELTAFLOW_REPO_ROOT`, set by `scripts/env.sh`, tells that build where to find the DeltaFlow sources. The generator runs `DeltaFlow16Config` with `DeltaFlowHarness`, and firtool writes `.local/chipyard/DeltaFlowHarness.sv`.

The main modules are:

| Module | Role |
| --- | --- |
| `FourTileSystem` | Four compute tiles, DMA engines and instruction dispatch |
| `AXI4DeltaFlow` | Instruction registers, completion FIFO and DMA AXI4 nodes |
| `DeltaFlow16Config` | Four-tile configuration and external interface bindings |
| `DeltaFlowHarness` | Chipyard host interface, clock/reset and external memory ports |

`DirectAXI4.scala` selects the compact PE implementation. `generated/DeltaFlowHarness.sv` is the full-system SystemVerilog used by the default simulation command. After editing Scala or the configuration, regenerate the RTL and pass the new `.sv` file to `scripts/build_sim.sh` as shown above. `PublicElaborate core` generates `FourTileSystem` for standalone core work.

## Host interface

`AXI4DeltaFlow` creates an `AXI4RegisterNode` at address 0x8000 with a 4-KiB register window and 4-byte register transfers. The external 32-bit host AXI interface reaches it through Chipyard's front bus and register bridge.

The host writes four instruction words, submits them, then polls the completion FIFO. [Programming](programming.md#mmio) lists the register offsets and completion fields. An external host or CPU drives this interface.

## Memory interfaces

Sixteen AXI4 master ports handle DMA reads and writes. Each port has 256-bit data and burst control. Tile t uses global lanes `4*t` through `4*t+3`. I/O binders export the ports from the Chipyard system, and harness binders connect them to the outer harness.

Each lane has an independent 4-GiB address window starting at `(lane+1)<<32`. DMA instructions specify an offset within that window. Four heads can use identical offsets because their input data and state occupy different lane windows.

Each tile has a 128-byte/cycle aggregate payload budget across its four lanes; all four tiles total 512 bytes/cycle. The C++ testbench services these ports with the AXI memory model described in [Simulation](simulation.md#axihbm-service-model).

## Clock and platform connections

`DeltaFlowHarness` uses one clock/reset domain and configures a 1,000-MHz reference clock. The [STA flow](synthesis.md#full-system-sta) applies a 1.0-ns clock period to the generated netlist.

For a board implementation, connect the host AXI interface to a register master and the sixteen DMA ports to HBM controller channels or an AXI interconnect. Supply clock/reset and the clock-domain bridges required by the platform. Keep AXI ordering, backpressure and write responses intact. The host submits instructions through registers and transfers tensor data through HBM.
