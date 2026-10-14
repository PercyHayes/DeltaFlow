# DeltaFlow: Extending Systolic Arrays from GEMM to Gated DeltaNet Attention

[![License](https://img.shields.io/badge/License-Apache--2.0-blue)](LICENSE) [![RTL](https://img.shields.io/badge/RTL-Chisel-orange)](https://github.com/chipsalliance/chisel) [![Integration](https://img.shields.io/badge/Integration-Chipyard-green)](https://github.com/ucb-bar/chipyard)

DeltaFlow is a programmable systolic accelerator for GEMM and Gated DeltaNet (GDN) attention, written in Chisel and integrated with Chipyard.

<p align="center"><img src="docs/assets/gdn_execution_animated.svg" alt="GDN execution on the shared compute fabric" width="960"></p>

## 🗂️ Repository layout

```text
.
├── rtl/                         Chisel RTL and integration
│   ├── src/main/scala/deltaflow/ Compute, memory, DMA and instruction control
│   ├── chipyard/                AXI4 integration and simulation harness
│   └── memory_models/           SRAM simulation and synthesis models
├── generated/                   Four-tile SystemVerilog
├── software/deltaflow/          Instruction compiler, data layouts and MMIO API
├── examples/                    GEMM, prefill, decode and replay workloads
├── scripts/                     Build, simulation, synthesis and STA tools
├── docs/                        Architecture, programming and setup guides
│   └── assets/                  Figures used by the README and documentation
├── profiles/                    GPU and TPU traces
├── reference/                   Mapped netlist for STA
├── licenses/                    Third-party license texts
├── requirements.txt             Python dependencies
└── LICENSE                      Apache-2.0 license
```

## 🛠️ Setup

RTL simulation uses Linux, Python, NumPy, GCC/G++, Make and Verilator. See [Environment](docs/environment.md) for tool versions and installation instructions.

From the project root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
source scripts/env.sh
```

Use `generated/DeltaFlowHarness.sv` to run the examples below. To generate SystemVerilog from Chisel, follow [Build from Chisel](#-build-from-chisel).

## 🚀 RTL simulation

### Build the simulator

```bash
BUILD_JOBS=32 SIM_THREADS=8 bash scripts/build_sim.sh
```

The script compiles `generated/DeltaFlowHarness.sv` and the C++ testbench into `.local/sim/chipyard_hbm`. The simulator runs all four tiles, DMA engines and the Chipyard host interface, using the SRAM read/write models in `rtl/memory_models/`.

### Run the examples

We provide four simulation examples covering GEMM, GDN prefill, decode and replaySSM-style replay . For each example, `examples/generate.py` writes the instruction stream and HBM data files, and `scripts/run_sim.py` runs the simulator. The testbench compares output writes with a CPU numerical reference and reports cycles, DMA traffic and main-array utilization.

**GEMM:** each tile multiplies a 256 × 2048 matrix by a 2048 × 256 matrix.

```bash
python examples/generate.py gemm --m 256 --n 256 --k 2048 --out .local/examples/gemm
python scripts/run_sim.py .local/examples/gemm
```

**GDN prefill:** four heads, 1,024 tokens, Dk = Dv = 128 and chunk size 64.

```bash
python examples/generate.py prefill --tokens 1024 --out .local/examples/prefill
python scripts/run_sim.py .local/examples/prefill
```

**Decode:** process four successive decode tokens per head.

```bash
python examples/generate.py decode --decode-tokens 4 --out .local/examples/decode
python scripts/run_sim.py .local/examples/decode
```

**Replay:** restore a checkpoint, reconstruct the state for two accepted draft tokens, then execute the next decode token.

```bash
python examples/generate.py replay --accepted 2 --out .local/examples/replay
python scripts/run_sim.py .local/examples/replay
```

See [Simulation](docs/simulation.md) for input and output formats, AXI timing and workload options.

## 🏗️ Architecture and programming

GEMM, GDN chunk prefill, decode and cached-correction replay use the same four-tile accelerator and instruction set. Each tile has its own SRAM and HBM ports. A shared instruction stream controls which tiles execute each operation.

- Four 64 × 64 systolic tiles provide 16,384 main MACs per cycle.
- Each tile has sixteen 16 × 16 subarrays, 65 auxiliary FP32 lanes, 400 KiB of SRAM and four 256-bit AXI4 payload ports.
- Matrix operands use BF16; state and accumulation use FP32; GDN outputs use BF16.
- The Python compiler generates 128-bit instructions for compute, DMA and synchronization.

<p align="center"><img src="docs/assets/architecture.svg" alt="DeltaFlow system and compute-tile architecture" width="960"></p>

Each PE has one BF16 coefficient register (**COEF**) and four FP32 accumulation registers (**ACC0–ACC3**). GEMM uses output-stationary accumulation. GDN applies triangular transforms in weight-stationary mode, then computes state and output updates in output-stationary mode.

During GDN prefill, preparation for the next chunk overlaps the current chunk's state and output computations. Each head's FP32 state stays in SRAM between chunks. DMA writes the BF16 outputs after each chunk and the final FP32 state after the sequence.

The Chisel datapath is in [rtl/src/main/scala/deltaflow](rtl/src/main/scala/deltaflow), with [FourTileSystem.scala](rtl/src/main/scala/deltaflow/system/FourTileSystem.scala) as the accelerator top. [DirectAXI4.scala](rtl/chipyard/src/main/scala/deltaflow/integration/DirectAXI4.scala) connects it to Chipyard. The Python compiler and MMIO API are in [software/deltaflow](software/deltaflow).

Compile an instruction file with:

```bash
python -m deltaflow gemm --m 8192 --n 512 --k 2560 --output .local/programs/projection.hex
python -m deltaflow prefill --chunks 128 --output .local/programs/prefill8k.hex
```

See [Architecture](docs/architecture.md) for the datapath and memory organization, and [Programming](docs/programming.md) for the 15 opcodes, instruction fields, tensor layouts and host interface.

## 🧩 Build from Chisel

Install the Chipyard build environment following [Environment](docs/environment.md#chisel-and-chipyard), then generate the system RTL and compile it:

```bash
source scripts/env.sh
bash scripts/elaborate_chipyard.sh
bash scripts/build_sim.sh .local/chipyard/DeltaFlowHarness.sv .local/sim_chisel
```

Set `CHIPYARD_DIR` before sourcing `scripts/env.sh` to use an existing installation. The host sends instructions through Chipyard's register bus, while sixteen AXI4 ports carry DMA traffic to and from external memory. See [Integration](docs/integration.md) for the configuration and port connections.

## ⏱️ Synthesis and timing analysis

Use Yosys with Slang for synthesis and OpenSTA for static timing analysis. Follow [Synthesis and timing](docs/synthesis.md) to download the ASAP7 LVT/TT standard-cell libraries and run the tools.

SRAM read/write simulation uses the models in [rtl/memory_models](rtl/memory_models). During synthesis, the `_bb.sv` files in that directory keep each SRAM instance as a memory block. [scripts/generate_sram.py](scripts/generate_sram.py) generates the SRAM Liberty file used for area and timing estimation.

Run STA on a newly synthesized netlist or the mapped netlist in [reference](reference). Both commands are described in the synthesis guide.

## 📊 GPU and TPU profiles

We also provide GPU and TPU profiling traces. Open the traces in [profiles/gpu](profiles/gpu) and [profiles/tpu](profiles/tpu) with the [Perfetto web viewer](https://ui.perfetto.dev/). [Profiles](profiles/README.md) explains how to open JSON files and decompress the TPU traces.

## 📚 Documentation

- [Environment](docs/environment.md)
- [Simulation](docs/simulation.md)
- [Architecture](docs/architecture.md)
- [Programming](docs/programming.md)
- [Chipyard integration](docs/integration.md)
- [Synthesis and timing](docs/synthesis.md)

Build scripts write generated files and logs to `.local/`. Setup uses `.deps/` for downloaded dependencies and `.venv/` for the Python environment.

## 📄 License

Copyright 2026 DeltaFlow contributors.

DeltaFlow's original RTL, software and documentation use [Apache-2.0](LICENSE). Third-party components keep their own licenses; read the license texts in [licenses](licenses).

## 🤝 Collaboration

We are looking for opportunities to collaborate on efficient large language models and algorithm–hardware co-design. If you're interested, feel free to contact us at [gushaowei2024@ia.ac.cn](mailto:gushaowei2024@ia.ac.cn).

## Acknowledgements

DeltaFlow builds on the following open-source projects:

- [Chipyard](https://github.com/ucb-bar/chipyard) for system integration and [Chisel](https://github.com/chipsalliance/chisel) for RTL development.
- [EasyFloat](https://github.com/VCA-EPFL/easyfloat) for floating-point arithmetic support.
- [FSA](https://github.com/VCA-EPFL/FSA) for reference accelerator code and the [Chipyard build environment](https://github.com/VCA-EPFL/chipyard-fsa) used in our build flow.
