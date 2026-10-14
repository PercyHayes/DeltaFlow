# Simulation

## Build the simulator

Set up the tools following [Environment](environment.md), then run from the project root:

```bash
source scripts/env.sh
BUILD_JOBS=32 SIM_THREADS=8 bash scripts/build_sim.sh
```

`scripts/build_sim.sh` compiles `generated/DeltaFlowHarness.sv` and the C++ testbench into `.local/sim/chipyard_hbm`. The generated system contains four 64 × 64 tiles, DMA engines, MMIO and the Chipyard harness. The script compiles subarrays separately to reduce build cost and uses the 1R1W read/write models from `rtl/memory_models/` for SRAM simulation. Hardware assertions are enabled.

## Run the examples

Each example starts with `examples/generate.py`, which writes its instruction stream, initial HBM contents and expected outputs. `scripts/run_sim.py` runs that example with the simulator.

**GEMM:** four independent matrices use the same shape and broadcast instruction stream.

```bash
python examples/generate.py gemm --m 256 --n 256 --k 2048 --out .local/examples/gemm
python scripts/run_sim.py .local/examples/gemm
```

**GDN prefill:** B = 1, H = 4, Dk = Dv = 128 and chunk size 64.

```bash
python examples/generate.py prefill --tokens 1024 --out .local/examples/prefill
python scripts/run_sim.py .local/examples/prefill
```

The input generator alternates two synthetic chunks. Each head starts from a nonzero H0 and keeps its state in SRAM across the sequence. Set `--tokens` to a multiple of 64, from 64 through 65,536. Longer sequences produce larger HBM images and take more host time to simulate.

**Decode:** GDN_STEP processes four successive tokens per head.

```bash
python examples/generate.py decode --decode-tokens 4 --out .local/examples/decode
python scripts/run_sim.py .local/examples/decode
```

**Replay:** reconstruct the state for accepted draft tokens and execute the next decode token.

```bash
python examples/generate.py replay --accepted 2 --out .local/examples/replay
python scripts/run_sim.py .local/examples/replay
```

Replay first runs history and draft tokens, saving a checkpoint and corrected E through HBM writes. It then reloads the checkpoint, reads the cached K, decay and E, rebuilds H for the accepted tokens, and runs the next decode token. `--accepted` selects 0, 2 or 4 draft tokens.

The CPU reference in `examples/arithmetic.py` calculates expected results in the accelerator's FP32/BF16 operation order. The testbench compares the RTL's output writes against those results. GEMM uses small integer inputs; the recurrent examples generate separate seeded inputs for each tile. Choose a separate output directory for each workload configuration.

## Input and output files

The generator writes four files:

| File | Purpose |
| --- | --- |
| `program_16.hex` | 128-bit instruction stream |
| `memory_16.txt` | Initial HBM contents and expected output writes |
| `sections_16.txt` | Workload section boundaries for reporting |
| `workload_16.json` | Workload dimensions and options |

The HBM data file uses this format:

```text
M lane offset_hex data_256bit_hex
E lane offset_hex expected_256bit_hex mask_256bit_hex
```

An M record initializes one HBM beat. An E record specifies an expected output value and the bits to compare. The driver checks AXI write handshakes against these expected outputs. It reports PASS after the final fence, all completions and the idle state, and saves `result_16.json` plus instruction, DMA, chunk and section-event CSV files. Numerical mismatches, protocol errors and hardware assertion failures stop the simulation. The driver allows up to 20 million cycles per run.

## Utilization counters

Main-array utilization is `panel_waves / (64 * cycles)`. The system has 64 subarrays, and each panel wave represents 256 main MACs. The cycle count covers the complete program: instruction submission, HBM traffic, preparation, computation and final output/state transfers.

For GEMM, `4 * M * N * K / (16384 * cycles)` reports useful utilization for four independent matrices of the same shape. This formula counts useful matrix operations, while the panel-wave counter also counts padding at matrix edges. Decode and replay execute on the auxiliary FP32 lanes; use their cycle counts to measure latency. The panel-wave metric measures activity in the main systolic array.

## AXI/HBM service model

The C++ testbench services the AR/R/AW/W/B channels with burst limits, response delays and periodic backpressure:

- Each lane permits up to eight outstanding read bursts.
- DMA bursts carry up to sixteen 32-byte beats and split at 4-KiB boundaries.
- Read service begins after `12 + lane % 4` cycles.
- The B response follows the last write beat by `19 + lane % 4` cycles.
- Periodic stalls apply to AR, AW, W and R.
- Each lane transfers at most one payload beat per cycle, shared between reads and writes.

These service rules determine the external-memory latency and bandwidth seen by the simulated accelerator.

## Generate RTL from Chisel

Follow [Integration](integration.md) to generate `.local/chipyard/DeltaFlowHarness.sv`, then pass that file to `scripts/build_sim.sh`. [Synthesis and timing](synthesis.md#pe-synthesis-and-sta) shows how to generate a PE for a smaller synthesis run.

## Run in the background

To keep a simulation running after an SSH disconnect, redirect its input and output and start it in a separate session:

```bash
nohup setsid python scripts/run_sim.py .local/examples/prefill > .local/prefill.log 2>&1 < /dev/null &
```
