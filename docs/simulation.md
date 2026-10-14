# Simulation

## Build the full system

From the release root, after [environment setup](environment.md):

```bash
source scripts/env.sh
BUILD_JOBS=32 SIM_THREADS=8 bash scripts/build_sim.sh
```

This compiles the complete Chipyard SystemVerilog, including all four 64 × 64 tiles, DMA, MMIO and the outer harness. Subarrays are compiled separately to reduce build cost. The script replaces the inline SRAM module with project-generated functional 1R1W models. Assertions are enabled. No process library is needed.

## Generate and run workloads

```bash
python examples/generate.py gemm --m 256 --n 256 --k 2048 --out .local/examples/gemm
python scripts/run_sim.py .local/examples/gemm

python examples/generate.py prefill --tokens 1024 --out .local/examples/prefill
python scripts/run_sim.py .local/examples/prefill

python examples/generate.py decode --decode-tokens 4 --out .local/examples/decode
python scripts/run_sim.py .local/examples/decode

python examples/generate.py replay --accepted 2 --out .local/examples/replay
python scripts/run_sim.py .local/examples/replay
```

Use a new output directory when changing a completed example. GEMM is four independent matrices with one broadcast shape/program. Prefill is B = 1, H = 4, Dk = Dv = 128 and C = 64. Two raw synthetic blocks alternate and each head has an independent resident state initialized to the provided nonzero H0. The supported generator length is any multiple of 64 up to 65,536; long runs take more host time and generate larger HBM images.

Decode processes actual single tokens using STEP, without padding every token to a C64 chunk. Replay first computes and exports history, restores an actual HBM checkpoint, reads corrected E from actual previous output writes, rebuilds the accepted state and executes the next decode token. `--accepted` selects 0, 2 or 4 draft tokens. It never substitutes host-precomputed E for the cache.

The ordered FP32/BF16 numerical oracle is used only to generate expected HBM writes. It recomputes prefill U/W from the provided raw inputs. RTL performs all accelerator calculations. GEMM uses exact small integer inputs; recurrent examples use separate seeded inputs on each tile. Functional checks retain state, numerical, protocol and output-mask comparisons.

## Fixture and result formats

Each directory contains `program_16.hex`, `memory_16.txt`, `sections_16.txt` and `workload_16.json`. The HBM image format is:

```text
M lane offset_hex data_256bit_hex
E lane offset_hex expected_256bit_hex mask_256bit_hex
```

M records initialize external memory. E records describe masked expected writes and do not initialize that memory. The driver observes actual AXI W handshakes, compares every required write and waits for the final fence, all completions and idle before reporting PASS. It produces `result_16.json` and instruction, DMA, chunk and section-event CSVs. RTL assertions and protocol errors terminate the run. The driver has a bounded 20-million-cycle timeout.

For the full array, measured main utilization is `panel_waves / (64 * cycles)`. Each panel wave represents 256 main MACs. The denominator includes MMIO, HBM traffic, preparation, draining and final H. Auxiliary FP32 work is not included in the main-array numerator. GEMM tails can perform padded work; for useful GEMM utilization use `4 * M * N * K / (16384 * cycles)` as well. Native decode/replay use auxiliary lanes, so a low or zero main-array utilization is not a failure of those modes.

## AXI/HBM service model

The testbench models AR/R/AW/W/B handshakes, burst bounds, response delays and backpressure. Each lane has at most eight inflight read bursts; DMA bursts are at most sixteen 32-byte beats and respect 4-KiB boundaries. Read service starts after `12 + lane % 4` cycles; B follows the last W by `19 + lane % 4` cycles. Periodic stalls affect AR, AW, W and R. Payload service is limited to one beat per lane per cycle, with read/write sharing the lane budget.

It is an AXI service model, not bank-aware HBM, refresh, PHY or board timing. Do not treat its cycles as a measured physical HBM platform result.

## Generate RTL from Chisel

Follow [Integration](integration.md) to generate the complete Chipyard hierarchy and pass the resulting SV to `scripts/build_sim.sh`. [Synthesis and timing](synthesis.md) also shows how to generate a small PE for area and timing analysis.

For a detached run, redirect files and standard input explicitly:

```bash
nohup setsid python scripts/run_sim.py .local/examples/prefill > .local/prefill.log 2>&1 < /dev/null &
```

This survives an SSH disconnect; a container or machine restart still stops it.
