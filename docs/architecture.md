# Architecture

## Complete system

The Chipyard harness exposes a host AXI register interface and sixteen direct AXI4 payload masters. The host submits a single instruction stream. A system router dispatches masked commands to four identical compute tiles and their DMA engines, with four AXI4 payload ports assigned to each tile. Four tiles can process four independent GDN heads concurrently. They share instruction dimensions and local addresses while retaining separate HBM windows, input values and state. GEMM can similarly run independent matrices or software-partitioned matrix regions on the four tiles.

<p align="center"><img src="assets/architecture.svg" alt="DeltaFlow system and compute-tile architecture" width="960"></p>

The figure expands Tile 1; all four tiles use the same compute and memory organization. The DMA engines are system-level components, with four channels serving each tile.

Each payload port transfers 256-bit data. Four ports give each tile a 128-byte/cycle aggregate payload budget, and the four tiles total 512 bytes/cycle. The simulation model shares each lane's transfer budget between reads and writes.

## Tile and PE

A tile contains sixteen independently controlled 16 × 16 subarrays forming a 64 × 64 systolic array, plus 65 auxiliary FP32 lanes. Of these, 64 shared FMA lanes process gates, local triangular blocks and state updates; one prefix lane computes cumulative gate products. Four local inverse controllers share the 64 FMA lanes to solve diagonal blocks. The four tiles provide 16,384 main MACs per cycle. Each subarray supports output-stationary (OS) and weight-stationary (WS) execution and tracks when its operands and results are ready for the next operation.

Each PE contains one BF16 coefficient register, **COEF**, four FP32 accumulation registers, **ACC0–ACC3**, and a five-stage FMA pipeline. The FMA performs BF16 multiplication with FP32 accumulation; the FP32 addend enters at the alignment stage.

In OS mode, north and west operands propagate south and east, while a selected ACC register retains the partial sum. Successive updates to the same ACC register require four enabled cycles. Interleaving across ACC0–ACC3 lets the pipeline accept one operation per enabled cycle while preserving each output's accumulation order.

<p align="center"><img src="assets/systolic_os_animated.svg" alt="OS operand propagation and four interleaved accumulator contexts" width="960"></p>

The animation shows a 4 × 4 crop of one 16 × 16 subarray. Operands move through the array while four colored ACC contexts interleave through each PE's shared FMA. [View the GIF](assets/systolic_os.gif).

In WS mode, COEF holds the stationary coefficient. The FMA multiplies it by the north input, adds the west partial sum and forwards the result east. ACC0–ACC3 hold OS partial sums. Tags, operands and partial sums advance with the pipeline and wait together when downstream backpressure stalls it.

GDN prepares local triangular blocks with the FP32 lanes, applies their transforms in WS mode and computes state/output updates in OS mode. When a subarray finishes its current work, the scheduler can use it for the next chunk's preparation. The next chunk starts its state-dependent computations once its preparation and the previous chunk's state update have both finished.

<p align="center"><img src="assets/gdn_execution_animated.svg" alt="GDN chunk preparation, shared state projection and output/state updates" width="960"></p>

The animation follows successive chunks. Its boxes represent operations on the shared array and FP32 lanes. The current chunk's projections reuse buffered incoming state, and the output projection stays in ACC for the following output update. Preparation for the next chunk uses subarrays as they become available; its state projection waits for the completed recurrent state. [View the GIF](assets/gdn_execution.gif).

## SRAM organization

| Space, per tile | Banks | Words per bank | Bits per word | Capacity |
| --- | ---: | ---: | ---: | ---: |
| Operand | 16 | 1,024 | 128 | 256 KiB |
| Work | 16 | 512 | 128 | 128 KiB |
| Gate | 16 | 256 | 32 | 16 KiB |
| Total | 48 | | | 400 KiB |

Every bank is independent-address 1R1W, with a synchronous one-cycle read and byte-masked write. Read data holds when read enable is low. On a same-edge same-address read/write, the functional model reads the previous contents. The complete chip contains 192 macros, with 64 instances of each geometry.

Operand SRAM holds packed GEMM A/B buffers, or GDN Q/K/V, preparation data and output staging. Work SRAM holds GEMM output/seed pages, or GDN intermediates and the resident FP32 state. Gate SRAM holds scalar decay and interval coefficients used in correlation and transform preparation; the paper labels this 16-KiB region as Aux SRAM. Accesses to different banks can proceed concurrently, while accesses to the same bank share its read and write ports.

For Dk = Dv = 128, H occupies 64 KiB in Work rows 256–511. DMA loads the initial H once, and H remains on chip across chunks. DMA writes the final H after the sequence and BF16 O after each chunk. [Synthesis and timing](synthesis.md#sram-models-and-area-estimates) describes the simulation models and SRAM area estimates.

## Operation support

GEMM uses OS accumulation over bounded K segments and supports M/N tails. The compiler maps 128 × 128 output patches per tile onto the four ACC contexts of the physical 64 × 64 array. Larger matrices use multiple patches and K segments. Inputs are BF16; GEMM output and intermediate accumulation are FP32.

GDN prefill uses chunk size 64, with BF16 Q/K/V, FP32 activated scalar decay and beta, FP32 state and BF16 O. Next-chunk preparation uses resources released by the current state/output phase. Decode runs short recurrent updates on the FP32 lanes. REBUILD reads cached corrected E, K and decay to reconstruct H from an HBM checkpoint.

The chunk engine takes one scalar decay gate per token. The examples load Q/K/V and activated gate values from HBM. Use GEMM programs for the matrix multiplications in the preceding projection layers.
