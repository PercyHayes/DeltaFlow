# Architecture

## Complete system

The Chipyard harness exposes a host AXI register interface and sixteen direct AXI4 payload masters. The host submits a single instruction stream. A system router dispatches masked commands to four identical compute tiles and their DMA engines, with four AXI4 payload ports assigned to each tile. Four tiles can process four independent GDN heads concurrently. They share instruction dimensions and local addresses while retaining separate HBM windows, input values and state. GEMM can similarly run independent matrices or software-partitioned matrix regions on the four tiles.

<p align="center"><img src="assets/architecture.svg" alt="DeltaFlow system and compute-tile architecture" width="960"></p>

The figure expands Tile 1; all four tiles use the same compute and memory organization. The DMA engines are system-level components, with four channels serving each tile.

The sixteen payload ports have 256-bit data. Each tile owns four ports and a nominal 128-byte/cycle aggregate payload quota; the chip total is 512 bytes/cycle. Read and write traffic share each modeled lane's service quota. These are logical AXI ports and bandwidth budgets, not a physical HBM PHY.

## Tile and PE

A tile contains sixteen independently controlled 16 × 16 systolic subarrays forming a 64 × 64 main array, plus an auxiliary unit with 65 FP32 arithmetic lanes. The auxiliary unit has 64 shared FMA lanes for gate processing, local triangular computation and state updates, and one FP32 prefix lane. Four local inverse controllers maintain the diagonal-block computation and share the FMA lanes; they are not additional groups of arithmetic lanes. The four tiles provide 16,384 main MACs/cycle. Subarrays support output-stationary (OS) and weight-stationary (WS) execution, independent completion/ownership and bounded operand/result handoff.

Each PE contains one BF16 coefficient register, **COEF**, four FP32 accumulation registers, **ACC0–ACC3**, and a five-stage FMA pipeline. The FMA performs BF16 multiplication with FP32 accumulation; the FP32 addend enters at the alignment stage.

In OS mode, north and west operands propagate south and east, while a selected ACC register retains the partial sum. Successive updates to the same ACC register require four enabled cycles. Interleaving across ACC0–ACC3 lets the pipeline accept one operation per enabled cycle while preserving each output's accumulation order.

<p align="center"><img src="assets/systolic_os_animated.svg" alt="OS operand propagation and four interleaved accumulator contexts" width="960"></p>

The animation shows a 4 × 4 crop of one 16 × 16 subarray and the shared FMA inside a PE. Colored ACC registers identify independent accumulation contexts. Playback illustrates operand propagation and context interleaving; it does not measure execution cycles or resource occupancy. A [GIF version](assets/systolic_os.gif) is also available.

In WS mode, COEF holds the stationary coefficient. The FMA multiplies it by the north input, adds the west partial sum and forwards the result east, bypassing ACC0–ACC3. Arithmetic metadata and operand/sum transfers follow the corresponding pipeline, including its advance/backpressure controls.

GDN uses local FP32 triangular processing, WS transform application and OS state/output updates in one array. Completed subarrays drive dependent updates; future preparation can overlap released portions of the current suffix. The hardware preserves subarray ownership, state recursion, rounding boundaries and backpressure. The next chunk can begin its state-dependent computations after both its preparation and the previous state update finish.

<p align="center"><img src="assets/gdn_execution_animated.svg" alt="GDN chunk preparation, shared state projection and output/state updates" width="960"></p>

The animation follows successive steady-state chunk intervals. The computation boxes map onto shared array and FP32 resources rather than separate hardware units. Current-chunk projections reuse the buffered incoming state; the output projection remains in ACC for the subsequent output update. Next-chunk preparation advances when inputs and resources are available, while its state-dependent projection waits for the completed recurrent state. A [GIF version](assets/gdn_execution.gif) is also available.

## Fixed SRAM budget

| Space, per tile | Banks | Words per bank | Bits per word | Capacity |
| --- | ---: | ---: | ---: | ---: |
| Operand | 16 | 1,024 | 128 | 256 KiB |
| Work | 16 | 512 | 128 | 128 KiB |
| Gate | 16 | 256 | 32 | 16 KiB |
| Total | 48 | | | 400 KiB |

Every bank is independent-address 1R1W, with a synchronous one-cycle read and byte-masked write. Read data holds when read enable is low. On a same-edge same-address read/write, the functional model reads the previous contents. The complete chip contains 192 macros, with 64 instances of each geometry.

Operand stores packed GEMM A/B buffers or GDN Q/K/V and preparation staging. Work stores GEMM output/seed pages or GDN intermediates and the resident FP32 state. Gate stores scalar decay/interval coefficients consumed by correlation and transform preparation. The paper's architecture diagram labels the 16-KiB gate region as Aux SRAM. Separate banks allow useful concurrent accesses; competing accesses to the same bank still arbitrate.

For Dk = Dv = 128, H occupies 64 KiB in Work rows 256–511. It remains on chip across chunks. Only the initial H is loaded and the final H is exported for the next layer/session. O is BF16 and exported per chunk. The included SRAM models are generated project RTL; their optional area/timing views are budgets, not characterized or manufacturable SRAM.

## Operation support

GEMM uses OS accumulation over bounded K segments and supports M/N tails. The compiler maps 128 × 128 output patches per tile onto the four ACC contexts of the physical 64 × 64 array. Larger matrices use multiple patches and K segments. Inputs are BF16; GEMM output and intermediate accumulation are FP32.

GDN prefill uses C = 64, with Q/K/V in BF16, activated scalar decay and beta in FP32, internal state in FP32 and O in BF16. Preparation and the current state/output phase overlap where ownership permits. Native decode uses the existing FP32 lanes for short recurrent updates. REBUILD consumes cached corrected E plus K and decay to reconstruct H from an HBM checkpoint, avoiding recomputing the state projection for those tokens.

The implemented gate is scalar per token. Native KDA requires channel-level gate handling and is not claimed by the current chunk engine. Projection layers that generate Q/K/V/g are separate GEMMs, not an implemented full LLM application in these examples.
