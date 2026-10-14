# Synthesis, area and timing

Use Yosys with Slang to synthesize the SystemVerilog, then run OpenSTA on the mapped netlist. The standard-cell library is **ASAP7 LVT, TT, 0.7 V, 25 °C** throughout the design.

## Download the standard-cell library

Download the ASAP7 platform from [OpenROAD-flow-scripts](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts) into `.deps/`:

```bash
mkdir -p .deps
git clone --depth 1 --filter=blob:none --sparse https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git .deps/orfs
git -C .deps/orfs sparse-checkout set flow/platforms/asap7
export ASAP7_LIB_DIR="$PWD/.deps/orfs/flow/platforms/asap7/lib/NLDM"
```

The scripts read these five **LVT/TT** files:

```text
asap7sc7p5t_SEQ_LVT_TT_nldm_220123.lib
asap7sc7p5t_AO_LVT_TT_nldm_211120.lib.gz
asap7sc7p5t_INVBUF_LVT_TT_nldm_220122.lib.gz
asap7sc7p5t_OA_LVT_TT_nldm_211120.lib.gz
asap7sc7p5t_SIMPLE_LVT_TT_nldm_211120.lib.gz
```

`scripts/prepare_lvt.py` merges them into one Liberty file in the run directory, preserving the cell tables and units. The library uses ps and fF; the STA scripts set their command units to ns and pF. Synthesis uses a 1-fF output load, and STA uses the equivalent 0.001-pF load. The platform's [ASAP7 configuration](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/master/flow/platforms/asap7/config.mk) describes its library groups.

## SRAM models and area estimates

Generate the SRAM files with:

```bash
python scripts/generate_sram.py --out .local/sram
```

For each of the three SRAM sizes, the script writes:

- `<name>.sv`: a read/write simulation model with independent read and write addresses, byte write masks and a one-cycle synchronous read. The read output holds its value when read enable is low.
- `<name>_bb.sv`: the memory's port declarations for synthesis. Yosys preserves each instance as an SRAM block.
- `deltaflow_sram.lib`: SRAM area, input capacitance, read delay and setup/hold parameters for synthesis and STA.

The matching `.sv` and `_bb.sv` files are in `rtl/memory_models/`. `scripts/build_sim.sh` uses the read/write models, and `scripts/synthesize.py` uses the blackbox declarations and generated Liberty file.

| SRAM size | Instances in the full system | Estimated area per instance, µm² |
| --- | ---: | ---: |
| 1,024 × 128, 1R1W | 64 | 8,310.904 |
| 512 × 128, 1R1W | 64 | 4,212.376 |
| 256 × 32, 1R1W | 64 | 540.778 |

The total estimated SRAM area is 836,099.712 µm². These values come from FakeRAM geometry estimates scaled by 1.5 for the port organization. The generator stores the resulting values directly. Its timing parameters are 0.2183 ns read access, 0.05 ns setup, 0.05 ns hold and 0.005 pF input capacitance.

## Synthesize the full system

Use the tool paths from [Environment](environment.md). Choose a project path without spaces, semicolons or quotation marks for the Slang command invocation.

```bash
python scripts/synthesize.py \
  --yosys .deps/oss-cad-suite/bin/yosys \
  --lib-dir "$ASAP7_LIB_DIR" \
  --source generated/DeltaFlowHarness.sv \
  --out .local/synthesis
```

The script loads the SRAM blackboxes, reads the SystemVerilog with Slang and synthesizes each distinct module definition while preserving all instances. It flattens arithmetic inside the PE and FP32 modules and keeps the panel/tile hierarchy. It removes simulation assertions, maps flip-flops and runs ABC with an 800-ps delay target. The mapping library excludes `*x1p*`, `*xp*`, SDF and ICG cells.

The output directory holds the synthesis logs, cell-area report `stat.json` and mapped netlist `mapped.v`. Its total area combines mapped standard cells with the SRAM estimates above. Use ABC's reported module delays to compare mappings, then run OpenSTA to analyze timing across the complete hierarchy.

`scripts/synthesize.py` produces the initial mapped netlist. The netlist in `reference/` also has fanout buffers and resized control cells from timing optimization.

## Full-system STA

To analyze the newly synthesized netlist, run:

```bash
python scripts/run_sta.py \
  --sta .local/opensta-build/sta \
  --lib-dir "$ASAP7_LIB_DIR" \
  --netlist .local/synthesis/mapped.v \
  --out .local/sta_fresh --threads 4
```

To analyze the optimized netlist in `reference/`, run:

```bash
python scripts/run_sta.py \
  --sta .local/opensta-build/sta \
  --lib-dir "$ASAP7_LIB_DIR" \
  --out .local/sta_reference --threads 4
```

The reference netlist is stored in three compressed parts. The script joins and decompresses them into the run directory, loads the standard-cell and SRAM Liberty files, and links the four-tile `DeltaFlowHarness` hierarchy.

The STA constraints are:

| Constraint | Value |
| --- | ---: |
| Ideal clock period | 1.0 ns |
| Setup uncertainty | 0.1 ns |
| Clock transition | 0.025 ns |
| Input/output delay | 0.1 ns |
| Input transition | 0.05 ns |
| Output load | 0.001 pF |

STA evaluates setup paths throughout the linked design. The run directory contains setup coverage, WNS, TNS, detailed paths and electrical-violation reports.

The reference netlist produced WNS +0.001853 ns, TNS 0, logic area 10.279465544 mm² and total area 11.115565256 mm² with the SRAM estimates above. Its longest reported combinational delay after Q is 0.845190 ns. This analysis uses the mapped netlist, an ideal clock and the SRAM timing parameters above. Placement and routing supply the wire and clock-tree delays for physical timing analysis.

The full reference run took about 46 minutes with four threads and peaked at about 185 GiB RAM. Use `--threads` to set the OpenSTA thread count and retain the run directory to inspect its reports.

## PE synthesis and STA

For a smaller run, generate the PE and use the same synthesis and STA scripts:

```bash
source scripts/env.sh
cd rtl
sbt -batch "runMain deltaflow.PublicElaborate pe $DELTAFLOW_REPO_ROOT/.local/pe"
cd ..
python scripts/synthesize.py --yosys .deps/oss-cad-suite/bin/yosys \
  --lib-dir "$ASAP7_LIB_DIR" --source .local/pe/RetimedWavePE.sv \
  --top RetimedWavePE --out .local/pe_synthesis
python scripts/run_sta.py --sta .local/opensta-build/sta \
  --lib-dir "$ASAP7_LIB_DIR" --netlist .local/pe_synthesis/mapped.v \
  --top RetimedWavePE --out .local/pe_sta
```

Use the PE flow to compare arithmetic implementations quickly. The full-system flow evaluates paths through the tile controllers, SRAM interfaces and Chipyard logic.
