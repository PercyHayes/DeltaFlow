# Synthesis, area and timing

The flow uses uniform **ASAP7 LVT, TT, 0.7 V, 25 °C** standard cells. No standard cell Liberty, LEF, GDS or downloaded SRAM macro is distributed in this release. Only project-generated SRAM RTL and the local budget-view generator are included.

## Download the process library

Obtain the ASAP7 platform from [OpenROAD-flow-scripts](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts). Its [ASAP7 configuration](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/master/flow/platforms/asap7/config.mk) defines the threshold and NLDM corner groups. Download to a dependency directory:

```bash
mkdir -p .deps
git clone --depth 1 --filter=blob:none --sparse https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git .deps/orfs
git -C .deps/orfs sparse-checkout set flow/platforms/asap7
export ASAP7_LIB_DIR="$PWD/.deps/orfs/flow/platforms/asap7/lib/NLDM"
```

Use these **five LVT/TT files**, with the exact dated library names below:

```text
asap7sc7p5t_SEQ_LVT_TT_nldm_220123.lib
asap7sc7p5t_AO_LVT_TT_nldm_211120.lib.gz
asap7sc7p5t_INVBUF_LVT_TT_nldm_220122.lib.gz
asap7sc7p5t_OA_LVT_TT_nldm_211120.lib.gz
asap7sc7p5t_SIMPLE_LVT_TT_nldm_211120.lib.gz
```

`prepare_lvt.py` merges these external groups into the run directory. It retains the original tables and units. The standard-cell library uses ps and fF; OpenSTA command units are explicitly ns and pF. The mapper's load is 1 fF; the STA output load is 0.001 pF. Do not silently substitute RVT, SLVT, FF or SS.

P&R is not implemented here. A future LVT P&R flow also needs the matching 1x technology/cell LEF, GDS and RC views from that upstream platform, and a real SRAM implementation. Those downloads belong in the local dependency directory.

## Generated SRAM budget views

```bash
python scripts/generate_sram.py --out .local/sram
```

The generator emits three exact 1R1W functional/blackbox interfaces and an illustrative Liberty file. Functional RTL matches byte masks, independent read and write addresses, one-cycle synchronous reads and held read output. Synthesis uses blackboxes; it must not synthesize the behavioral memory array into FFs.

| Macro | Instances in complete chip | Budget area per instance, µm² |
| --- | ---: | ---: |
| 1,024 × 128, 1R1W | 64 | 8,310.904 |
| 512 × 128, 1R1W | 64 | 4,212.376 |
| 256 × 32, 1R1W | 64 | 540.778 |

Total SRAM budget area is 836,099.712 µm². These retained estimates came from FakeRAM geometry with an assumed 1.5 multiplier for the port organization. The generator fixes the resulting areas so reproducing this budget does not require FakeRAM. Its timing assumptions are 0.2183 ns access, 0.05 ns setup, 0.05 ns hold and 0.005 pF input capacitance. They are not transistor characterization, SRAM power measurements or a manufacturable macro.

## Fresh synthesis

Use explicit tool paths from [Environment](environment.md). Keep work paths free of spaces and command delimiters because the Slang frontend consumes its file arguments directly.

```bash
python scripts/synthesize.py \
  --yosys .deps/oss-cad-suite/bin/yosys \
  --lib-dir "$ASAP7_LIB_DIR" \
  --source generated/DeltaFlowHarness.sv \
  --out .local/synthesis
```

The flow selects SRAM blackboxes, elaborates SV with Slang, shares identical RTLIL module definitions by direct body equality, and keeps every physical instance. It flattens arithmetic within PE/FP32 modules while preserving panel/tile hierarchy. It removes simulation assertions for synthesis, maps flip-flops and uses ABC with an 800-ps mapping target. It excludes the same `*x1p*`, `*xp*`, SDF and ICG cells as the retained screening setup.

Outputs include frontend and mapping logs, `stat.json` and `mapped.v`. The complete linked design area includes the illustrative SRAM area. ABC's module delay is a screening statistic, not full-chip STA. Fresh mapping may differ from the selected reference; it does not reproduce the subsequent load repair and local cell-sizing campaign automatically.

## Complete linked STA

Run a fresh mapped netlist:

```bash
python scripts/run_sta.py \
  --sta .local/opensta-build/sta \
  --lib-dir "$ASAP7_LIB_DIR" \
  --netlist .local/synthesis/mapped.v \
  --out .local/sta_fresh --threads 4
```

Or analyze the actual selected complete mapped reference shipped as three compressed parts, each below GitHub's single-file size limit:

```bash
python scripts/run_sta.py \
  --sta .local/opensta-build/sta \
  --lib-dir "$ASAP7_LIB_DIR" \
  --out .local/sta_reference --threads 4
```

The script concatenates/decompresses the parts in the run directory, reads the complete four-tile hierarchy and the generated SRAM budget views, then links all modules. It never substitutes one tile's timing for the full system.

Constraints are a 1.0-ns ideal clock, 0.1-ns setup uncertainty, 0.025-ns clock transition, 0.1-ns input/output delay, 0.05-ns input transition and 0.001-pF output load. There are no false paths, multicycle paths or mode exemptions. Reports include setup coverage, WNS, TNS, full paths and electrical violations.

The retained selected-reference result is WNS +0.001853 ns, TNS 0, logic area 10.279465544 mm² and area with SRAM budget 11.115565256 mm². Its worst reported logic after Q is 0.845190 ns, so the separate 800-ps logic design budget was not reached. This is ideal-clock, pre-layout setup analysis, not physical 1-GHz signoff. Wire parasitics, CTS, P&R and characterized SRAM are absent.

Full reference STA previously took about 46 minutes with four threads and about 185 GiB peak RAM. Allow real available memory to determine whether a run can continue; do not terminate simply at an arbitrary RSS threshold. Reduce threads if genuinely necessary. Keep logs and run directories for diagnosis.

## Small arithmetic probe

Before a full run, elaborate the selected PE and exercise the same mapping and STA scripts on it:

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

A positive PE WNS does not prove array/controller or complete-system closure.
