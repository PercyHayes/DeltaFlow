#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
out="${1:-$DELTAFLOW_REPO_ROOT/.local/chipyard}"
mkdir -p "$out"
out="$(cd -- "$out" && pwd)"
python "$DELTAFLOW_REPO_ROOT/scripts/setup_chipyard.py"
cd "$CHIPYARD_DIR"
mkdir -p .local/tmp
generator_args=(
  "--target-dir $out"
  "--name deltaflow"
  "--top-module deltaflow.integration.DeltaFlowHarness"
  "--legacy-configs deltaflow.integration:DeltaFlow16Config"
)
sbt -batch "chipyard/runMain chipyard.Generator ${generator_args[*]}"
firtool "$out/deltaflow.fir" --format=fir --verilog \
  --disable-all-randomization --annotation-file="$out/deltaflow.anno.json" \
  --disable-annotation-unknown --disable-annotation-classless \
  --blackbox-path="$out" --strip-debug-info -o "$out/DeltaFlowHarness.sv"
python "$DELTAFLOW_REPO_ROOT/scripts/portable_sv.py" "$out/DeltaFlowHarness.sv"
