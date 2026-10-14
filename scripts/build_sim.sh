#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source_sv="${1:-$DELTAFLOW_REPO_ROOT/generated/DeltaFlowHarness.sv}"
build_dir="${2:-$DELTAFLOW_REPO_ROOT/.local/sim}"
mkdir -p "$build_dir/tmp"
build_dir="$(cd -- "$build_dir" && pwd)"
python "$DELTAFLOW_REPO_ROOT/scripts/prepare_rtl.py" --source "$source_sv" --out "$build_dir/rtl"
mapfile -t sources < "$build_dir/rtl/sources.f"
export TMPDIR="$build_dir/tmp"
# Some conda distributions report a root without bin/. Hierarchical child
# Verilator invocations require that executable layout.
verilator_root="$(verilator -V | tr -d '\000' | sed -n 's/^ *VERILATOR_ROOT *= *//p' | head -1)"
if [[ ! -x "$verilator_root/bin/verilator" ]]; then
  local_root="$DELTAFLOW_REPO_ROOT/.local/verilator_root"
  mkdir -p "$local_root/bin"
  ln -sfn "$verilator_root/include" "$local_root/include"
  for tool in verilator verilator_bin; do
    ln -sfn "$(command -v "$tool")" "$local_root/bin/$tool"
  done
  ln -sfn "$verilator_root/bin/verilator_includer" "$local_root/bin/verilator_includer"
  export VERILATOR_ROOT="$local_root"
fi
export MAKEFLAGS="OPT_FAST=-O3 OPT_SLOW=-O0 VK_PCH_I_FAST= VK_PCH_I_SLOW="
verilator --cc --exe -j 1 --verilate-jobs 1 --assert \
  --hierarchical "$DELTAFLOW_REPO_ROOT/scripts/panels.vlt" \
  --threads "${SIM_THREADS:-8}" --threads-dpi all \
  --top-module DeltaFlowHarness -Wno-fatal -Wno-WIDTH -Wno-STMTDLY \
  -CFLAGS "-DVL_USER_STOP -DVL_USER_FATAL -DVL_USER_FINISH -I$build_dir" \
  --Mdir "$build_dir" -o chipyard_hbm "${sources[@]}" \
  "$DELTAFLOW_REPO_ROOT/rtl/chipyard/src/test/cpp/chipyard_hbm.cpp"
# Keep code generation serial; then compile independent C++ files in parallel.
make -C "$build_dir" -f VDeltaFlowHarness_hier.mk -j 1 hier_verilation
make -C "$build_dir" -f VDeltaFlowHarness_hier.mk -j "${BUILD_JOBS:-32}" hier_build
