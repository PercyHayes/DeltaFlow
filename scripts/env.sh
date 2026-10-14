#!/usr/bin/env bash
# Source this file from any working directory. All build products stay local.
export DELTAFLOW_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export CHIPYARD_DIR="${CHIPYARD_DIR:-$DELTAFLOW_REPO_ROOT/.deps/chipyard-fsa}"
if [[ -f "$CHIPYARD_DIR/env.sh" && "${DELTAFLOW_SKIP_CHIPYARD_ENV:-0}" != 1 ]]; then
  source "$CHIPYARD_DIR/env.sh"
fi
export FSA_CHIPYARD_DIR="$CHIPYARD_DIR"
export DELTAFLOW_RTL_BUILD_DIR="$DELTAFLOW_REPO_ROOT/.local/sbt"
export DELTAFLOW_COMPACT_PE=1
export PYTHONPATH="$DELTAFLOW_REPO_ROOT/software${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
if [[ -f "$DELTAFLOW_REPO_ROOT/.venv/bin/activate" ]]; then
  source "$DELTAFLOW_REPO_ROOT/.venv/bin/activate"
fi
mkdir -p "$DELTAFLOW_REPO_ROOT/.local/tmp" "$DELTAFLOW_REPO_ROOT/rtl/.local/tmp"
export TMPDIR="$DELTAFLOW_REPO_ROOT/.local/tmp"
# sbt 1.8.2's Unix boot socket cannot use a long absolute workspace path.
# A short relative JVM temp directory keeps actual files inside the build cwd.
export JAVA_TOOL_OPTIONS="${JAVA_TOOL_OPTIONS:-} -Djava.io.tmpdir=.local/tmp -Dsbt.server.autostart=false"
