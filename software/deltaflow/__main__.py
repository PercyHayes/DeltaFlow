"""Emit executable programs for the same unified four-tile ISA."""

import argparse
import json
from pathlib import Path
from .gemm_stream import compile_gemm_stream
from .system import Program, emit_gdn_prefill


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    gemm = sub.add_parser("gemm")
    for dim in ("m", "n", "k"):
        gemm.add_argument("--" + dim, type=int, required=True)
    gemm.add_argument("--output", type=Path, required=True)
    prefill = sub.add_parser("prefill")
    prefill.add_argument("--chunks", type=int, required=True)
    prefill.add_argument("--output", type=Path, required=True)
    a = parser.parse_args()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    if a.mode == "gemm":
        plan = compile_gemm_stream(a.m, a.n, a.k, schedule="overlap")
        a.output.write_text("\n".join(plan["words"]) + "\n")
        a.output.with_suffix(".json").write_text(json.dumps(plan, indent=2) + "\n")
        print(len(plan["words"]), "instructions; final fence tag", plan["final_fence_tag"])
    else:
        p = Program()
        tag = emit_gdn_prefill(p, a.chunks, output_address=0x08000000)
        a.output.write_text(p.hex())
        print(len(p.words), "instructions; final fence tag", tag)


if __name__ == "__main__":
    main()
