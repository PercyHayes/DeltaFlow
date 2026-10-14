"""One ISA and HBM fixture format for GEMM, prefill, decode and replay."""

from pathlib import Path
import argparse
import os

ROOT = Path(__file__).resolve().parents[1]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["gemm", "prefill", "decode", "replay"])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--m", type=int, default=256)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--k", type=int, default=2048)
    parser.add_argument("--tokens", type=int, default=256)
    parser.add_argument("--decode-tokens", type=int, default=4)
    parser.add_argument("--accepted", type=int, choices=[0, 2, 4], default=2)
    args = parser.parse_args()
    if args.mode == "gemm":
        from gemm import generate

        generate(args.out, args.m, args.n, args.k, "overlap")
    elif args.mode == "prefill":
        from prefill import generate

        generate(args.out, args.tokens)
    else:
        os.environ["DELTAFLOW_EXAMPLE_OUTPUT"] = str(args.out.resolve())
        from recurrent import decode, replay

        if args.mode == "decode":
            decode(tokens=args.decode_tokens)
        else:
            replay(args.accepted)
