"""Address-distinct, K-varying integer GEMM fixtures for the real Chipyard host.
All matrices and numerical expectations live in the external AXI image only.
"""

from pathlib import Path
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))
from deltaflow.gemm_stream import compile_gemm_stream
from deltaflow.system import gemm_operand_rows, pack_rows


def generate(folder, m, n, k, schedule, **addresses):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    if (folder / "result_16.json").exists():
        raise ValueError("refuse to overwrite a completed fixture")
    plan = compile_gemm_stream(m, n, k, schedule=schedule, **addresses)
    with (folder / "memory_16.txt").open("w") as f:
        for tile in range(4):
            a = ((np.arange(m)[:, None] * 3 + np.arange(k)[None, :] * 5 + tile) % 7 - 3).astype(
                np.int64
            )
            b = ((np.arange(k)[:, None] * 7 + np.arange(n)[None, :] * 3 + tile * 2) % 5 - 2).astype(
                np.int64
            )
            c = a @ b
            abits = (a.astype(np.float32).view(np.uint32) >> 16).tolist()
            bbits = (b.astype(np.float32).view(np.uint32) >> 16).tolist()
            for desc in plan["images"]:
                rows = gemm_operand_rows(
                    abits, bbits, desc["row"], desc["column"], desc["k_start"], desc["k_count"]
                )
                groups = (0, 1) if desc["kind"] == "ab" else (1,)
                for port, words in enumerate(pack_rows(rows, groups)):
                    for i, word in enumerate(words):
                        f.write(f'M {4*tile+port} {desc["address"]+32*i:x} {word:064x}\n')
            cbits = c.astype(np.float32).view(np.uint32)
            for patch in plan["patches"]:
                rows = []
                masks = []
                for r in range(128):
                    for h in range(2):
                        row = [0] * 16
                        mask = [0] * 16
                        for col in range(64):
                            if r < patch["m_valid"] and 64 * h + col < patch["n_valid"]:
                                row[col // 4] |= int(
                                    cbits[patch["row"] + r, patch["column"] + 64 * h + col]
                                ) << (32 * (col % 4))
                                mask[col // 4] |= 0xFFFFFFFF << (32 * (col % 4))
                        rows.append(row)
                        masks.append(mask)
                for port, (words, valid) in enumerate(zip(pack_rows(rows), pack_rows(masks))):
                    for i, (word, mask) in enumerate(zip(words, valid)):
                        if mask:
                            f.write(
                                f'E {4*tile+port} {patch["output_address"]+32*i:x} {word:064x} {mask:064x}\n'
                            )
    (folder / "program_16.hex").write_text("\n".join(plan["words"]) + "\n")
    (folder / "sections_16.txt").write_text(f'gemm_stream {plan["final_fence_tag"]}\n')
    (folder / "workload_16.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(folder, len(plan["words"]), "instructions", len(plan["patches"]), "patches", flush=True)
    return plan
