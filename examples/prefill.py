"""Compile chunk prefill and compute ordered expectations from raw HBM inputs."""

from pathlib import Path
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))
from deltaflow.system import Program, emit_gdn_prefill, pack_rows, gdn_gate_rows
from reference import prepare, factors, suffix
from common import state_rows, output_rows


def generate(folder, tokens=256):
    if tokens < 64 or tokens % 64 or tokens > 65536:
        raise ValueError("prefill length must be a multiple of 64, from 64 to 65536")
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    if (folder / "result_16.json").exists():
        raise ValueError("use a new directory for a completed fixture")
    source = json.loads((ROOT / "examples/data/gdn_inputs.json").read_text())
    raw = {name: np.asarray(source[name], dtype=np.uint16) for name in ("q", "k", "v")}
    data = {name: (values.astype(np.uint32) << 16).view(np.float32) for name, values in raw.items()}
    g = np.asarray(source["decay_bits"], dtype=np.uint32).view(np.float32)
    beta = np.asarray(source["beta_bits"], dtype=np.uint32).view(np.float32)
    h = np.asarray(source["h_initial_bits"], dtype=np.uint32).view(np.float32).copy()
    templates = []
    for block in range(2):
        q, k, v = (data[name][block] for name in ("q", "k", "v"))
        u, w = prepare(k, v, g[block], beta[block], 16)
        templates.append((u, w, factors(q, k, g[block])))
    program = Program()
    emit_gdn_prefill(program, tokens // 64, output_address=0x08000000)
    (folder / "program_16.hex").write_text(program.hex())
    (folder / "sections_16.txt").write_text(f"final_h_exported {program._tag}\n")

    def image(stream, base, rows, groups=(0, 1), expected=False):
        for tile in range(4):
            for port, words in enumerate(pack_rows(rows, groups)):
                for index, word in enumerate(words):
                    tail = " " + "f" * 64 if expected else ""
                    stream.write(
                        f"{'E' if expected else 'M'} {4*tile+port} "
                        f"{base+32*index:x} {word:064x}{tail}\n"
                    )

    def column_rows(low, high=None):
        rows = [[0] * 16 for _ in range(128)]
        for token in range(64):
            bank, lane = divmod(token, 8)
            for column in range(128):
                rows[column][bank] |= int(low[token, column]) << (16 * lane)
                if high is not None:
                    rows[column][bank + 8] |= int(high[token, column]) << (16 * lane)
        return rows

    with (folder / "memory_16.txt").open("w") as stream:
        image(stream, 0x18000000, state_rows(h)[0])
        for chunk in range(tokens // 64):
            block = chunk % 2
            base = 0x100000 + chunk * 0x10000
            image(stream, base, column_rows(raw["k"][block], raw["q"][block]))
            image(stream, base + 0x4000, column_rows(raw["v"][block]), (0,))
            gates = zip(g[block].view(np.uint32).tolist(), beta[block].view(np.uint32).tolist())
            image(stream, base + 0xC000, gdn_gate_rows(list(gates)), (1,))
            u, w, f = templates[block]
            h, o, _ = suffix(u, w, h, f, state_epilogue=True)
            assert np.isfinite(h).all() and np.isfinite(o).all()
            image(stream, 0x08000000 + chunk * 0x10000, output_rows(o)[0], expected=True)
        image(stream, 0xB0000, state_rows(h)[0], expected=True)
    metadata = dict(
        kind="gdn_chunk_prefill",
        tokens=tokens,
        heads=4,
        dk=128,
        dv=128,
        chunk=64,
        instructions=len(program.words),
        main_array_macs=(tokens // 64) * 19922944,
        input="two synthetic raw blocks alternate; identical values on four independent heads",
        gates="activated FP32 decay and beta; no tensor payload in instructions",
    )
    (folder / "workload_16.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(folder, len(program.words), "instructions")
    return metadata
