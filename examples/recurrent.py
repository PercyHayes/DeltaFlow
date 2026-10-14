"""Native short recurrence and real-HBM corrected-E replay fixtures."""

from pathlib import Path
import sys
import json
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))
from deltaflow.system import Program, pack_rows
from reference import AR, bf16, recurrence, error
from common import make_tokens
from common import bits, state_rows, output_rows


def projection(x, h):
    partial = np.zeros((8, h.shape[1]), np.float32)
    for j in range(len(x)):
        partial[j % 8] = AR.fma(x[j], h[j], partial[j % 8])
    total = np.zeros(h.shape[1], np.float32)
    for j in range(8):
        total = AR.fma(np.float32(1), partial[j], total)
    return total


def step(data, h):
    q, k, v, g, beta = data
    h = h.copy()
    outputs = []
    errors = []
    for i in range(len(q)):
        e = AR.mul(beta[i], AR.fma(-g[i], projection(k[i], h), v[i]))
        h = AR.fma(k[i, :, None], e[None, :], AR.mul(g[i], h))
        outputs.append(bf16(projection(q[i], h)))
        errors.append(e)
    return h, np.asarray(outputs), np.asarray(errors)


def rebuild(data, corrections, h):
    _, k, _, g, _ = data
    h = h.copy()
    for i in range(len(k)):
        h = AR.fma(k[i, :, None], corrections[i, None, :], AR.mul(g[i], h))
    return h


def vector_row(x, fp32=False):
    values = bits(x) if fp32 else bits(x) >> 16
    lanes = 4 if fp32 else 8
    width = 32 if fp32 else 16
    row = [0] * 16
    mask = [0] * 16
    for i, x in enumerate(values):
        row[i // lanes] |= int(x) << (width * (i % lanes))
        mask[i // lanes] |= ((1 << width) - 1) << (width * (i % lanes))
    return row, mask


def raw_rows(data):
    q, k, v, g, b = data
    rows = []
    for i in range(len(q)):
        rows.extend(
            [
                vector_row(q[i])[0],
                vector_row(k[i])[0],
                vector_row(v[i])[0],
                vector_row(np.asarray([g[i], b[i]], np.float32), True)[0],
                [0] * 16,
                [0] * 16,
            ]
        )
    return rows


def result_rows(o, e):
    rows = []
    masks = []
    for oo, ee in zip(o, e):
        for x, wide in ((oo, False), (ee[:64], True), (ee[64:], True)):
            row, mask = vector_row(x, wide)
            rows.append(row)
            masks.append(mask)
    return rows, masks


class Fixture:
    def __init__(self, name, cases):
        self.name = name
        self.dk = cases[0][1].shape[0]
        self.dv = cases[0][1].shape[1]
        self.p = Program()
        self.memory = []
        self.expected = []
        self.sections = []
        self.ops = []
        self.index = 0
        self.h = [h.copy() for _, h in cases]
        self.initial = 0x18000000
        self.checkpoints = {self.initial: [x.copy() for x in self.h]}
        for tile, h in enumerate(self.h):
            self.image(self.initial, state_rows(h)[0], tile)
        self.p.fence()
        self.load_h(self.initial)

    def image(self, base, rows, tile):
        for port, words in enumerate(pack_rows(rows)):
            self.memory.extend(
                f"M {4*tile+port} {base+32*i:x} {w:064x}" for i, w in enumerate(words)
            )

    def expect(self, base, rows, masks, tile):
        for port, (words, valid) in enumerate(zip(pack_rows(rows), pack_rows(masks))):
            self.expected.extend(
                f"E {4*tile+port} {base+32*i:x} {w:064x} {m:064x}"
                for i, (w, m) in enumerate(zip(words, valid))
                if m
            )

    def load_h(self, address):
        self.p.fence()
        self.p.dma(address=address, region=1, local=512, beats=4 * self.dk)
        self.p.fence(dma=True)
        self.h = [x.copy() for x in self.checkpoints[address]]

    def store_h(self, name):
        address = self.initial + len(self.checkpoints) * 0x10000
        self.p.fence()
        self.p.dma(store=True, address=address, region=1, local=512, beats=4 * self.dk)
        tag = self.p.fence()
        self.sections.append((name, tag))
        self.checkpoints[address] = [x.copy() for x in self.h]
        for tile, h in enumerate(self.h):
            self.expect(address, *state_rows(h), tile)
        return address

    def execute(self, name, sequences):
        n = len(sequences[0][0])
        base = 0x100000 + self.index * 0x10000
        out = 0x1000000 + self.index * 0x10000
        self.index += 1
        histories = []
        for tile, data in enumerate(sequences):
            self.image(base, raw_rows(data), tile)
            self.h[tile], o, e = step(data, self.h[tile])
            self.expect(out, *result_rows(o, e), tile)
            histories.append((data, e))
        self.p.dma(address=base, local=0, beats=12 * n)
        self.p.fence(dma=True)
        self.p.gdn_recurrent(self.dk, self.dv, n)
        self.p.fence()
        self.p.dma(store=True, address=out, local=1024, beats=6 * n)
        tag = self.p.fence()
        self.sections.append((name, tag))
        self.ops.append(dict(name=name, kind="step", tokens=n, input=base, output=out))
        return dict(raw=base, out=out, histories=histories, tokens=n)

    def replay(self, name, checkpoint, history, count):
        self.load_h(checkpoint)
        if count:
            for i in range(count):
                self.p.dma(address=history["raw"] + (12 * i + 2) * 32, local=12 * i + 2, beats=2)
                self.p.dma(address=history["raw"] + (12 * i + 6) * 32, local=12 * i + 6, beats=2)
                self.p.dma(address=history["out"] + (6 * i + 2) * 32, local=12 * i + 8, beats=4)
            self.p.fence(dma=True)
            self.p.gdn_recurrent(self.dk, self.dv, count, rebuild=True)
            tag = self.p.fence()
            self.sections.append((name, tag))
            for tile, (data, e) in enumerate(history["histories"]):
                self.h[tile] = rebuild(tuple(x[:count] for x in data), e[:count], self.h[tile])
        self.ops.append(
            dict(
                name=name,
                kind="cached_e_rebuild",
                tokens=count,
                checkpoint=checkpoint,
                cache=history["out"],
                source="actual previous RTL DMA_STORE; no host E values",
            )
        )

    def finish(self, precision, extra):
        folder = Path(
            os.environ.get("DELTAFLOW_EXAMPLE_OUTPUT", str(ROOT / ".local/examples" / self.name))
        )
        if (folder / "result_16.json").exists():
            raise ValueError("completed fixture is immutable")
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "program_16.hex").write_text(self.p.hex())
        (folder / "memory_16.txt").write_text("\n".join(self.memory + self.expected) + "\n")
        (folder / "sections_16.txt").write_text("".join(f"{n} {t}\n" for n, t in self.sections))
        (folder / "workload_16.json").write_text(
            json.dumps(
                dict(
                    instructions=len(self.p.words),
                    expected_hbm_beats=len(self.expected),
                    dk=self.dk,
                    dv=self.dv,
                    heads=4,
                    side=16,
                    main_macs=0,
                    operations=self.ops,
                    scope="existing 64 FP32 auxiliary lanes/tile; no new MACs or PE registers",
                    **extra,
                ),
                indent=2,
            )
            + "\n"
        )
        (folder / "precision.json").write_text(json.dumps(precision, indent=2) + "\n")
        print(self.name, len(self.p.words), flush=True)


def decode(dk=128, dv=128, tokens=4):
    cases = [make_tokens(tokens, dk, dv, 101 + t) for t in range(4)]
    f = Fixture(f"decode_d{dk}_v{dv}_t{tokens}", cases)
    if dk != 128:
        for data, _ in cases:
            data[3][0] = 0
            data[4][-1] = 0
    outputs = [[] for _ in cases]
    for i in range(tokens):
        data = [tuple(x[i : i + 1] for x in d) for d, _ in cases]
        # State stays resident, and the following DMA waits for the previous O/E B.
        h = f.execute(f"decode_{i}", data)
        for tile, (d, _) in enumerate(h["histories"]):
            outputs[tile].append(projection(d[0][0], f.h[tile]))
    f.store_h("final_state")
    precision = []
    for tile, (data, h) in enumerate(cases):
        rh, ro = recurrence(*data, h)
        precision.append(dict(h=error(f.h[tile], rh), o=error(bf16(np.asarray(outputs[tile])), ro)))
    f.finish(precision, dict(logical_tokens=tokens, kind="causal_native_decode"))


def replay(accepted):
    cases = [make_tokens(9, 128, 128, 301 + t) for t in range(4)]
    f = Fixture(f"replay_accept{accepted}", cases)
    history = f.execute("history", [tuple(x[:4] for x in d) for d, _ in cases])
    wanted = [h.copy() for h in f.h]
    f.replay("periodic_rebuild", f.initial, history, 4)
    for a, b in zip(f.h, wanted):
        assert np.array_equal(bits(a), bits(b))
    checkpoint = f.store_h("checkpoint_commit")
    draft = f.execute("draft", [tuple(x[4:8] for x in d) for d, _ in cases])
    f.replay("accepted_rebuild", checkpoint, draft, accepted)
    # Old history is now folded into Hc: only newly accepted draft entries replay.
    f.execute("next_decode", [tuple(x[8:9] for x in d) for d, _ in cases])
    f.store_h("final_state")
    precision = []
    for tile, (data, h) in enumerate(cases):
        selected = np.r_[np.arange(4 + accepted), 8]
        rh, _ = recurrence(*tuple(x[selected] for x in data), h)
        precision.append(dict(h=error(f.h[tile], rh)))
    f.finish(
        precision,
        dict(
            kind="hbmcached_correction_replay",
            checkpoint_interval=4,
            accepted=accepted,
            logical_tokens=5 + accepted,
            retired_history=4,
        ),
    )
