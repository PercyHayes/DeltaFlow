"""Numerical and HBM packing helpers shared by all example programs."""

import numpy as np
from reference import bf16


def bits(x):
    return np.asarray(x, np.float32).view(np.uint32)


def state_rows(h):
    dk, dv = h.shape
    rows = []
    masks = []
    u = bits(h)
    for key in range(dk):
        for half in range(2):
            row = [0] * 16
            mask = [0] * 16
            for c in range(64):
                if half * 64 + c < dv:
                    row[c // 4] |= int(u[key, half * 64 + c]) << (32 * (c % 4))
                    mask[c // 4] |= 0xFFFFFFFF << (32 * (c % 4))
            rows.append(row)
            masks.append(mask)
    return rows, masks


def output_rows(o):
    rows = []
    masks = []
    u = bits(o) >> 16
    for values in u:
        row = [0] * 16
        mask = [0] * 16
        for c, x in enumerate(values):
            row[c // 8] |= int(x) << (16 * (c % 8))
            mask[c // 8] |= 65535 << (16 * (c % 8))
        rows.append(row)
        masks.append(mask)
    return rows, masks


def make_tokens(tokens, dk, dv, seed, *, channel_gates=False):
    rng = np.random.default_rng(seed)
    q = rng.normal(size=(tokens, dk))
    q = bf16(q / np.linalg.norm(q, axis=1, keepdims=True))
    k = rng.normal(size=(tokens, dk))
    k = bf16(k / np.linalg.norm(k, axis=1, keepdims=True))
    v = bf16(rng.normal(size=(tokens, dv)))
    g = rng.uniform(0.85, 0.999, size=(tokens, dk) if channel_gates else tokens).astype(np.float32)
    beta = rng.uniform(0.1, 0.9, size=tokens).astype(np.float32)
    h = rng.normal(scale=0.02, size=(dk, dv)).astype(np.float32)
    return (q, k, v, g, beta), h
