"""Shared GDN numeric reference, independent of any cycle scheduler.

Ordered FP32 FMA and explicit BF16 boundaries model the hardware arithmetic.
FP64 recurrence and algebra provide independent mathematical comparisons.
The scalar backend is implemented in arithmetic.py and arithmetic_native.c.
"""

import numpy as np
from arithmetic import Arithmetic, bf16

AR = Arithmetic("fp32")


def gates(g):
    c = len(g)
    gate = np.eye(c, dtype=np.float32)
    prefix = np.empty(c, dtype=np.float32)
    p = np.float32(1)
    for i in range(c):
        p = AR.mul(p, g[i])
        prefix[i] = p
        if i:
            gate[i, :i] = AR.mul(gate[i - 1, :i], g[i])
    return gate, prefix


def prepare(k, v, g, beta, b, route="direct"):
    gate, prefix = gates(g)
    l = np.tril(AR.mul(AR.mul(AR.dot(k, k.T), gate), beta[:, None]), -1)
    rhs = np.concatenate(
        (AR.mul(beta[:, None], v), AR.mul(AR.mul(beta, prefix)[:, None], k)), axis=1
    )
    x = np.zeros_like(rhs)
    coeff = {}
    for i in range(4):
        s = slice(i * b, (i + 1) * b)
        local = l[s, s]
        inverse = np.eye(b, dtype=np.float32)
        for r in range(1, b):
            inverse[r, :r] = -local[r, :r]
            for j in range(1, r):
                inverse[r, :j] = AR.fma(-local[r, j], inverse[j, :j], inverse[r, :j])
        coeff[i, i] = bf16(inverse)
        residual = rhs[s].copy()
        for j in range(i):
            prev = slice(j * b, (j + 1) * b)
            coeff[i, j] = -bf16(l[s, prev])
            residual = AR.dot(coeff[i, j], x[prev], initial=residual)
        x[s] = bf16(AR.dot(bf16(inverse), bf16(residual)))
    assert route in ("direct", "inverse")
    if route == "inverse":
        blocks = {}
        t = np.zeros((4 * b, 4 * b), np.float32)
        for i in range(4):
            blocks[i, i] = coeff[i, i]
            for j in range(i):
                partial = np.zeros((b, b), np.float32)
                for kidx in range(j, i):
                    partial = AR.dot(coeff[i, kidx], blocks[kidx, j], initial=partial)
                blocks[i, j] = bf16(AR.dot(coeff[i, i], bf16(partial)))
            for j in range(i + 1):
                t[i * b : (i + 1) * b, j * b : (j + 1) * b] = blocks[i, j]
        x = bf16(AR.dot(t, bf16(rhs)))
    return x[:, : v.shape[1]], x[:, v.shape[1] :]


def factors(q, k, g):
    gate, prefix = gates(g)
    return dict(
        qbar=bf16(AR.mul(prefix[:, None], q)),
        n=bf16(AR.mul(k.T, gate[-1])),
        m=bf16(AR.mul(AR.dot(q, k.T), gate)),
        d=prefix[-1],
    )


def suffix(u, w, h, f, *, state_epilogue=False):
    snapshot = bf16(h)
    e = bf16(AR.dot(-w, snapshot, initial=u))
    z = AR.dot(f["qbar"], snapshot)
    updated = (
        AR.fma(f["d"], h, AR.dot(f["n"], e))
        if state_epilogue
        else AR.dot(f["n"], e, initial=AR.mul(f["d"], h))
    )
    return (updated, bf16(AR.dot(f["m"], e, initial=z)), e)


def recurrence(q, k, v, g, beta, h):
    q, k, v, g, beta, h = [np.asarray(a, np.float64) for a in (q, k, v, g, beta, h)]
    h = h.copy()
    out = []
    for i in range(len(q)):
        h *= g[i]
        e = beta[i] * (v[i] - k[i] @ h)
        h += k[i, :, None] * e
        out.append(q[i] @ h)
    return h, np.asarray(out)


def algebra(q, k, v, g, beta, h):
    q, k, v, g, beta, h = [np.asarray(a, np.float64) for a in (q, k, v, g, beta, h)]
    c = len(g)
    gate = np.eye(c)
    prefix = np.cumprod(g)
    for i in range(1, c):
        gate[i, :i] = gate[i - 1, :i] * g[i]
    a = np.eye(c) + np.tril(beta[:, None] * (k @ k.T) * gate, -1)
    rhs = beta[:, None] * (v - prefix[:, None] * (k @ h))
    e = np.linalg.solve(a, rhs)
    return prefix[-1] * h + (k.T * gate[-1]) @ e, prefix[:, None] * (q @ h) + ((q @ k.T) * gate) @ e


def error(x, ref):
    return dict(
        relative_frobenius=float(np.linalg.norm(x - ref) / max(np.linalg.norm(ref), 1e-30)),
        max_absolute=float(np.max(np.abs(x - ref))),
        finite=bool(np.isfinite(x).all()),
    )
