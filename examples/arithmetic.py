"""Separate operand/accumulator formats and explicit RNE/FMA boundaries.

Host libm fmaf with explicit DAZ/FTZ follows the inspected EasyFloat contract.
NaNs use the same positive canonical quiet representation as EasyFloat.
"""

import ctypes
from pathlib import Path
import subprocess
import tempfile

import numpy as np


def bf16(x):
    x = np.asarray(x, dtype=np.float32)
    u = x.view(np.uint32)
    u = np.where((u & 0x7F800000) == 0, u & np.uint32(0x80000000), u).astype(np.uint32)
    finite = (u & 0x7F800000) != 0x7F800000
    with np.errstate(over="ignore"):
        rounded = (u + np.uint32(0x7FFF) + ((u >> 16) & 1)) & np.uint32(0xFFFF0000)
    # EasyFloat preserves signed infinity and canonicalizes all NaN payloads.
    special = np.where((u & 0x007FFFFF) != 0, np.uint32(0x7FC00000), u)
    return (
        np.where(finite, rounded, special & np.uint32(0xFFFF0000))
        .astype(np.uint32)
        .view(np.float32)
    )


_NATIVE = None


def native():
    global _NATIVE
    if _NATIVE is None:
        source = Path(__file__).with_name("arithmetic_native.c")
        build = source.parent.parent / ".local" / "reference"
        build.mkdir(parents=True, exist_ok=True)
        # Each process compiles its own object. A concurrent fixture generator
        # must never overwrite a shared object while another process loads it.
        # Linux keeps the loaded mapping valid after this temporary file closes.
        with tempfile.TemporaryDirectory(prefix="arithmetic_", dir=build) as temporary:
            lib = Path(temporary) / "libarithmetic.so"
            subprocess.run(
                [
                    "gcc",
                    "-O2",
                    "-std=c11",
                    "-fno-fast-math",
                    "-ffp-contract=off",
                    "-shared",
                    "-fPIC",
                    str(source),
                    "-o",
                    str(lib),
                    "-lm",
                ],
                check=True,
            )
            library = ctypes.CDLL(str(lib))
        pointer = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        library.panel_fma.argtypes = [pointer] * 4 + [ctypes.c_size_t]
        library.panel_dot.argtypes = [pointer] * 3 + [ctypes.c_int] * 3
        library.panel_fma.restype = library.panel_dot.restype = None
        # Publish only a fully configured handle to other Python threads.
        _NATIVE = library
    return _NATIVE


class Arithmetic:
    def __init__(self, mode="fp32"):
        assert mode in ("fp64", "fp32", "mixed", "mixed_solve")
        self.mode = mode
        self.dtype = np.float64 if mode == "fp64" else np.float32

    def acc(self, x):
        return np.asarray(x, dtype=self.dtype)

    def elem(self, x):
        return bf16(x) if self.mode in ("mixed", "mixed_solve") else self.acc(x)

    def fma(self, a, b, c):
        a, b, c = np.broadcast_arrays(self.acc(a), self.acc(b), self.acc(c))
        if self.mode == "fp64":
            return a * b + c
        shape = a.shape
        a, b, c = [np.ascontiguousarray(x) for x in (a, b, c)]
        out = np.empty(shape, dtype=np.float32)
        native().panel_fma(a, b, c, out, out.size)
        return out

    def mul(self, a, b):
        return self.fma(a, b, 0.0)

    def add(self, a, b):
        return self.fma(1.0, a, b)

    def dot(self, a, b, initial=None, full=False):
        cast = self.acc if full else self.elem
        a, b = [np.ascontiguousarray(cast(x)) for x in (a, b)]
        assert a.ndim == b.ndim == 2 and a.shape[1] == b.shape[0]
        out = (
            np.zeros((len(a), b.shape[1]), dtype=self.dtype)
            if initial is None
            else self.acc(initial).copy()
        )
        assert out.shape == (len(a), b.shape[1])
        if self.mode == "fp64":
            return out + a @ b
        native().panel_dot(a, b, out, len(a), b.shape[1], a.shape[1])
        return out

    def product(self, g):
        out = np.ones(g.shape[1], dtype=self.dtype)
        for row in g:
            out = self.mul(out, row)
        return out

    def prefix(self, g):
        out, prefix = np.ones(g.shape[1], dtype=self.dtype), []
        for row in g:
            out = self.mul(out, row)
            prefix.append(out)
        return np.asarray(prefix, dtype=self.dtype).reshape(g.shape)
