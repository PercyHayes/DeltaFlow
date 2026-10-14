"""128-bit little-endian compute ISA, version 1."""

from dataclasses import dataclass
from enum import IntEnum


class Opcode(IntEnum):
    CLEAR = 0
    GEMM_ACC = 1
    STORE = 2
    FENCE = 3
    SEED = 4


FIELDS = (
    ("opcode", 0, 8),
    ("tile_mask", 8, 4),
    ("context_mask", 12, 4),
    ("tag", 16, 16),
    ("k_count", 32, 16),
    ("input_base", 48, 16),
    ("output_base", 64, 16),
    ("m_valid", 80, 16),
    ("n_valid", 96, 16),
)


@dataclass(frozen=True)
class Instruction:
    opcode: int
    tile_mask: int = 15
    context_mask: int = 15
    tag: int = 0
    k_count: int = 0
    input_base: int = 0
    output_base: int = 0
    m_valid: int = 128
    n_valid: int = 128

    def encode(self):
        word = 0
        for name, shift, width in FIELDS:
            value = int(getattr(self, name))
            if not 0 <= value < 1 << width:
                raise ValueError(f"{name} outside {width}-bit field")
            word |= value << shift
        if (
            self.opcode not in Opcode._value2member_map_
            or not self.tile_mask
            or not self.context_mask
        ):
            raise ValueError("invalid opcode or empty mask")
        return word

    def words(self):
        value = self.encode()
        return [(value >> i) & 0xFFFFFFFF for i in range(0, 128, 32)]

    @classmethod
    def decode(cls, value):
        if not 0 <= value < 1 << 112:
            raise ValueError("reserved bits must be zero")
        result = cls(**{n: (value >> s) & ((1 << w) - 1) for n, s, w in FIELDS})
        result.encode()
        return result
