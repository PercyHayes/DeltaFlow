"""Unified compute/DMA instruction stream and port-striped HBM packing.

All modes share this stream, tag space, tile masks and physical local addresses.
Offsets are relative to the independently bound (tile, port) HBM window.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from .isa import Instruction, Opcode


class SystemOpcode(IntEnum):
    DMA_LOAD = 0x10
    DMA_STORE = 0x11
    SYSTEM_FENCE = 0x12
    BUFFER_FENCE = 0x13
    DMA_FENCE = 0x14
    GDN_CHUNK = 0x21
    GDN_PREPARE = 0x22
    GDN_RELEASE = 0x23
    GDN_STEP = 0x24
    GDN_REBUILD = 0x25


def _field(value, width, name):
    if not isinstance(value, int) or not 0 <= value < 1 << width:
        raise ValueError(f"{name} outside {width} bits")
    return value


def dma_word(
    *,
    store=False,
    tile_mask=15,
    port_mask=15,
    tag=0,
    address=0,
    local=0,
    beats=1,
    region=0,
    stride2=False,
):
    for v, w, n in [
        (tile_mask, 4, "tile_mask"),
        (port_mask, 4, "port_mask"),
        (tag, 16, "tag"),
        (address, 48, "address"),
        (local, 16, "local"),
        (beats, 16, "beats"),
    ]:
        _field(v, w, n)
    if not tile_mask or not port_mask or region not in (0, 1):
        raise ValueError("nonempty tile/port masks and Operand=0 or Work=1 required")
    if address % 32 or not beats or address + beats * 32 >= 1 << 48:
        raise ValueError("aligned, nonempty, nonwrapping DMA required")
    if not isinstance(stride2, bool):
        raise ValueError("stride2 must be boolean")
    if local + (beats - 1) * (2 if stride2 else 1) >= (2048 if region == 0 else 1024):
        raise ValueError("DMA exceeds frozen SRAM capacity")
    return (
        int(SystemOpcode.DMA_STORE if store else SystemOpcode.DMA_LOAD)
        | tile_mask << 8
        | port_mask << 12
        | tag << 16
        | address << 32
        | local << 80
        | beats << 96
        | region << 112
        | int(stride2) << 113
    )


def lane_location(index, port):
    """A 256-bit beat joins two 128-bit banks; successive beats alternate groups."""
    if not 0 <= index < 2048 or not 0 <= port < 4:
        raise ValueError("invalid lane location")
    return index // 2, 8 * (index % 2) + 2 * port


def pack_rows(rows, groups=(0, 1)):
    """16-bank rows -> four contiguous byte-addressed AXI lane streams."""
    if tuple(groups) not in ((0, 1), (0,), (1,)):
        raise ValueError("groups must be (0,1), (0,) or (1,)")
    streams = [[] for _ in range(4)]
    for row in rows:
        if len(row) != 16 or any(not 0 <= int(v) < 1 << 128 for v in row):
            raise ValueError("each row must contain sixteen 128-bit words")
        for port in range(4):
            for group in groups:
                b = 8 * group + 2 * port
                streams[port].append(int(row[b]) | int(row[b + 1]) << 128)
    return streams


@dataclass
class Program:
    """One executable stream; fences order both DMA and compute engines.

    DMA may follow GEMM_ACC for the other input buffer before BUFFER_FENCE.
    This waits for input consumption and DMA, preserving arithmetic in flight. STORE is
    ordered in the tile compute queue; fence before output DMA to expose writes.
    GDN_CHUNK uses the same SRAM, DMA, fence and completion contract.
    """

    side: int = 16
    words: list[int] = field(default_factory=list)
    _tag: int = 0

    def __post_init__(self):
        if self.side not in (4, 16):
            raise ValueError("system side must be 4 or 16")

    def tag(self):
        self._tag += 1
        if self._tag > 65535:
            raise ValueError("drain and start a new tag epoch")
        return self._tag

    def dma(self, **kw):
        self.words.append(dma_word(tag=self.tag(), **kw))
        return self._tag

    def fence(self, tile_mask=15, *, buffers=False, dma=False):
        if not 0 < tile_mask < 16:
            raise ValueError("empty or invalid tile mask")
        if buffers and dma:
            raise ValueError("choose one fence scope")
        self.words.append(
            int(
                SystemOpcode.DMA_FENCE
                if dma
                else SystemOpcode.BUFFER_FENCE if buffers else SystemOpcode.SYSTEM_FENCE
            )
            | tile_mask << 8
            | self.tag() << 16
        )
        return self._tag

    def gemm(self, opcode, **kw):
        self.words.append(
            Instruction(
                opcode,
                tag=self.tag(),
                **({"m_valid": 8 * self.side, "n_valid": 8 * self.side} | kw),
            ).encode()
        )
        return self._tag

    def _gdn(self, opcode, dk, dv, tile_mask, slot, prepared=False, next_chunk=False):
        if (
            not 1 <= dk <= 8 * self.side
            or not 1 <= dv <= 8 * self.side
            or not 0 < tile_mask < 16
            or slot not in (0, 1)
        ):
            raise ValueError("invalid GDN dimensions, slot or mask")
        self.words.append(
            int(opcode)
            | tile_mask << 8
            | self.tag() << 16
            | dk << 32
            | dv << 40
            | slot << 48
            | int(prepared) << 49
            | int(next_chunk) << 50
        )
        return self._tag

    def gdn_chunk(self, dk=128, dv=128, tile_mask=15, *, slot=0, prepared=False, next_chunk=False):
        return self._gdn(SystemOpcode.GDN_CHUNK, dk, dv, tile_mask, slot, prepared, next_chunk)

    def gdn_prepare(self, dk=128, dv=128, tile_mask=15, *, slot=1):
        return self._gdn(SystemOpcode.GDN_PREPARE, dk, dv, tile_mask, slot)

    def gdn_release(self, slot, tile_mask=15):
        if slot not in (0, 1) or not 0 < tile_mask < 16:
            raise ValueError("invalid GDN slot/mask")
        self.words.append(
            int(SystemOpcode.GDN_RELEASE) | tile_mask << 8 | self.tag() << 16 | slot << 48
        )
        return self._tag

    def hex(self):
        return "".join(f"{word:032x}\n" for word in self.words)

    def gdn_recurrent(
        self, dk=128, dv=128, tokens=1, *, state_base=256, rebuild=False, tile_mask=15
    ):
        """Native short recurrence or cached-E rebuild; all data remains in SRAM/HBM.

        Operand: six rows/token Q,K,V,g/beta,Elo,Ehi; scratch384..399;
        output512+3*token: BF16 O, FP32 Elo/Ehi. Work H: state_base+2*key+half.
        REBUILD consumes K/g/E and emits only updated H. No PE additions.
        """
        if not 1 <= dk <= 8 * self.side or not 1 <= dv <= 8 * self.side or not 1 <= tokens <= 64:
            raise ValueError("invalid recurrent dimensions or token count")
        if not 0 <= state_base <= 512 - 2 * dk or not 0 < tile_mask < 16:
            raise ValueError("invalid state base or tile mask")
        opcode = SystemOpcode.GDN_REBUILD if rebuild else SystemOpcode.GDN_STEP
        self.words.append(
            int(opcode)
            | tile_mask << 8
            | self.tag() << 16
            | dk << 32
            | dv << 40
            | tokens << 48
            | state_base << 55
        )
        return self._tag


def gemm_operand_rows(a, b, row, column, k_start, k_count, side=16):
    """Pack BF16 *bit patterns* for one tile/segment, padding M/N tails with zero.

    No float conversion or numerical oracle is hidden in this layout routine.
    A is M×K, B is K×N; their common K dimension must match.
    """
    t = 4 * side
    if side not in (4, 16) or not a or not b or not b[0]:
        raise ValueError("nonempty matrices and supported side required")
    m = len(a)
    k = len(b)
    n = len(b[0])
    if any(len(x) != k for x in a) or any(len(x) != n for x in b):
        raise ValueError("ragged matrices or K mismatch")
    if min(row, column, k_start) < 0 or not 1 <= k_count <= 256 or k_start + k_count > k:
        raise ValueError("invalid operand slice")
    result = []
    for kk in range(k_start, k_start + k_count):
        for half in range(2):
            words = [0] * 16
            for lane in range(t):
                r = row + half * t + lane
                c = column + half * t + lane
                av = int(a[r][kk]) if r < m else 0
                bv = int(b[kk][c]) if c < n else 0
                _field(av, 16, "A BF16 bits")
                _field(bv, 16, "B BF16 bits")
                words[lane // 8] |= av << (16 * (lane % 8))
                words[t // 8 + lane // 8] |= bv << (16 * (lane % 8))
            result.append(words)
    return result


def compile_system_gemm(m, n, k, side=16, k_block=256, *, independent=False):
    """Executable DMA + compute epochs for arbitrary positive GEMM dimensions.

    Each epoch computes one 2×2-tile output supertile, or (independent=True)
    the same output coordinates of four independent matrices. The host binds each port's
    input/output windows, packs tensors as described, submits the .hex stream,
    and drains the final SYSTEM_FENCE before tag reuse/next epoch. Work reuse
    waits for actual output AXI B. No hidden payload adapter is used.

    When independent=True and all participating tiles share the same m_valid/n_valid,
    CLEAR/ACC/STORE are broadcast with tile_mask covering all tiles simultaneously,
    reducing compute instruction count by 75%; DMA and fences are unchanged.
    """
    if side not in (4, 16) or min(m, n, k, k_block) <= 0 or k_block > 256:
        raise ValueError("positive dimensions, supported side, K segment <=256 required")
    t = 4 * side
    segments = (k + k_block - 1) // k_block
    stride = 4 * k_block * 32
    output_address = (segments * stride + 4095) // 4096 * 4096
    epochs = []
    patch_step = 2 * t if independent else 4 * t
    for mr in range(0, m, patch_step):
        for nc in range(0, n, patch_step):
            tiles = []
            for tile in range(4):
                r = mr + (0 if independent else (tile // 2) * 2 * t)
                c = nc + (0 if independent else (tile % 2) * 2 * t)
                if r < m and c < n:
                    tiles.append(
                        {
                            "tile": tile,
                            "row": r,
                            "column": c,
                            "m_valid": min(2 * t, m - r),
                            "n_valid": min(2 * t, n - c),
                        }
                    )
            mask = sum(1 << item["tile"] for item in tiles)
            # In independent mode all tiles share identical m_valid/n_valid, so CLEAR/ACC/STORE
            # can be issued once with tile_mask=mask rather than four separate per-tile instructions.
            # Compute instructions shrink by 75%; total reduction depends on DMA/fence count.
            mv_set = set(item["m_valid"] for item in tiles)
            nv_set = set(item["n_valid"] for item in tiles)
            can_broadcast = independent and len(mv_set) == 1 and len(nv_set) == 1
            bcast_mv = tiles[0]["m_valid"] if can_broadcast else None
            bcast_nv = tiles[0]["n_valid"] if can_broadcast else None
            p = Program(side)
            inputs = []

            def load(segment):
                count = min(k_block, k - segment * k_block)
                p.dma(
                    tile_mask=mask,
                    address=segment * stride,
                    local=(segment % 2) * 1024,
                    beats=4 * count,
                )
                inputs.append(
                    {
                        "segment": segment,
                        "k_start": segment * k_block,
                        "k_count": count,
                        "hbm_offset_per_port": segment * stride,
                        "local_row": (segment % 2) * 512,
                        "bytes_per_port": 4 * count * 32,
                    }
                )

            load(0)
            p.fence(mask, buffers=True)
            if can_broadcast:
                p.gemm(Opcode.CLEAR, tile_mask=mask, m_valid=bcast_mv, n_valid=bcast_nv)
            else:
                for item in tiles:
                    p.gemm(
                        Opcode.CLEAR,
                        tile_mask=1 << item["tile"],
                        m_valid=item["m_valid"],
                        n_valid=item["n_valid"],
                    )
            for segment in range(segments):
                count = min(k_block, k - segment * k_block)
                if can_broadcast:
                    p.gemm(
                        Opcode.GEMM_ACC,
                        tile_mask=mask,
                        m_valid=bcast_mv,
                        n_valid=bcast_nv,
                        k_count=count,
                        input_base=(segment % 2) * 512,
                    )
                else:
                    for item in tiles:
                        p.gemm(
                            Opcode.GEMM_ACC,
                            tile_mask=1 << item["tile"],
                            m_valid=item["m_valid"],
                            n_valid=item["n_valid"],
                            k_count=count,
                            input_base=(segment % 2) * 512,
                        )
                if segment + 1 < segments:
                    load(segment + 1)
                p.fence(mask, buffers=True)
            if can_broadcast:
                p.gemm(Opcode.STORE, tile_mask=mask, m_valid=bcast_mv, n_valid=bcast_nv)
            else:
                for item in tiles:
                    p.gemm(
                        Opcode.STORE,
                        tile_mask=1 << item["tile"],
                        m_valid=item["m_valid"],
                        n_valid=item["n_valid"],
                    )
            p.fence(mask)
            p.dma(
                store=True,
                tile_mask=mask,
                port_mask=3 if side == 4 else 15,
                address=output_address,
                region=1,
                beats=4 * t if side == 4 else 8 * t,
                stride2=side == 4,
            )
            finish = p.fence(mask)
            epochs.append(
                {
                    "epoch": len(epochs),
                    "tiles": tiles,
                    "inputs": inputs,
                    "output_hbm_offset_per_port": output_address,
                    "output_format": "fp32",
                    "final_fence_tag": finish,
                    "words": [f"{w:032x}" for w in p.words],
                    "broadcast": can_broadcast,
                }
            )
    return {
        "schema_version": 1,
        "backend": "FourTileSystem",
        "shape": [m, n, k],
        "side": side,
        "isa": "unified GEMM/GDN/DMA; GEMM compute v1 compatible",
        "epochs": epochs,
        "mapping": "four independent matrices" if independent else "four cooperating tiles",
        "memory_contract": "400KiB/tile at b16: Operand 256KiB, Work 128KiB, Gate 16KiB; see docs/architecture.md",
        "layout": "port-striped native rows; independent (tile,port) windows; M/N zero padding",
        "runtime": "wait for successful final SYSTEM_FENCE completion before next epoch, tag reuse or HBM window reuse",
        "limits": "no automatic host tensor packing cost, MMIO bridge or cross-supertile output overlap claim",
    }


def gdn_gate_rows(gates, side=16):
    """Data packer, never an instruction: activated FP32 (g,beta) to DMA rows.

    High banks of raw-relative rows252..255 are unused by Q/K/V or Gram tail.
    DMA local=505+1536*slot, stride2=True, side/4 beats per port.
    """
    if side not in (4, 16) or len(gates) != 4 * side:
        raise ValueError("full gate chunk required")
    rows = [[0] * 16 for _ in range(side // 4)]
    for i, (decay, beta) in enumerate(gates):
        if not 0 <= decay <= 0x3F800000 or not 0 <= beta <= 0x3F800000:
            raise ValueError("invalid activated FP32 gate")
        rows[i // 16][8 + (i % 16) // 2] |= (decay | beta << 32) << (64 * (i % 2))
    return rows


def emit_gdn_prefill(
    program,
    chunks,
    *,
    dk=128,
    dv=128,
    tile_mask=15,
    input_address=0x100000,
    input_stride=0x10000,
    output_address=0x1000000,
    output_stride=0x10000,
    initial_state_address=0x18000000,
    final_state_address=0xB0000,
    overlap=True,
):
    """Append one bounded head sequence per selected tile to the common stream.

    Each port window has QK at chunk+0, V at +0x4000, gates at +0xc000.
    BF16 O and final FP32 H use native row striping. Offsets are independent
    across tile/port windows. Inputs are already packed, never in instructions.
    An optional future reservation precedes DMA readiness; PREPARE is its data
    commit after DMA_FENCE. Exactly two raw/O slots and one resident H are used.
    """
    if not isinstance(chunks, int) or chunks < 1 or input_stride < 0xC100 or output_stride < 0x1000:
        raise ValueError("positive bounded chunks and disjoint per-chunk windows required")
    if not 1 <= dk <= 8 * program.side or not 1 <= dv <= 8 * program.side or not 0 < tile_mask < 16:
        raise ValueError("invalid GDN shape/mask")
    # The Direct AXI4 integration binds one 4-GiB window to each port. Validate
    # the complete layout before emitting any descriptor; output aliasing with
    # future chunks can otherwise silently replace inputs during long prefill.
    for name, value in (
        ("input_address", input_address),
        ("input_stride", input_stride),
        ("output_address", output_address),
        ("output_stride", output_stride),
        ("initial_state_address", initial_state_address),
        ("final_state_address", final_state_address),
    ):
        if not isinstance(value, int) or value < 0 or value % 32:
            raise ValueError(f"{name} must be a nonnegative, 32-byte-aligned integer")
    output_beats = 4 * program.side if program.side == 4 else 8 * program.side
    final_state_beats = (2 if program.side == 4 else 4) * dk
    ranges = (
        (
            "input",
            input_address,
            input_address + (chunks - 1) * input_stride + 0xC000 + 32 * (program.side // 4),
        ),
        (
            "output",
            output_address,
            output_address + (chunks - 1) * output_stride + 32 * output_beats,
        ),
        ("initial H", initial_state_address, initial_state_address + 32 * 4 * dk),
        ("final H", final_state_address, final_state_address + 32 * final_state_beats),
    )
    for index, (name, start, end) in enumerate(ranges):
        if end > 1 << 32:
            raise ValueError(f"{name} exceeds the 4-GiB HBM port window")
        for other, other_start, other_end in ranges[:index]:
            # Initial H is consumed before the first chunk. The final export
            # may replace it in place after all computation and DMA retire.
            if name == "final H" and other == "initial H":
                continue
            if max(start, other_start) < min(end, other_end):
                raise ValueError(f"HBM {name} and {other} ranges overlap")
    # The program is deliberately bounded by the tag epoch. Do not partially
    # emit a sequence that would wrap tags while operations remain in flight.
    needed = 9 * chunks + 3 if overlap else 8 * chunks + 4
    if program._tag + needed > 65535:
        raise ValueError("sequence exceeds tag epoch")
    p = program
    t = 4 * p.side
    ports = 3 if p.side == 4 else 15
    p.dma(tile_mask=tile_mask, address=initial_state_address, region=1, local=512, beats=4 * dk)

    def load(chunk, wait=True):
        base = input_address + chunk * input_stride
        slot = chunk % 2
        p.dma(tile_mask=tile_mask, address=base, local=1536 * slot, beats=2 * dk)
        p.dma(
            tile_mask=tile_mask,
            address=base + 0x4000,
            local=256 + 1536 * slot,
            beats=dv,
            stride2=True,
        )
        p.dma(
            tile_mask=tile_mask,
            address=base + 0xC000,
            local=505 + 1536 * slot,
            beats=p.side // 4,
            stride2=True,
        )
        if wait:
            p.fence(tile_mask, dma=True)

    def drain(chunk):
        p.dma(
            store=True,
            tile_mask=tile_mask,
            address=output_address + chunk * output_stride,
            local=1152 + 256 * (chunk % 2),
            beats=t if p.side == 4 else 2 * t,
            port_mask=ports,
            stride2=p.side == 4,
        )
        p.fence(tile_mask, dma=True)
        p.gdn_release(chunk % 2, tile_mask)

    load(0)
    p.gdn_chunk(dk, dv, tile_mask, next_chunk=overlap and chunks > 1)
    if overlap and chunks > 1:
        load(1)
        p.gdn_prepare(dk, dv, tile_mask, slot=1)
    for chunk in range(1, chunks):
        if overlap:
            # Current raw readers retire at the previous PREPARE gate handshake.
            # Fetch chunk+1 into that raw slot during current H/O, before waiting
            # for its O writeback. Input and output leases are independent.
            if chunk + 1 < chunks:
                load(chunk + 1, wait=False)
            p.gdn_chunk(
                dk, dv, tile_mask, slot=chunk % 2, prepared=True, next_chunk=chunk + 1 < chunks
            )
            if chunk + 1 < chunks:
                p.fence(tile_mask, dma=True)
                p.gdn_prepare(dk, dv, tile_mask, slot=(chunk + 1) % 2)
        else:
            load(chunk)
            p.gdn_chunk(dk, dv, tile_mask, slot=chunk % 2)
        drain(chunk - 1)
    drain(chunks - 1)
    p.fence(tile_mask)
    p.dma(
        store=True,
        tile_mask=tile_mask,
        address=final_state_address,
        region=1,
        local=512,
        beats=2 * dk if p.side == 4 else 4 * dk,
        port_mask=ports,
        stride2=p.side == 4,
    )
    return p.fence(tile_mask)
