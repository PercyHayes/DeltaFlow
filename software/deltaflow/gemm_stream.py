"""Continuous four-tile GEMM programs using the existing ISA and 400-KiB SRAM.

Each tile computes its own matrix. All four share shape/local addresses, not
values or external address windows. No instruction carries a tensor payload.
"""

from .system import Program
from .isa import Opcode


def compile_gemm_stream(
    m,
    n,
    k,
    *,
    schedule="overlap",
    k_block=256,
    input_address=0x20000000,
    output_address=0x30000000,
    store_overlap=None,
    defer_output=None,
):
    """Return one fully drained tag epoch, layout descriptors and instruction words.

    serial: broadcast compute, with full output-DMA drain at each output patch.
    overlap: prefetch the next patch, prioritize current inputs and overlap
             previous output DMA with a full K segment. Alternate Work pages.
    reuse_a2: experimental pair of output columns, K order preserved; switch
              partial sums through Work with STORE/SEED, replace only B.

    Both controls default to enabled for overlap (disabled for comparison schedules).
    store_overlap moves initial CLEAR under input DMA and queues final STORE
    without the redundant BUFFER_FENCE. STORE itself waits for ACC retirement;
    its following SYSTEM_FENCE still commits Work and drains every DMA.
    defer_output retains the previous Work page until the current last full K segment,
    prioritizing current input DMA. Only two output pages are live. That same
    SYSTEM_FENCE retires the previous export before the page can be overwritten.

    The caller packs the described HBM images and submits the whole stream. A
    patch is NOT a drained tag epoch. Only final_fence_tag permits mode switch,
    HBM/input-image reuse or tag reuse. Reject overlong streams rather than wrap.
    """
    if schedule not in ("serial", "overlap", "reuse_a2"):
        raise ValueError("unknown GEMM schedule")
    if any(not isinstance(x, int) or x <= 0 for x in (m, n, k, k_block)) or k_block > 256:
        raise ValueError("positive integer dimensions; K segment <=256")
    if store_overlap is None:
        store_overlap = schedule == "overlap"
    if defer_output is None:
        defer_output = store_overlap and schedule == "overlap"
    if not isinstance(store_overlap, bool) or not isinstance(defer_output, bool):
        raise ValueError("overlap controls must be booleans")
    if (store_overlap or defer_output) and schedule != "overlap":
        raise ValueError("store/DMA overlap options require the overlap schedule")
    if defer_output and not store_overlap:
        raise ValueError("deferred output requires early STORE")
    segments = (k + k_block - 1) // k_block
    # A short tail cannot cover an output burst. Launch during the preceding
    # full segment, whose input is already ready, before loading the tail.
    output_segment = max(0, segments - 1 - int(k % k_block != 0))
    patches = [
        dict(
            index=i,
            row=r,
            column=c,
            m_valid=min(128, m - r),
            n_valid=min(128, n - c),
            output_address=output_address + i * 0x10000,
        )
        for i, (r, c) in enumerate((r, c) for r in range(0, m, 128) for c in range(0, n, 128))
    ]
    if input_address % 4096 or output_address % 4096 or min(input_address, output_address) < 0:
        raise ValueError("nonnegative, 4-KiB-aligned HBM windows required")
    images = []
    image_ids = {}
    next_address = input_address
    p = Program(16)

    def image(patch, segment, kind):
        nonlocal next_address
        count = min(k_block, k - segment * k_block)
        key = (kind, patch["row"] if kind == "ab" else 0, patch["column"], segment)
        if key not in image_ids:
            size = count * (128 if kind == "ab" else 64)  # per AXI port
            desc = dict(
                kind=kind,
                row=patch["row"],
                column=patch["column"],
                k_start=segment * k_block,
                k_count=count,
                address=next_address,
                bytes_per_port=size,
            )
            images.append(desc)
            image_ids[key] = desc
            next_address += (size + 4095) // 4096 * 4096
        return image_ids[key]

    def load(patch, segment, slot, kind="ab"):
        desc = image(patch, segment, kind)
        p.dma(
            address=desc["address"],
            local=slot * 1024 + (1 if kind == "b" else 0),
            beats=desc["bytes_per_port"] // 32,
            stride2=kind == "b",
        )

    def compute(op, patch, **kw):
        return p.gemm(op, m_valid=patch["m_valid"], n_valid=patch["n_valid"], **kw)

    def acc(patch, segment, slot):
        compute(
            Opcode.GEMM_ACC,
            patch,
            k_count=min(k_block, k - segment * k_block),
            input_base=slot * 512,
        )

    def export(patch, page):
        patch["output_dma_tag"] = p.dma(
            store=True, address=patch["output_address"], local=page * 512, region=1, beats=512
        )

    if schedule in ("serial", "overlap"):
        # Slot parity is continuous across patch boundaries, even for odd K counts.
        def slot(i, s):
            return (i * segments + s) % 2 if schedule == "overlap" else s % 2

        for i, patch in enumerate(patches):
            if i == 0 or schedule == "serial":
                load(patch, 0, slot(i, 0))
                if store_overlap:
                    compute(Opcode.CLEAR, patch)
                p.fence(buffers=True)
            if not (store_overlap and i == 0):
                compute(Opcode.CLEAR, patch)
            for segment in range(segments):
                acc(patch, segment, slot(i, segment))
                # Preserve the previous output page until this DMA and the
                # following full fence retire. Prioritize current input loads.
                if defer_output and segment == output_segment and i > 0:
                    export(patches[i - 1], (i - 1) % 2)
                if segment + 1 < segments:
                    load(patch, segment + 1, slot(i, segment + 1))
                elif schedule == "overlap" and i + 1 < len(patches):
                    load(patches[i + 1], 0, slot(i + 1, 0))
                # Internal K transitions require ready inputs. At the end,
                # STORE orders itself behind ACC and waits for its retirement;
                # its following full fence also drains the independent DMA.
                if segment + 1 < segments or not store_overlap:
                    p.fence(buffers=True)
            page = i % 2 if schedule == "overlap" else 0
            compute(Opcode.STORE, patch, output_base=page * 256)
            p.fence()  # Work store is committed before any DMA read.
            if not defer_output or i + 1 == len(patches):
                export(patch, page)
            if schedule == "serial":
                p.fence()
        if schedule == "overlap":
            p.fence()
    else:
        # Pair only columns of one output row block so the A slice is identical.
        groups = []
        for patch in patches:
            if groups and len(groups[-1]) == 1 and groups[-1][0]["row"] == patch["row"]:
                groups[-1].append(patch)
            else:
                groups.append([patch])
        for group in groups:
            load(group[0], 0, 0)
            p.fence(buffers=True)
            for segment in range(segments):
                slot = segment % 2
                for page, patch in enumerate(group):
                    if page:
                        # Previous BUFFER_FENCE retired every old B reader. The
                        # STORE may run while DMA replaces only the high banks.
                        load(patch, segment, slot, "b")
                        p.fence(buffers=True)
                    compute(
                        Opcode.CLEAR if segment == 0 else Opcode.SEED, patch, output_base=page * 256
                    )
                    acc(patch, segment, slot)
                    if page == len(group) - 1 and segment + 1 < segments:
                        load(group[0], segment + 1, 1 - slot)
                    p.fence(buffers=True)
                    compute(Opcode.STORE, patch, output_base=page * 256)
            p.fence()
            for page, patch in enumerate(group):
                export(patch, page)
            p.fence()
    if next_address > output_address or patches[-1]["output_address"] + 16384 > 1 << 32:
        raise ValueError("HBM images overlap or exceed the 4-GiB port window")
    return dict(
        schema_version=1,
        side=16,
        shape=[m, n, k],
        schedule=schedule,
        k_block=k_block,
        store_overlap=store_overlap,
        defer_output=defer_output,
        words=[f"{w:032x}" for w in p.words],
        final_fence_tag=p._tag,
        patches=patches,
        images=images,
        main_macs=4 * len(patches) * 128 * 128 * k,
        useful_macs=4 * m * n * k,
        physical_main_macs=16384,
        sram_bytes_per_tile=409600,
        output_format="fp32",
        boundary="one final drained stream; patches are not host/tag epochs",
    )
