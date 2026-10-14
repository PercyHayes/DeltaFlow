"""Conservative short-window programs on the existing unified ISA.

No new opcode or tensor payload in instructions. A window still executes C64
(on b16); the caller packs neutral tail tokens. Windows are fenced through O's
AXI B before accepting the next input, suitable for causally supplied decode.
"""


def emit_serial_windows(
    program, windows, *, dk=128, dv=128, tile_mask=15, initial_state_address, final_state_address
):
    windows = list(windows)
    if not windows or not 1 <= dk <= 8 * program.side or not 1 <= dv <= 8 * program.side:
        raise ValueError("nonempty windows and supported dimensions required")
    p = program
    t = 4 * p.side
    small = p.side == 4
    ports = 3 if small else 15
    p.dma(tile_mask=tile_mask, address=initial_state_address, region=1, local=512, beats=4 * dk)
    for index, (source, destination) in enumerate(windows):
        slot = index % 2
        p.dma(tile_mask=tile_mask, address=source, local=1536 * slot, beats=2 * dk)
        p.dma(
            tile_mask=tile_mask,
            address=source + 0x4000,
            local=256 + 1536 * slot,
            beats=dv,
            stride2=True,
        )
        p.dma(
            tile_mask=tile_mask,
            address=source + 0xC000,
            local=505 + 1536 * slot,
            beats=p.side // 4,
            stride2=True,
        )
        p.fence(tile_mask, dma=True)
        p.gdn_chunk(dk, dv, tile_mask, slot=slot)
        p.dma(
            store=True,
            tile_mask=tile_mask,
            address=destination,
            local=1152 + 256 * slot,
            beats=t if small else 2 * t,
            port_mask=ports,
            stride2=small,
        )
        p.fence(tile_mask, dma=True)
        p.gdn_release(slot, tile_mask)
        p.fence(tile_mask)
    p.dma(
        store=True,
        tile_mask=tile_mask,
        address=final_state_address,
        region=1,
        local=512,
        beats=2 * dk if small else 4 * dk,
        port_mask=ports,
        stride2=small,
    )
    return p.fence(tile_mask)


def emit_state_copy(program, source, destination, *, dk=128, tile_mask=15):
    """Restore/materialize a checkpoint through DMA without executing a token."""
    p = program
    small = p.side == 4
    p.fence(tile_mask)
    p.dma(tile_mask=tile_mask, address=source, region=1, local=512, beats=4 * dk)
    p.fence(tile_mask, dma=True)
    p.dma(
        store=True,
        tile_mask=tile_mask,
        address=destination,
        region=1,
        local=512,
        beats=2 * dk if small else 4 * dk,
        port_mask=3 if small else 15,
        stride2=small,
    )
    return p.fence(tile_mask)
