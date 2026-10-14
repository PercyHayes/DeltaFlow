"""Host transport for the existing Program ISA; register ABI, no tensor payloads."""

BASE = 0x8000
STAGING = tuple(BASE + 4 * i for i in range(4))
SUBMIT = BASE + 0x10
STATUS = BASE + 0x14
SUBMITTED = BASE + 0x18
COMPLETION = BASE + 0x20


def writes(word):
    if not isinstance(word, int) or not 0 <= word < 1 << 128:
        raise ValueError("one 128-bit instruction required")
    return [(address, (word >> (32 * i)) & 0xFFFFFFFF) for i, address in enumerate(STAGING)] + [
        (SUBMIT, 1)
    ]


def completion(word):
    if not word >> 31:
        return None
    return dict(
        tag=word & 65535,
        opcode=(word >> 16) & 255,
        tile=(word >> 24) & 3,
        port=(word >> 26) & 3,
        dma=bool(word & (1 << 28)),
        error=bool(word & (1 << 29)),
    )


class Transport:
    """32-bit register transactions supplied by a host/board implementation.

    Poll submit-ready before committing and drain completions while waiting.
    There must be only one instruction producer: the ready observation remains
    valid because only this producer can consume the available instruction slot.
    FBUS ordering may block completion reads behind an unchecked commit write.
    """

    def __init__(self, write32, read32):
        self.write32 = write32
        self.read32 = read32
        self.events = []

    def submit(self, word):
        sequence = writes(word)
        for address, value in sequence[:-1]:
            self.write32(address, value)
        while True:
            status = self.read32(STATUS)
            if status & ((15 << 7) | (1 << 11)):
                raise RuntimeError(f"DeltaFlow fault/status 0x{status:08x}")
            if status & 2:
                break
            event = self.poll()
            if event is not None:
                self.events.append(event)
        self.write32(*sequence[-1])

    def poll(self):
        event = completion(self.read32(COMPLETION))
        if event is not None and event["error"]:
            raise RuntimeError(f"DeltaFlow completion error, tag {event['tag']}")
        return event

    def execute(self, words):
        """Submit a drained tag epoch and collect every selected engine reply.

        The caller supplies the board's ordered read32/write32 functions. Tensor
        upload/download is an external HBM operation; this transport only moves
        instruction metadata and completion records. Tags may be reused in a
        later execute call after this call has fully drained.
        """
        words = list(words)
        pending = set()
        collected = []
        if self.events:
            raise RuntimeError("consume earlier completion events before a new epoch")
        for word in words:
            writes(word)  # Width validation before touching the bus.
            op = word & 255
            tag = word >> 16 & 65535
            tiles = word >> 8 & 15
            ports = word >> 12 & 15
            if op in (18, 19, 20):
                keys = [(op, tag, 0, 0, False)]
            elif op in (16, 17):
                keys = [
                    (op, tag, t, p, True)
                    for t in range(4)
                    if tiles >> t & 1
                    for p in range(4)
                    if ports >> p & 1
                ]
            elif op in (0, 1, 2, 3, 4, 33, 34, 35, 36, 37):
                keys = [(op, tag, t, 0, False) for t in range(4) if tiles >> t & 1]
            else:
                raise ValueError("unknown opcode")
            if not tiles or not keys or any(key in pending for key in keys):
                raise ValueError("empty mask or repeated completion identity")
            pending.update(keys)

        def accept(event):
            key = tuple(event[k] for k in ("opcode", "tag", "tile", "port", "dma"))
            if key not in pending:
                raise RuntimeError("unexpected or duplicate completion")
            pending.remove(key)
            collected.append(event)

        for word in words:
            self.submit(word)
            for event in self.events:
                accept(event)
            self.events.clear()
        while pending:
            event = self.poll()
            if event is not None:
                accept(event)
            else:
                status = self.read32(STATUS)
                if status & ((15 << 7) | (1 << 11)):
                    raise RuntimeError(f"DeltaFlow fault/status 0x{status:08x}")
        return collected
