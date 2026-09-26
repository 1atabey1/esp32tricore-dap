"""Unit decoding against messages built from the spec's field layouts."""

from mcds_trace.decode import Decoder
from mcds_trace.tram import CORE_DTU0, CORE_TSU, CORE_WTU, SYMLEN_BITS


class Packer:
    """LSB-first bit writer producing one paragraph."""

    def __init__(self):
        self.bits = []

    def put(self, value, n):
        self.bits += [(value >> i) & 1 for i in range(n)]

    def msg(self, core, tt, fields):
        """fields: [(value, nbits), ...] from the low bits up."""
        body = [(core, 5), (tt, 3)] + fields
        need = sum(n for _, n in body)
        code, length = min(((c, l) for c, l in SYMLEN_BITS.items() if l >= need),
                           key=lambda cl: cl[1])
        self.put(code, 4)
        for v, n in body:
            self.put(v, n)
        self.put(0, length - need)            # leading zeros of the last field

    def paragraph(self):
        bits = self.bits + [1] * 12             # <endoftrace>
        bits += [0] * (-len(bits) % 8)
        return bytes(sum(bits[i + k] << k for k in range(8)) for i in range(0, len(bits), 8))


def test_dtw_compressed_and_tsr_time():
    p = Packer()
    p.msg(CORE_TSU, 0, [(1000, 32)])                                   # TSR 1000
    # DTW uncompressed: L1 = 8 (code 2), dsize byte, data 0x5A, addr
    p.msg(CORE_DTU0, 4, [(0x2, 4), (0, 2), (0x5A, 8), (0x70001234, 32)])
    p.msg(CORE_TSU, 1, [(1000 ^ 1010, 12)])                            # TSR 1010 (compressed)
    # DTW compressed: data 0x5A->0x5B (xor 1), address unchanged (xor 0, no bits)
    p.msg(CORE_DTU0, 5, [(0x1, 4), (0, 2), (0x1, 4)])
    ev = list(Decoder().paragraph(p.paragraph()))
    assert [(e.kind, e.addr, e.value, e.cycles) for e in ev] == [
        ('write', 0x70001234, 0x5A, 1000),
        ('write', 0x70001234, 0x5B, 1010),
    ]


def test_dtwd_word_then_dta_split():
    p = Packer()
    p.msg(CORE_DTU0, 0, [(2, 2), (0xDEADBEEF, 32)])                    # DTWD word
    p.msg(CORE_DTU0, 2, [(0, 2), (0x7000A000, 32)])                    # DTA
    p.msg(CORE_DTU0, 1, [(2, 2), (0xDEADBEEF ^ 0x12345678, 32)])       # DTWD compressed
    ev = list(Decoder().paragraph(p.paragraph()))
    assert ev[0].value == 0xDEADBEEF and ev[0].addr is None
    assert (ev[1].addr, ev[1].value, ev[1].size) == (0x7000A000, 0x12345678, 4)


def _tsr_par(*values):
    p = Packer()
    for v in values:
        p.msg(CORE_TSU, 0, [(v, 32)])
        p.msg(CORE_DTU0, 4, [(0x2, 4), (0, 2), (v & 0xFF, 8), (0x70001234, 32)])
    return p.paragraph()


def test_torn_paragraphs_are_dropped():
    # Time running backwards inside a paragraph: overwritten while read.
    dec = Decoder()
    ev = list(dec.paragraph(_tsr_par(1000, 2000)))
    assert [e.kind for e in ev] == ['write', 'write']
    ev = list(dec.paragraph(_tsr_par(3000, 2500), 1))
    assert [e.kind for e in ev] == ['gap'] and dec.stats.torn == 1
    # The rewound state carries on: the next good paragraph decodes.
    ev = list(dec.paragraph(_tsr_par(4000), 2))
    assert [(e.kind, e.cycles) for e in ev] == [('write', 4000)]


def test_endoftrace_only_in_the_final_paragraph():
    dec = Decoder()
    # Every Packer paragraph ends with <endoftrace>: legit only when final.
    assert [e.kind for e in dec.paragraph(_tsr_par(10), 0, final=False)] == ['gap']
    assert [e.kind for e in dec.paragraph(_tsr_par(20), 1, final=True)] == ['write']


def test_lapped_paragraph_is_a_gap():
    # Steady paragraphs 1000 cycles apart, then one a whole lap (8) later
    # whose first, compressed message refers to a paragraph never read.
    dec = Decoder()
    for k in range(6):
        ev = list(dec.paragraph(_tsr_par(1000 * k + 10, 1000 * k + 500), k))
        assert [e.kind for e in ev] == ['write', 'write']
    p = Packer()
    p.msg(CORE_DTU0, 5, [(0x1, 4), (0, 2), (0x7, 4)])                 # stale base
    p.msg(CORE_TSU, 0, [(13010, 32)])
    p.msg(CORE_DTU0, 4, [(0x2, 4), (0, 2), (0x42, 8), (0x70001234, 32)])
    ev = list(dec.paragraph(p.paragraph(), 6))
    assert [e.kind for e in ev] == ['gap', 'write'] and ev[1].value == 0x42
    assert dec.stats.laps == 1 and dec.stats.unsynced == 1
    # A continuous boundary is not a lap.
    ev = list(dec.paragraph(_tsr_par(13600), 7))
    assert [e.kind for e in ev] == ['write'] and dec.stats.laps == 1


def test_wps_compression():
    p = Packer()
    p.msg(CORE_WTU, 0, [(1, 4)])        # WPS id 1
    p.msg(CORE_WTU, 1, [(1, 4)])        # compressed: 1 ^ 1 = 0
    p.msg(CORE_WTU, 1, [(1, 4)])        # back to 1
    ev = list(Decoder().paragraph(p.paragraph()))
    assert [e.wp for e in ev] == [1, 0, 1]
