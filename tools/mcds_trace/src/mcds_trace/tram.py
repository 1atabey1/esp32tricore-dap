"""Generic miniMCDS trace memory format (TC3xx TS, Tables 337-341).

The buffer is a bit stream, LSB first: bit i is (byte[i // 8] >> (i % 8)) & 1.
Messages are <length> <body> with the length in the lowest bits; every
1 kB paragraph starts on a length field (the previous one is padded with a
<skip> message).  The body is <core_ID:5> <trace_type:3> <trace_data>,
again from the low bits up.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

PARAGRAPH_BYTES = 1024

# Table 338: <symlength> code -> body length in bits.
SYMLEN_BITS = {0x2: 12, 0x3: 16, 0x4: 20, 0x5: 24, 0x6: 32, 0x7: 40,
               0x8: 44, 0x9: 56, 0xA: 64, 0xB: 76, 0xC: 92, 0xD: 112}

# Table 341: <length 1> code -> bits of <data 1> inside TT4..TT7.
LENGTH1_BITS = {0x0: 0, 0x1: 4, 0x2: 8, 0x3: 12, 0x4: 16, 0x5: 20, 0x6: 24,
                0x7: 28, 0x8: 32, 0x9: 36, 0xA: 40, 0xB: 44, 0xC: 64}

CODE_TICK = 0x0      # <tick>: empty message, one time tag step
CODE_ERROR = 0x1     # <errormessage>: <error_cnt:7> <core_ID:5> 0001
CODE_MULTICK = 0xE   # <multick> <ticount:8>
CODE_ABSIZE = 0xF    # <bicount:8> <absize>; bicount 0xFF = <endoftrace>

# Table 339: core IDs on TC3xx.
CORE_SKIP = 0x00
CORE_DCU = 0x08
CORE_DTU0 = 0x09
CORE_DTU1 = 0x0B
CORE_PTU = 0x0C
CORE_TSU = 0x0E
CORE_WTU = 0x0F

CORE_NAMES = {CORE_DCU: 'DCU', CORE_DTU0: 'DTU0', CORE_DTU1: 'DTU1',
              CORE_PTU: 'PTU', CORE_TSU: 'TSU', CORE_WTU: 'WTU'}


class Bits:
    """LSB-first bit reader over a bytes object."""

    def __init__(self, data: bytes, start_bit: int = 0, end_bit: int | None = None):
        self.data = data
        # The whole buffer as one little-endian integer: bit k of the stream
        # is bit k of the integer, so a field is one shift and mask.
        self._int = int.from_bytes(data, 'little')
        self.pos = start_bit
        self.end = len(data) * 8 if end_bit is None else end_bit

    def left(self) -> int:
        return self.end - self.pos

    def take(self, n: int) -> int:
        if n > self.end - self.pos:
            raise EOFError('need %d bits, %d left' % (n, self.end - self.pos))
        value = (self._int >> self.pos) & ((1 << n) - 1)
        self.pos += n
        return value


@dataclass
class RawMessage:
    """One message of the trace memory, before unit-specific decoding.

    kind: 'msg' (a unit message), 'tick', 'multick', 'error', 'skip', 'end'.
    For 'msg': core, tt and data (trace_data as an integer, dbits wide).
    """
    kind: str
    bit: int                 # position of the length field in the paragraph
    core: int = 0
    tt: int = 0
    data: int = 0
    dbits: int = 0
    ticks: int = 0           # tick / multick count
    err_cnt: int = 0


def parse_paragraph(par: bytes) -> Iterator[RawMessage]:
    """Split one paragraph (normally 1 kB) into raw messages."""
    bits = Bits(par)
    while bits.left() >= 4:
        at = bits.pos
        code = bits.take(4)
        if code == CODE_TICK:
            yield RawMessage('tick', at, ticks=1)
            continue
        if code == CODE_MULTICK:
            if bits.left() < 8:
                return
            count = bits.take(8)
            if count:                      # 0 is an alignment filler
                yield RawMessage('multick', at, ticks=count)
            continue
        if code == CODE_ERROR:
            if bits.left() < 12:
                return
            core = bits.take(5)
            yield RawMessage('error', at, core=core, err_cnt=bits.take(7))
            continue
        if code == CODE_ABSIZE:
            if bits.left() < 8:
                return
            length = bits.take(8)
            if length == 0xFF:
                yield RawMessage('end', at)
                return
        elif code in SYMLEN_BITS:
            length = SYMLEN_BITS[code]
        else:
            # Not a valid length code.  The writer ends a paragraph with
            # <endoftrace> or <skip>, so this is damage (e.g. overwritten
            # while it was read), not the end.
            yield RawMessage('damaged', at)
            return
        if length > bits.left():
            if bits.left() > 32:
                yield RawMessage('damaged', at)
            return                         # truncated at the paragraph end
        if length < 8:
            bits.take(length)
            continue
        core = bits.take(5)
        if core == CORE_SKIP:
            # <skip> pads the rest of the paragraph.
            yield RawMessage('skip', at, dbits=length)
            return
        tt = bits.take(3)
        dbits = length - 8
        yield RawMessage('msg', at, core=core, tt=tt, data=bits.take(dbits), dbits=dbits)


def split_tt(msg: RawMessage, subtype_bits: int = 0) -> dict:
    """Split <trace_data> by the standard trace type (Table 340).

    Returns a dict with 'subtype', 'data1', 'data1_bits', 'data2', 'data2_bits',
    'compressed'.  TT0/1: data1 only.  TT2/3: subtype + data1.  TT4/5:
    <length 1> + data1 + data2.  TT6/7: subtype + <length 1> + data1 + data2.
    Fields sit from the low bits up: subtype next to the trace type.
    """
    bits = Bits(msg.data.to_bytes((msg.dbits + 7) // 8 or 1, 'little'), 0, msg.dbits)
    out = {'subtype': 0, 'data1': 0, 'data1_bits': 0, 'data2': 0, 'data2_bits': 0,
           'compressed': bool(msg.tt & 1)}
    tt = msg.tt
    if tt in (2, 3, 6, 7) and subtype_bits:
        out['subtype'] = bits.take(subtype_bits)
    if tt in (4, 5, 6, 7):
        code = bits.take(4)
        n1 = LENGTH1_BITS.get(code, 0)
        n1 = min(n1, bits.left())
        out['data1_bits'] = n1
        out['data1'] = bits.take(n1)
        out['data2_bits'] = bits.left()
        out['data2'] = bits.take(bits.left())
    else:
        out['data1_bits'] = bits.left()
        out['data1'] = bits.take(bits.left())
    return out


def paragraphs(buf: bytes, size: int = PARAGRAPH_BYTES) -> Iterator[bytes]:
    for i in range(0, len(buf) - size + 1, size):
        yield buf[i:i + size]
