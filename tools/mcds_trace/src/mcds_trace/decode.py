"""Unit-level decoding of miniMCDS messages into timed events (TC3xx TS 9.3.9-9.3.14).

DTU_TC field order (TC3xx B-step), LSB first after core_ID and trace_type:
    DTWD  TT0/1: dsize[2] data
    DTA/DTWA/DTRA TT2/3: subtype[2]=0/1/3 addr
    DTRD  TT2/3: subtype[2]=2 dsize[2] data
    DTW   TT4/5: L1[4] dsize[2] data[len(L1)] addr
    DTR   TT6/7: L1[4] dsize[2] data[len(L1)] addr
Odd trace types are compressed: XOR with the reconstructed value of the same
compression engine (one address and one data engine per DTU core ID).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Iterator

from .tram import (CORE_DTU0, CORE_DTU1, CORE_NAMES, CORE_TSU, CORE_WTU,
                   LENGTH1_BITS, Bits, RawMessage, parse_paragraph)

DSIZE_BYTES = {0: 1, 1: 2, 2: 4, 3: 4}      # DOUBLE: lower 32 bits only
ADDR_BITS = 32

# A paragraph whose first absolute time stamp lies this many paragraph
# intervals past the end of the one before was written a lap later (the
# reader missed a whole lap); at a real boundary time is continuous.
LAP_FACTOR = 3
LAP_MIN_INTERVALS = 4          # paragraph intervals measured before judging


@dataclass
class Event:
    cycles: int              # emulation clock cycles (unwrapped); -1 if unknown
    kind: str                # 'write' | 'read' | 'wps' | 'wpm' | 'evc' | 'lost' | 'gap'
    core: str = ''
    addr: int | None = None
    value: int | None = None
    size: int = 0
    wp: int | None = None    # watch-point id / bitset / counter id
    count: int | None = None
    master: int | None = None
    paragraph: int = 0


@dataclass
class _DtuState:
    addr: int = 0
    data: int = 0
    addr_ok: bool = False    # base known: an uncompressed address was seen
    data_ok: bool = False


class _Unsynced(Exception):
    """A compressed field arrived before its engine had an uncompressed base."""


@dataclass
class DecodeStats:
    paragraphs: int = 0
    messages: int = 0
    unknown: int = 0
    errors: int = 0          # ERR messages
    lost_messages: int = 0   # sum of ERR err_cnt
    gaps: int = 0
    backsteps: int = 0       # time went backwards: a paragraph was torn by a lap
    unsynced: int = 0        # compressed messages dropped before a base was known
    torn: int = 0            # paragraphs dropped as inconsistent (overwritten while read)
    laps: int = 0            # paragraphs a whole lap newer than the one before
    by_type: dict = field(default_factory=dict)
    malformed: dict = field(default_factory=dict)   # 'core/TTn/bits' -> count


class Decoder:
    """Stateful decoder: feed paragraphs in order, get events."""

    def __init__(self, tick_mode: bool = False):
        self.tick_mode = tick_mode
        self.stats = DecodeStats()
        self.reset()
        self._prev_first = None    # first absolute TSR of the paragraph before
        self._interval = 0.0       # running mean of the paragraph interval
        self._intervals = 0
        self._first_abs = None     # this paragraph's first absolute TSR

    def reset(self) -> None:
        """Forget every compression base (after a gap).  The TSR history is
        kept for unwrapping, but time is unknown until an uncompressed TSR."""
        self.dtu = {CORE_DTU0: _DtuState(), CORE_DTU1: _DtuState()}
        self.wps = self.wpm = self.evc_id = self.evc_cnt = 0
        self.wps_ok = self.wpm_ok = self.evc_ok = False
        self.tsa = 0
        self.tsa_ok = False
        self.tsr_ok = False
        if not hasattr(self, 'tsr'):
            self.tsr = None        # last reconstructed TSR (32 bit)
            self.epoch = 0         # unwrap for the 32-bit counter
        self.base_cycles = -1      # unwrapped time of the last TSR
        self.ticks = 0             # ticks since the last TSR

    # -- time -----------------------------------------------------------------

    def _now(self) -> int:
        if self.base_cycles < 0:
            return -1
        return self.base_cycles + (self.ticks if self.tick_mode else 0)

    def _tsr(self, value: int) -> None:
        # A real wrap moves from the top of the 32-bit range to the bottom; a
        # smaller backward step means the paragraph before was torn (lapped).
        if self.tsr is not None and value < self.tsr:
            if self.tsr - value > 1 << 31:
                self.epoch += 1 << 32
            else:
                self.stats.backsteps += 1
                self._suspect = True
        self.tsr = value
        self.base_cycles = self.epoch + value
        self.ticks = 0

    # -- units ----------------------------------------------------------------

    def _count(self, name: str) -> None:
        self.stats.by_type[name] = self.stats.by_type.get(name, 0) + 1

    def _dtu(self, m: RawMessage, par: int) -> Iterator[Event]:
        st = self.dtu[m.core]
        comp = bool(m.tt & 1)
        b = Bits(m.data.to_bytes((m.dbits + 7) // 8 or 1, 'little'), 0, m.dbits)
        core = CORE_NAMES[m.core]

        def data_field(nbits: int) -> tuple[int, int]:
            dsize = b.take(2)
            raw = b.take(min(nbits, b.left()))
            size = DSIZE_BYTES[dsize]
            mask = (1 << (8 * size)) - 1
            if comp and not st.data_ok:
                raise _Unsynced
            val = ((raw ^ st.data) if comp else raw) & mask
            st.data, st.data_ok = val, True
            return val, size

        def addr_field() -> tuple[int, int | None]:
            raw = b.take(b.left())
            if comp and not st.addr_ok:
                raise _Unsynced
            full = (raw ^ st.addr) if comp else raw
            st.addr, st.addr_ok = full, True
            prefix = full >> ADDR_BITS
            return full & 0xFFFFFFFF, (prefix >> 1) & 0xF if prefix else None

        tt = m.tt
        if tt in (0, 1):                                   # DTWD
            self._count('DTWD')
            val, size = data_field(b.left() - 2)
            yield Event(self._now(), 'write', core, None, val, size, paragraph=par)
        elif tt in (2, 3):
            sub = b.take(2)
            if sub == 2:                                   # DTRD
                self._count('DTRD')
                val, size = data_field(b.left() - 2)
                yield Event(self._now(), 'read', core, None, val, size, paragraph=par)
            else:                                          # DTA / DTWA / DTRA
                name = {0: 'DTA', 1: 'DTWA', 3: 'DTRA'}[sub]
                self._count(name)
                addr, master = addr_field()
                if sub == 0:
                    self._pending_dta = (addr, master)     # the DTWD/DTRD follows
                else:
                    yield Event(self._now(), 'write' if sub == 1 else 'read', core,
                                addr, None, 0, master=master, paragraph=par)
        else:                                              # DTW (4/5), DTR (6/7)
            self._count('DTW' if tt < 6 else 'DTR')
            n1 = LENGTH1_BITS.get(b.take(4), 0)
            val, size = data_field(n1)
            addr, master = addr_field()
            yield Event(self._now(), 'write' if tt < 6 else 'read', core, addr, val, size,
                        master=master, paragraph=par)

    def _wtu(self, m: RawMessage, par: int) -> Iterator[Event]:
        comp = bool(m.tt & 1)
        b = Bits(m.data.to_bytes((m.dbits + 7) // 8 or 1, 'little'), 0, m.dbits)
        if m.tt in (0, 1):
            self._count('WPS')
            raw = b.take(b.left())
            if comp and not self.wps_ok:
                raise _Unsynced
            self.wps, self.wps_ok = (raw ^ self.wps) if comp else raw, True
            yield Event(self._now(), 'wps', 'WTU', wp=self.wps & 0x7, paragraph=par)
        elif m.tt in (2, 3):
            self._count('WPM')
            raw = b.take(b.left())
            if comp and not self.wpm_ok:
                raise _Unsynced
            self.wpm, self.wpm_ok = (raw ^ self.wpm) if comp else raw, True
            yield Event(self._now(), 'wpm', 'WTU', wp=self.wpm & 0xFF, paragraph=par)
        elif m.tt in (4, 5):
            self._count('EVC')
            n1 = LENGTH1_BITS.get(b.take(4), 0)
            cid = b.take(min(n1, b.left()))
            cnt = b.take(b.left())
            if comp:
                if not self.evc_ok:
                    raise _Unsynced
                cid ^= self.evc_id
                cnt ^= self.evc_cnt
            self.evc_id, self.evc_cnt, self.evc_ok = cid, cnt, True
            yield Event(self._now(), 'evc', 'WTU', wp=cid, count=cnt, paragraph=par)
        else:
            self.stats.unknown += 1

    def _tsu(self, m: RawMessage) -> None:
        comp = bool(m.tt & 1)
        if m.tt in (0, 1):
            self._count('TSR')
            if comp and not self.tsr_ok:
                raise _Unsynced
            value = ((m.data ^ self.tsr) if comp else m.data) & 0xFFFFFFFF
            if comp and value < self.tsr and self.tsr - value < 1 << 31:
                # The XOR chain broke (a TSR was dropped under overload):
                # time is unknown until the next uncompressed TSR.
                self.stats.backsteps += 1
                self._suspect = True
                self.tsr_ok = False
                self.base_cycles = -1
                return
            self._tsr(value)
            self.tsr_ok = True
            if not comp and self._first_abs is None:
                self._first_abs = self.base_cycles
        elif m.tt in (2, 3):
            self._count('TSA')
            if comp and not self.tsa_ok:
                raise _Unsynced
            self.tsa = ((m.data ^ self.tsa) if comp else m.data) & 0xFFFFFFFF
            self.tsa_ok = True
        else:
            self.stats.unknown += 1

    # -- paragraphs -----------------------------------------------------------

    def paragraph(self, par_bytes: bytes, index: int = 0, gap: bool = False,
                  final: bool = True) -> Iterator[Event]:
        """Decode one 1 kB paragraph.  gap=True: data was lost before it.

        A paragraph the writer overwrote while it was read is a mix of new and
        old messages: time runs backwards inside it, or the message stream
        breaks off (an invalid length code, unknown units, fields running past
        the paragraph).  Such a paragraph is dropped as a whole, the decoder
        state rewound to before it, and a gap reported instead."""
        if not gap and self._lapped(par_bytes):
            # Its leading compressed messages refer to a paragraph never read.
            self.stats.laps += 1
            gap = True
        if gap:
            self.stats.gaps += 1
            self.reset()
            yield Event(-1, 'gap', paragraph=index)
        self.stats.paragraphs += 1
        saved = (self.tsr, self.epoch, self.base_cycles, self.ticks, self.tsr_ok)
        counts = (self.stats.unknown, sum(self.stats.malformed.values()))
        self._suspect = False
        self._first_abs = None
        events = list(self._paragraph_events(par_bytes, index, final))
        if (self._suspect or self.stats.unknown != counts[0]
                or sum(self.stats.malformed.values()) != counts[1]):
            self.stats.torn += 1
            self.reset()
            self.tsr, self.epoch, self.base_cycles, self.ticks, self.tsr_ok = saved
            self.tsr_ok = False
            self.base_cycles = -1
            self._prev_first = None
            yield Event(-1, 'gap', paragraph=index)
            return
        first = self._first_abs
        if first is not None and self._prev_first is not None and not gap                 and first > self._prev_first:
            n = min(self._intervals, 15)
            self._interval = (self._interval * n + (first - self._prev_first)) / (n + 1)
            self._intervals += 1
        if first is not None:
            self._prev_first = first
        yield from events

    def _lapped(self, par_bytes: bytes) -> bool:
        """True if this paragraph starts a lap after the one decoded before."""
        if not self.tsr_ok or self.tsr is None or self._intervals < LAP_MIN_INTERVALS:
            return False
        for k, m in enumerate(parse_paragraph(par_bytes)):
            if m.kind == 'msg' and m.core == CORE_TSU and m.tt == 0:
                jump = ((m.data & 0xFFFFFFFF) - self.tsr) & 0xFFFFFFFF
                return LAP_FACTOR * self._interval < jump < 1 << 31
            if k >= 16:
                break
        return False

    def _paragraph_events(self, par_bytes: bytes, index: int, final: bool) -> Iterator[Event]:
        self._pending_dta = None
        for m in parse_paragraph(par_bytes):
            if m.kind == 'damaged':
                self._suspect = True
                return
            if m.kind == 'end' and not final:
                # Only the last written paragraph ends early: this one was
                # still being written (the reader fell a lap behind).
                self._suspect = True
                return
            if m.kind == 'tick':
                self.ticks += 1
                continue
            if m.kind == 'multick':
                self.ticks += m.ticks
                continue
            if m.kind == 'error':
                self.stats.errors += 1
                self.stats.lost_messages += m.err_cnt
                yield Event(self._now(), 'lost', CORE_NAMES.get(m.core, '?'),
                            count=m.err_cnt, paragraph=index)
                continue
            if m.kind != 'msg':
                continue
            self.stats.messages += 1
            try:
                if m.core in (CORE_DTU0, CORE_DTU1):
                    for ev in list(self._dtu(m, index)):
                        if self._pending_dta and ev.addr is None and ev.value is not None:
                            ev.addr, ev.master = self._pending_dta
                            self._pending_dta = None
                        yield ev
                elif m.core == CORE_WTU:
                    yield from list(self._wtu(m, index))
                elif m.core == CORE_TSU:
                    self._tsu(m)
                else:
                    self.stats.unknown += 1
            except _Unsynced:
                self.stats.unsynced += 1
            except EOFError:
                key = '%s/TT%d/%db' % (CORE_NAMES.get(m.core, m.core), m.tt, m.dbits)
                self.stats.malformed[key] = self.stats.malformed.get(key, 0) + 1


def decode_records(records: Iterable, tick_mode: bool = False) -> tuple[list[Event], DecodeStats]:
    """Decode capfile.Record objects (one paragraph each) in order."""
    dec = Decoder(tick_mode=tick_mode)
    events: list[Event] = []
    prev = None
    records = list(records)
    legacy = not any(r.final for r in records)     # files from before the FINAL flag
    for k, rec in enumerate(records):
        gap = rec.gap or (prev is not None and rec.seq != prev + 1)
        prev = rec.seq
        final = rec.final or (legacy and k == len(records) - 1)
        events.extend(dec.paragraph(rec.payload, rec.seq, gap=gap, final=final))
    return events, dec.stats
