"""Unit-level decoding of miniMCDS messages into timed events (TC3xx TS 9.3.9-9.3.14).

DTU_TC field order (TC3xx B-step), LSB first after core_ID and trace_type:
    DTWD  TT0/1: dsize[2] data
    DTA/DTWA/DTRA TT2/3: subtype[2]=0/1/3 addr
    DTRD  TT2/3: subtype[2]=2 dsize[2] data
    DTW   TT4/5: L1[4] dsize[2] data[len(L1)] addr
    DTR   TT6/7: L1[4] dsize[2] data[len(L1)] addr
Odd trace types are compressed: XOR with the reconstructed value of the same
compression engine (one address and one data engine per DTU core ID).

The message framing is tram.parse_paragraph's; the decoder runs it inline, in
one pass over the paragraph, because a live trace delivers several hundred
thousand messages a second.  Events come out as rows (see ROW_FIELDS) and, for
callers that want objects, as Event.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Iterator

from .tram import (CODE_ABSIZE, CODE_ERROR, CODE_MULTICK, CODE_TICK, CORE_DTU0, CORE_DTU1,
                   CORE_NAMES, CORE_SKIP, CORE_TSU, CORE_WTU, LENGTH1_BITS, SYMLEN_BITS)

DSIZE_BYTES = {0: 1, 1: 2, 2: 4, 3: 4}      # DOUBLE: lower 32 bits only
ADDR_BITS = 32

# A paragraph whose first absolute time stamp lies this many paragraph
# intervals past the end of the one before was written a lap later (the
# reader missed a whole lap); at a real boundary time is continuous.
LAP_FACTOR = 3
LAP_MIN_INTERVALS = 4          # paragraph intervals measured before judging

# Event rows: (cycles, kind, addr, value, size, wp, count, core, master), with
# -1 for "none" in the integer fields and kind an index into KINDS.
KINDS = ('write', 'read', 'wps', 'wpm', 'evc', 'lost', 'gap')
KIND_CODE = {k: i for i, k in enumerate(KINDS)}
K_WRITE, K_READ, K_WPS, K_WPM, K_EVC, K_LOST, K_GAP = range(7)
ROW_FIELDS = ('cycles', 'kind', 'addr', 'value', 'size', 'wp', 'count', 'core', 'master')
GAP_ROW = (-1, K_GAP, -1, -1, 0, -1, -1, '', -1)

_SYMLEN = [SYMLEN_BITS.get(c, 0) for c in range(16)]
_LEN1 = [LENGTH1_BITS.get(c, 0) for c in range(16)]
_DSIZE = [1, 2, 4, 4]
_DMASK = [0xFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
_W128 = (1 << 128) - 1
_TYPES = ('DTWD', 'DTRD', 'DTA', 'DTWA', 'DTRA', 'DTW', 'DTR', 'WPS', 'WPM', 'EVC', 'TSR', 'TSA')
(_T_DTWD, _T_DTRD, _T_DTA, _T_DTWA, _T_DTRA, _T_DTW, _T_DTR, _T_WPS, _T_WPM, _T_EVC,
 _T_TSR, _T_TSA) = range(len(_TYPES))
_SUB_TYPE = {0: _T_DTA, 1: _T_DTWA, 3: _T_DTRA}


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


def event_of(row: tuple, paragraph: int = 0) -> Event:
    c, k, a, v, s, w, n, core, m = row
    return Event(c, KINDS[k], core, None if a < 0 else a, None if v < 0 else v, s,
                 None if w < 0 else w, None if n < 0 else n, None if m < 0 else m, paragraph)


def row_of(ev: Event) -> tuple:
    return (ev.cycles, KIND_CODE.get(ev.kind, K_GAP), -1 if ev.addr is None else ev.addr,
            -1 if ev.value is None else ev.value, ev.size or 0, -1 if ev.wp is None else ev.wp,
            -1 if ev.count is None else ev.count, ev.core,
            -1 if ev.master is None else ev.master)


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
        self.tsr = None            # last reconstructed TSR (32 bit)
        self.epoch = 0             # unwrap for the 32-bit counter
        self.reset()
        self._prev_first = None    # first absolute TSR of the paragraph before
        self._interval = 0.0       # running mean of the paragraph interval
        self._intervals = 0
        self._types = [0] * len(_TYPES)

    def reset(self) -> None:
        """Forget every compression base (after a gap).  The TSR history is
        kept for unwrapping, but time is unknown until an uncompressed TSR."""
        # Per DTU core ID: [addr, data, addr_ok, data_ok].
        self.dtu = {CORE_DTU0: [0, 0, False, False], CORE_DTU1: [0, 0, False, False]}
        self.wps = self.wpm = self.evc_id = self.evc_cnt = 0
        self.wps_ok = self.wpm_ok = self.evc_ok = False
        self.tsa = 0
        self.tsa_ok = False
        self.tsr_ok = False
        self.base_cycles = -1      # unwrapped time of the last TSR
        self.ticks = 0             # ticks since the last TSR

    # -- paragraphs -----------------------------------------------------------

    def paragraph(self, par_bytes: bytes, index: int = 0, gap: bool = False,
                  final: bool = True) -> Iterator[Event]:
        """Decode one 1 kB paragraph into Events (see rows)."""
        for row in self.rows(par_bytes, index, gap, final):
            yield event_of(row, index)

    def rows(self, par_bytes: bytes, index: int = 0, gap: bool = False,
             final: bool = True) -> list[tuple]:
        """Decode one 1 kB paragraph into event rows.  gap=True: data was lost
        before it.

        A paragraph the writer overwrote while it was read is a mix of new and
        old messages: time runs backwards inside it, or the message stream
        breaks off (an invalid length code, unknown units, fields running past
        the paragraph).  Such a paragraph is dropped as a whole, the decoder
        state rewound to before it, and a gap reported instead.  A paragraph
        written a whole lap after the one before starts a gap too."""
        st = self.stats
        big = int.from_bytes(par_bytes, 'little')
        nbits = len(par_bytes) * 8
        out: list[tuple] = []
        if not gap and self._lapped(big, nbits):
            # Its leading compressed messages refer to a paragraph never read.
            st.laps += 1
            gap = True
        if gap:
            st.gaps += 1
            self.reset()
            out.append(GAP_ROW)
        st.paragraphs += 1
        saved = (self.tsr, self.epoch, self.base_cycles, self.ticks, self.tsr_ok)
        counts = (st.unknown, sum(st.malformed.values()))
        mark = len(out)
        suspect, first = self._scan(big, nbits, final, out)
        if suspect or st.unknown != counts[0] or sum(st.malformed.values()) != counts[1]:
            st.torn += 1
            del out[mark:]
            self.reset()
            self.tsr, self.epoch, self.base_cycles, self.ticks, self.tsr_ok = saved
            self.tsr_ok = False
            self.base_cycles = -1
            self._prev_first = None
            out.append(GAP_ROW)
            return out
        if first is not None and self._prev_first is not None and not gap \
                and first > self._prev_first:
            n = min(self._intervals, 15)
            self._interval = (self._interval * n + (first - self._prev_first)) / (n + 1)
            self._intervals += 1
        if first is not None:
            self._prev_first = first
        return out

    def _lapped(self, big: int, nbits: int) -> bool:
        """True if this paragraph starts a lap after the one decoded before."""
        if not self.tsr_ok or self.tsr is None or self._intervals < LAP_MIN_INTERVALS:
            return False
        # The first 17 messages, counted as tram.parse_paragraph yields them.
        pos = k = 0
        while nbits - pos >= 4:
            w = (big >> pos) & _W128
            code = w & 0xF
            if code == CODE_TICK:
                pos += 4
            elif code == CODE_MULTICK:
                if nbits - pos < 12:
                    break
                pos += 12
                if not (w >> 4) & 0xFF:
                    continue                    # a filler, not a message
            elif code == CODE_ERROR:
                if nbits - pos < 16:
                    break
                pos += 16
            else:
                if code == CODE_ABSIZE:
                    if nbits - pos < 12:
                        break
                    length = (w >> 4) & 0xFF
                    if length == 0xFF:
                        break
                    hdr = 12
                else:
                    length = _SYMLEN[code]
                    hdr = 4
                if length > nbits - pos - hdr:
                    break
                body = (big >> (pos + hdr)) & ((1 << length) - 1)
                pos += hdr + length
                if length < 8:
                    continue                    # not yielded either
                core = body & 0x1F
                if core == CORE_SKIP:
                    break
                if core == CORE_TSU and (body >> 5) & 7 == 0:
                    jump = ((body >> 8) - self.tsr) & 0xFFFFFFFF
                    return LAP_FACTOR * self._interval < jump < 1 << 31
            if k >= 16:
                break
            k += 1
        return False

    def _scan(self, big: int, end: int, final: bool, out: list) -> tuple[bool, int | None]:
        """Decode the messages of one paragraph into `out`; returns (suspect,
        the first absolute TSR's time or None).  Framing as tram.parse_paragraph."""
        st = self.stats
        types = self._types
        append = out.append
        tick_mode = self.tick_mode
        dtu = self.dtu
        tsr, epoch, base, ticks, tsr_ok = (self.tsr, self.epoch, self.base_cycles, self.ticks,
                                           self.tsr_ok)
        tsa, tsa_ok = self.tsa, self.tsa_ok
        wps, wps_ok, wpm, wpm_ok = self.wps, self.wps_ok, self.wpm, self.wpm_ok
        evc_id, evc_cnt, evc_ok = self.evc_id, self.evc_cnt, self.evc_ok
        pending = None                  # a DTA's (addr, master), for the DTWD/DTRD after it
        suspect = False
        first = None
        msgs = unknown = unsynced = backsteps = errors = lost = 0
        malformed = None
        pos = 0
        while end - pos >= 4:
            w = (big >> pos) & _W128
            code = w & 0xF
            if code == CODE_TICK:
                ticks += 1
                pos += 4
                continue
            if code == CODE_MULTICK:
                if end - pos < 12:
                    break
                ticks += (w >> 4) & 0xFF        # 0 is an alignment filler
                pos += 12
                continue
            if code == CODE_ERROR:
                if end - pos < 16:
                    break
                ec = (w >> 9) & 0x7F
                errors += 1
                lost += ec
                now = -1 if base < 0 else (base + ticks if tick_mode else base)
                append((now, K_LOST, -1, -1, 0, -1, ec, CORE_NAMES.get((w >> 4) & 0x1F, '?'), -1))
                pos += 16
                continue
            if code == CODE_ABSIZE:
                if end - pos < 12:
                    break
                length = (w >> 4) & 0xFF
                if length == 0xFF:              # <endoftrace>
                    # Only the last written paragraph ends early: this one was
                    # still being written (the reader fell a lap behind).
                    suspect = suspect or not final
                    break
                hdr = 12
            else:
                length = _SYMLEN[code]
                if not length:
                    # Not a valid length code: damage (overwritten while read).
                    suspect = True
                    break
                hdr = 4
            left = end - pos - hdr
            if length > left:
                suspect = suspect or left > 32  # else truncated at the paragraph end
                break
            if length < 8:
                pos += hdr + length
                continue
            if hdr == 4:
                body = (w >> 4) & ((1 << length) - 1)
            else:
                body = (big >> (pos + 12)) & ((1 << length) - 1)
            pos += hdr + length
            core = body & 0x1F
            if core == CORE_SKIP:               # <skip> pads the rest
                break
            tt = (body >> 5) & 7
            data = body >> 8
            dbits = length - 8
            msgs += 1
            comp = tt & 1

            if core == CORE_DTU0 or core == CORE_DTU1:
                s = dtu[core]
                if tt <= 1:                                         # DTWD
                    types[_T_DTWD] += 1
                    if dbits < 2:
                        malformed = _bad(malformed, core, tt, dbits)
                        continue
                    if comp and not s[3]:
                        unsynced += 1
                        continue
                    dsize = data & 3
                    val = ((data >> 2) ^ s[1] if comp else data >> 2) & _DMASK[dsize]
                    s[1] = val
                    s[3] = True
                    if pending is not None:
                        a, m = pending
                        pending = None
                    else:
                        a = m = -1
                    now = -1 if base < 0 else (base + ticks if tick_mode else base)
                    append((now, K_WRITE, a, val, _DSIZE[dsize], -1, -1, CORE_NAMES[core], m))
                elif tt <= 3:
                    if dbits < 2:
                        malformed = _bad(malformed, core, tt, dbits)
                        continue
                    sub = data & 3
                    if sub == 2:                                    # DTRD
                        types[_T_DTRD] += 1
                        if dbits < 4:
                            malformed = _bad(malformed, core, tt, dbits)
                            continue
                        if comp and not s[3]:
                            unsynced += 1
                            continue
                        dsize = (data >> 2) & 3
                        val = ((data >> 4) ^ s[1] if comp else data >> 4) & _DMASK[dsize]
                        s[1] = val
                        s[3] = True
                        if pending is not None:
                            a, m = pending
                            pending = None
                        else:
                            a = m = -1
                        now = -1 if base < 0 else (base + ticks if tick_mode else base)
                        append((now, K_READ, a, val, _DSIZE[dsize], -1, -1, CORE_NAMES[core], m))
                    else:                                           # DTA / DTWA / DTRA
                        types[_SUB_TYPE[sub]] += 1
                        if comp and not s[2]:
                            unsynced += 1
                            continue
                        full = (data >> 2) ^ s[0] if comp else data >> 2
                        s[0] = full
                        s[2] = True
                        prefix = full >> ADDR_BITS
                        a = full & 0xFFFFFFFF
                        m = (prefix >> 1) & 0xF if prefix else -1
                        if sub == 0:
                            pending = (a, m)                        # the DTWD/DTRD follows
                        else:
                            now = -1 if base < 0 else (base + ticks if tick_mode else base)
                            append((now, K_WRITE if sub == 1 else K_READ, a, -1, 0, -1, -1,
                                    CORE_NAMES[core], m))
                else:                                               # DTW (4/5), DTR (6/7)
                    types[_T_DTW if tt < 6 else _T_DTR] += 1
                    if dbits < 6:
                        malformed = _bad(malformed, core, tt, dbits)
                        continue
                    n1 = _LEN1[data & 0xF]
                    if n1 > dbits - 6:
                        n1 = dbits - 6
                    if comp and not s[3]:
                        unsynced += 1
                        continue
                    dsize = (data >> 4) & 3
                    raw = (data >> 6) & ((1 << n1) - 1)
                    val = (raw ^ s[1] if comp else raw) & _DMASK[dsize]
                    s[1] = val
                    s[3] = True
                    if comp and not s[2]:
                        unsynced += 1
                        continue
                    full = (data >> (6 + n1)) ^ s[0] if comp else data >> (6 + n1)
                    s[0] = full
                    s[2] = True
                    prefix = full >> ADDR_BITS
                    now = -1 if base < 0 else (base + ticks if tick_mode else base)
                    append((now, K_WRITE if tt < 6 else K_READ, full & 0xFFFFFFFF, val,
                            _DSIZE[dsize], -1, -1, CORE_NAMES[core],
                            (prefix >> 1) & 0xF if prefix else -1))

            elif core == CORE_TSU:
                if tt <= 1:                                         # TSR
                    types[_T_TSR] += 1
                    if comp and not tsr_ok:
                        unsynced += 1
                        continue
                    value = (data ^ tsr if comp else data) & 0xFFFFFFFF
                    if tsr is not None and value < tsr:
                        if comp and tsr - value < 1 << 31:
                            # The XOR chain broke (a TSR was dropped under
                            # overload): time is unknown until the next
                            # uncompressed TSR.
                            backsteps += 1
                            suspect = True
                            tsr_ok = False
                            base = -1
                            continue
                        if tsr - value > 1 << 31:
                            epoch += 1 << 32            # a real wrap
                        else:
                            # A backward step: the paragraph before was torn.
                            backsteps += 1
                            suspect = True
                    tsr = value
                    base = epoch + value
                    ticks = 0
                    tsr_ok = True
                    if not comp and first is None:
                        first = base
                elif tt <= 3:                                       # TSA
                    types[_T_TSA] += 1
                    if comp and not tsa_ok:
                        unsynced += 1
                        continue
                    tsa = (data ^ tsa if comp else data) & 0xFFFFFFFF
                    tsa_ok = True
                else:
                    unknown += 1

            elif core == CORE_WTU:
                if tt <= 1:                                         # WPS
                    types[_T_WPS] += 1
                    if comp and not wps_ok:
                        unsynced += 1
                        continue
                    wps = data ^ wps if comp else data
                    wps_ok = True
                    now = -1 if base < 0 else (base + ticks if tick_mode else base)
                    append((now, K_WPS, -1, -1, 0, wps & 0x7, -1, 'WTU', -1))
                elif tt <= 3:                                       # WPM
                    types[_T_WPM] += 1
                    if comp and not wpm_ok:
                        unsynced += 1
                        continue
                    wpm = data ^ wpm if comp else data
                    wpm_ok = True
                    now = -1 if base < 0 else (base + ticks if tick_mode else base)
                    append((now, K_WPM, -1, -1, 0, wpm & 0xFF, -1, 'WTU', -1))
                elif tt <= 5:                                       # EVC
                    types[_T_EVC] += 1
                    if dbits < 4:
                        malformed = _bad(malformed, core, tt, dbits)
                        continue
                    n1 = _LEN1[data & 0xF]
                    if n1 > dbits - 4:
                        n1 = dbits - 4
                    cid = (data >> 4) & ((1 << n1) - 1)
                    cnt = data >> (4 + n1)
                    if comp:
                        if not evc_ok:
                            unsynced += 1
                            continue
                        cid ^= evc_id
                        cnt ^= evc_cnt
                    evc_id, evc_cnt, evc_ok = cid, cnt, True
                    now = -1 if base < 0 else (base + ticks if tick_mode else base)
                    append((now, K_EVC, -1, -1, 0, cid, cnt, 'WTU', -1))
                else:
                    unknown += 1
            else:
                unknown += 1

        self.tsr, self.epoch, self.base_cycles, self.ticks, self.tsr_ok = (tsr, epoch, base,
                                                                            ticks, tsr_ok)
        self.tsa, self.tsa_ok = tsa, tsa_ok
        self.wps, self.wps_ok, self.wpm, self.wpm_ok = wps, wps_ok, wpm, wpm_ok
        self.evc_id, self.evc_cnt, self.evc_ok = evc_id, evc_cnt, evc_ok
        st.messages += msgs
        st.unknown += unknown
        st.unsynced += unsynced
        st.backsteps += backsteps
        st.errors += errors
        st.lost_messages += lost
        if malformed:
            for k, n in malformed.items():
                st.malformed[k] = st.malformed.get(k, 0) + n
        by = st.by_type
        for i, n in enumerate(types):
            if n:
                by[_TYPES[i]] = n
        return suspect, first


def _bad(malformed: dict | None, core: int, tt: int, dbits: int) -> dict:
    key = '%s/TT%d/%db' % (CORE_NAMES.get(core, core), tt, dbits)
    malformed = malformed if malformed is not None else {}
    malformed[key] = malformed.get(key, 0) + 1
    return malformed


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
