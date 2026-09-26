"""From decoded trace events to plottable signals.

A Signal is one traced quantity: an ELF leaf (a number, enum, pointer or
bitfield inside a variable), a raw address range, or the watch-point hits of a
slot (compact mode).  The Extractor keeps a shadow of the watched memory, so a
store of any width updates every signal it overlaps and a signal is decoded
from all of its bytes - a byte store into a struct, the two halves of a
64-bit value or a bitfield read-modify-write come out right.  The probe's
start snapshot seeds the shadow; without it a signal starts at its first
complete store.

Samples go into a SignalStore: per signal, growing float64 arrays of time
(seconds since the first timed event) and value, safe to read from another
thread while the extractor appends.
"""

from __future__ import annotations

import struct
import threading
from dataclasses import dataclass, field

import numpy as np

from .decode import K_GAP, K_LOST, K_READ, K_WPM, K_WPS, row_of
from .elfsyms import Node, decode_leaf, format_value

_F32 = struct.Struct('<f')
_F64 = struct.Struct('<d')

PALETTE = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
           '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#393b79', '#e6550d',
           '#31a354', '#756bb1', '#636363', '#843c39']


@dataclass
class Signal:
    id: str                       # stable key: the ELF path, 'raw:0x..:n' or 'hits:j'
    label: str
    addr: int = 0
    size: int = 0
    kind: str = 'value'           # 'value' | 'hits'
    node: Node | None = None      # decoding; None: unsigned little-endian of `size`
    slot: int = -1                # hits: the watch slot
    color: str = PALETTE[0]
    signed: bool = False          # raw signals only

    @property
    def end(self) -> int:
        return self.addr + self.size

    def decode(self, raw: bytes):
        if self.node is not None:
            return decode_leaf(self.node, raw)
        return int.from_bytes(raw[:self.size], 'little', signed=self.signed)

    def int_decoder(self):
        """A function from a store of exactly this signal's bytes (as the
        unsigned little-endian integer) to what decode() gives for them."""
        size = self.size
        node = self.node
        if node is None:
            signed = self.signed
        elif node.size != node.type.size or node.type.size == 0:
            return lambda v: decode_leaf(node, v.to_bytes(size, 'little'))
        elif node.bit_size:
            shift, nb = node.bit_shift, node.bit_size
            mask, sign = (1 << nb) - 1, 1 << (nb - 1)
            if node.type.encoding == 'signed':
                return lambda v: (((v >> shift) & mask) ^ sign) - sign
            return lambda v: (v >> shift) & mask
        elif node.type.kind == 'base' and node.type.encoding == 'float':
            if size == 4:
                return lambda v: _F32.unpack(v.to_bytes(4, 'little'))[0]
            if size == 8:
                return lambda v: _F64.unpack(v.to_bytes(8, 'little'))[0]
            return lambda v: None
        else:
            signed = node.type.encoding == 'signed'
        if signed:
            sign = 1 << (8 * size - 1)
            return lambda v: (v ^ sign) - sign
        return lambda v: v

    def format(self, value) -> str:
        if value is None or (isinstance(value, float) and np.isnan(value)):
            return '-'
        if self.node is not None:
            return format_value(self.node, int(value) if self.node.type.encoding != 'float'
                                and float(value).is_integer() else value)
        return str(int(value)) if float(value).is_integer() else '%.6g' % value

    def to_json(self) -> dict:
        d = {'id': self.id, 'label': self.label, 'addr': '0x%08X' % self.addr,
             'size': self.size, 'kind': self.kind, 'color': self.color}
        if self.kind == 'hits':
            d['slot'] = self.slot
        if self.node is None and self.kind == 'value':
            d['signed'] = self.signed
        return d


def raw_signal(addr: int, size: int, label: str | None = None, signed: bool = False) -> Signal:
    return Signal('raw:0x%08X:%d' % (addr, size), label or '0x%08X' % addr, addr, size,
                  signed=signed)


def leaf_signal(node: Node) -> Signal:
    return Signal(node.path, _short(node.path), node.addr, node.size, node=node)


def hits_signal(slot: int, label: str) -> Signal:
    return Signal('hits:%d' % slot, label, kind='hits', slot=slot)


def _short(path: str) -> str:
    """A compact label: drop leading namespaces, keep the member chain, and
    show std::array elements as plain indices."""
    head, sep, tail = path.partition('.')
    head = head.rsplit('::', 1)[-1]
    return (head + sep + tail if sep else head).replace('._M_elems[', '[')


# -- storage ------------------------------------------------------------------

class Series:
    """Growing (t, v) arrays; append from one thread, snapshot from others."""

    def __init__(self, capacity: int = 4096):
        self._t = np.empty(capacity)
        self._v = np.empty(capacity)
        self.n = 0
        self.last = np.nan            # latest value, for the readout

    def append(self, t: float, v: float) -> None:
        if self.n == len(self._t):
            grow = max(4096, len(self._t))
            self._t = np.concatenate([self._t, np.empty(grow)])
            self._v = np.concatenate([self._v, np.empty(grow)])
        self._t[self.n] = t
        self._v[self.n] = v
        self.n += 1
        self.last = v

    def extend(self, t: np.ndarray, v: np.ndarray) -> None:
        k = len(t)
        if self.n + k > len(self._t):
            cap = max(self.n + k, 2 * len(self._t))
            nt = np.empty(cap)
            nv = np.empty(cap)
            nt[:self.n] = self._t[:self.n]
            nv[:self.n] = self._v[:self.n]
            self._t, self._v = nt, nv
        self._t[self.n:self.n + k] = t
        self._v[self.n:self.n + k] = v
        self.n += k
        if k:
            self.last = v[-1]

    def view(self) -> tuple[np.ndarray, np.ndarray]:
        """Arrays of the samples so far (views; valid while the caller uses them)."""
        n = self.n
        return self._t[:n], self._v[:n]

    def drop_before(self, t_min: float) -> bool:
        """Forget samples older than t_min (keeps one before it for step plots);
        True if any went."""
        t, _ = self.view()
        i = int(np.searchsorted(t, t_min)) - 1
        if i > 0:
            k = self.n - i
            self._t[:k] = self._t[i:self.n]
            self._v[:k] = self._v[i:self.n]
            self.n = k
            return True
        return False


class SignalStore:
    """All series of a session plus its markers (gaps, overflows)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.series: dict[str, Series] = {}
        self.gaps: list[float] = []        # times where data was lost
        self.t_end = 0.0                    # latest time seen, seconds
        self.samples = 0
        self.trimmed = False                # trim() dropped old samples

    def get(self, sid: str) -> Series:
        s = self.series.get(sid)
        if s is None:
            s = self.series[sid] = Series()
        return s

    def snapshot(self, sid: str) -> tuple[np.ndarray, np.ndarray]:
        with self.lock:
            s = self.series.get(sid)
            if s is None:
                return np.empty(0), np.empty(0)
            t, v = s.view()
            return t.copy(), v.copy()

    def window(self, sid: str, x0: float, x1: float) -> tuple[np.ndarray, np.ndarray]:
        """Copies of the samples in [x0, x1] plus one on each side (a step plot
        holds the one before; lines run to the one after)."""
        with self.lock:
            s = self.series.get(sid)
            if s is None:
                return np.empty(0), np.empty(0)
            t, v = s.view()
            i0 = max(int(np.searchsorted(t, x0, side='left')) - 1, 0)
            i1 = min(int(np.searchsorted(t, x1, side='right')) + 1, len(t))
            return t[i0:i1].copy(), v[i0:i1].copy()

    def span(self) -> tuple[float, float]:
        with self.lock:
            lo = min((s._t[0] for s in self.series.values() if s.n), default=0.0)
            return lo, self.t_end

    def trim(self, keep_seconds: float) -> None:
        with self.lock:
            cut = self.t_end - keep_seconds
            for s in self.series.values():
                self.trimmed |= s.drop_before(cut)
            self.gaps = [g for g in self.gaps if g >= cut]


# -- extraction ---------------------------------------------------------------

class Shadow:
    """Bytes of the watched memory with a known-mask."""

    def __init__(self):
        self.ranges: list[tuple[int, bytearray, bytearray]] = []   # (base, data, known)

    def cover(self, lo: int, hi: int) -> None:
        """Make [lo, hi) representable; ranges stay disjoint and keep their bytes."""
        spans = sorted([(b, b + len(d)) for b, d, _ in self.ranges] + [(lo, hi)])
        merged: list[list[int]] = []
        for a, b in spans:
            if merged and a <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        new = []
        for a, b in merged:
            data, known = bytearray(b - a), bytearray(b - a)
            for base, d, k in self.ranges:
                if a <= base and base + len(d) <= b:
                    data[base - a:base - a + len(d)] = d
                    known[base - a:base - a + len(k)] = k
            new.append((a, data, known))
        self.ranges = new

    def write(self, addr: int, raw: bytes) -> None:
        for base, data, known in self.ranges:
            lo = max(addr, base)
            hi = min(addr + len(raw), base + len(data))
            if lo < hi:
                data[lo - base:hi - base] = raw[lo - addr:hi - addr]
                known[lo - base:hi - base] = b'\x01' * (hi - lo)

    def read(self, addr: int, size: int) -> bytes | None:
        """The bytes, or None if any is still unknown."""
        for base, data, known in self.ranges:
            if base <= addr and addr + size <= base + len(data):
                k = known[addr - base:addr - base + size]
                if 0 in k:
                    return None
                return bytes(data[addr - base:addr - base + size])
        return None


@dataclass
class ExtractStats:
    events: int = 0
    samples: int = 0
    untimed: int = 0              # events before the first timestamp
    unmatched: int = 0            # accesses outside every selected signal
    lost: int = 0                 # target-side overflow messages
    gaps: int = 0


class Extractor:
    """Turns decoder events into samples of the selected signals."""

    def __init__(self, signals: list[Signal], store: SignalStore, emu_hz: float,
                 config: dict | None = None, t0: int | None = None):
        self.store = store
        self.emu_hz = float(emu_hz or 0)
        self.stats = ExtractStats()
        self.t0: int | None = t0          # cycles at time zero (first timed event)
        self.shadow = Shadow()
        self.values = [s for s in signals if s.kind == 'value' and s.size > 0]
        self.hits = {s.slot: s for s in signals if s.kind == 'hits'}
        for s in self.values:
            self.shadow.cover(s.addr, s.end)
        self.values.sort(key=lambda s: s.addr)
        self._seeded = False
        self._starts = np.array([s.addr for s in self.values], dtype=np.int64)
        self._max_size = max((s.size for s in self.values), default=0)
        self.single = self.values[0] if len(self.values) == 1 else None
        self._plans: dict[int, object] = {}         # addr * 64 + size -> _plan()
        self._exact_pending: dict[int, tuple] = {}  # exact stores not in the shadow yet
        if config:
            self.seed(config)
            if self.t0 is not None:          # re-extraction: time zero is known already
                self._initial_samples()

    def seed(self, config: dict) -> None:
        """Apply the probe's start snapshot (config['snapshot'])."""
        self._seeded = False
        for sn in config.get('snapshot') or []:
            try:
                addr = int(str(sn['addr']), 0)
                raw = bytes.fromhex(sn['hex'])
            except (KeyError, ValueError, TypeError):
                continue
            self.shadow.write(addr, raw)
            self._seeded = True

    def _initial_samples(self) -> None:
        """At time zero, every signal the snapshot fully covers gets its start
        value, so variables that never change still show up."""
        if not getattr(self, '_seeded', False) or self.t0 is None:
            return
        for s in self.values:
            raw = self.shadow.read(s.addr, s.size)
            if raw is None:
                continue
            v = s.decode(raw)
            if v is not None:
                self.store.get(s.id).append(0.0, float(v))
                self.stats.samples += 1

    def _time(self, cycles: int) -> float | None:
        if cycles < 0:
            return None
        if self.t0 is None:
            self.t0 = cycles
            self._initial_samples()
        if self.emu_hz:
            return (cycles - self.t0) / self.emu_hz
        return float(cycles - self.t0)

    def _overlapping(self, lo: int, hi: int) -> list[Signal]:
        if not self.values:
            return []
        i = int(np.searchsorted(self._starts, lo - self._max_size, side='left'))
        out = []
        for s in self.values[i:]:
            if s.addr >= hi:
                break
            if s.end > lo:
                out.append(s)
        return out

    def _plan(self, addr: int, size: int):
        """How a store of `size` bytes at `addr` is handled: None (no signal),
        an _Exact list when it covers exactly the signals it touches, else the
        overlapping signals (decoded through the shadow)."""
        hit = self._overlapping(addr, addr + size)
        if not hit:
            return None
        if all(s.addr == addr and s.size == size for s in hit):
            return _Exact([(s.id, s.int_decoder()) for s in hit], hit)
        return hit

    def feed(self, events) -> int:
        """Process Events; returns the samples produced."""
        return self.feed_rows([row_of(ev) for ev in events])

    def feed_rows(self, rows) -> int:
        """Process decoder rows (decode.ROW_FIELDS); returns the samples produced."""
        st = self.stats
        store = self.store
        plans = self._plans
        exact = self._exact_pending
        pend: dict[str, tuple[list, list]] = {}
        produced = 0
        hz = self.emu_hz
        with store.lock:
            t_end = store.t_end
            t0 = self.t0
            for r in rows:
                st.events += 1
                cycles, kind = r[0], r[1]
                if kind == K_GAP:
                    st.gaps += 1
                    store.gaps.append(t_end)
                    continue
                if t0 is None and cycles >= 0:
                    self._time(cycles)          # time zero: initial samples
                    t0 = self.t0
                if kind == K_LOST:
                    st.lost += max(r[6], 0)
                    store.gaps.append(t_end if cycles < 0 else
                                      ((cycles - t0) / hz if hz else float(cycles - t0)))
                    continue
                if cycles < 0:
                    st.untimed += 1
                    continue
                t = (cycles - t0) / hz if hz else float(cycles - t0)
                if t > t_end:
                    t_end = t
                if kind <= K_READ:
                    addr, value, size = r[2], r[3], r[4] or 4
                    if addr < 0:
                        produced += self._data_only(value, r[4], t, pend)
                        continue
                    key = addr * 64 + size
                    plan = plans.get(key, 0)
                    if plan == 0:
                        plan = plans[key] = self._plan(addr, size)
                    if plan is None:
                        st.unmatched += 1
                        continue
                    if value >= 0 and type(plan) is _Exact:
                        # The whole of each signal: no shadow round trip (the
                        # shadow gets the last value before it is next read).
                        exact[key] = (addr, size, value)
                        for sid, dec in plan:
                            v = dec(value)
                            if v is None:
                                continue
                            p = pend.get(sid)
                            if p is None:
                                p = pend[sid] = ([], [])
                            p[0].append(t)
                            p[1].append(float(v))
                            produced += 1
                        continue
                    if type(plan) is _Exact:
                        plan = plan.hit             # an address-only access
                    if exact:
                        # Only a hit signal's own exact stores can overlap it.
                        for s in plan:
                            e = exact.pop(s.addr * 64 + s.size, None)
                            if e is not None:
                                self.shadow.write(e[0], e[2].to_bytes(e[1], 'little'))
                    produced += self._through_shadow(plan, addr, value, size, t, pend)
                elif kind == K_WPS:
                    s = self.hits.get(r[5])
                    if s is not None:
                        _pend(pend, s.id, t, 1.0)
                        produced += 1
                elif kind == K_WPM:
                    wp = r[5]
                    if wp >= 0:
                        for j, s in self.hits.items():
                            if wp & (1 << j):
                                _pend(pend, s.id, t, 1.0)
                                produced += 1
            store.t_end = t_end
            if exact:
                self._flush_exact()
            for sid, (tl, vl) in pend.items():
                store.get(sid).extend(np.array(tl, dtype=float), np.array(vl, dtype=float))
            st.samples += produced
            store.samples += produced
        return produced

    def _flush_exact(self) -> None:
        # An exact store's range is its signals' range, which no other
        # signal overlaps (or the store would not be exact): order is free.
        for addr, size, value in self._exact_pending.values():
            self.shadow.write(addr, value.to_bytes(size, 'little'))
        self._exact_pending.clear()

    def _data_only(self, value: int, size: int, t: float, pend: dict) -> int:
        # Data-only payload: attributable only when one signal is traced.
        s = self.single
        if s is None or value < 0:
            self.stats.unmatched += 1
            return 0
        raw = int(value).to_bytes(max(size, 1), 'little', signed=False)[:s.size]
        v = s.decode(raw.ljust(s.size, bytes(1)))
        _pend(pend, s.id, t, np.nan if v is None else float(v))
        return 1

    def _through_shadow(self, hit: list[Signal], addr: int, value: int, size: int, t: float,
                        pend: dict) -> int:
        if value < 0:
            # Address-only payload: the access is known, the value is not.
            for s in hit:
                _pend(pend, s.id, t, np.nan)
            return len(hit)
        self.shadow.write(addr, int(value).to_bytes(size, 'little', signed=False))
        n = 0
        for s in hit:
            raw = self.shadow.read(s.addr, s.size)
            if raw is None:
                continue
            v = s.decode(raw)
            if v is None:
                continue
            _pend(pend, s.id, t, float(v))
            n += 1
        return n


class _Exact(list):
    """A store plan: (signal id, int decoder) of the signals it covers exactly."""

    def __init__(self, items, hit):
        super().__init__(items)
        self.hit = hit


def _pend(pend: dict, sid: str, t: float, v: float) -> None:
    p = pend.get(sid)
    if p is None:
        p = pend[sid] = ([], [])
    p[0].append(t)
    p[1].append(v)


# -- watch slots --------------------------------------------------------------

@dataclass
class SlotPlan:
    ranges: list[tuple[int, int]] = field(default_factory=list)    # (addr, size), <= 2
    signals: list[list[str]] = field(default_factory=list)          # signal ids per range
    extra_bytes: int = 0          # watched bytes no selected signal covers

    def slots(self, access: str = 'w', names: list[str] | None = None) -> list[dict]:
        out = []
        for j in range(2):
            if j < len(self.ranges):
                a, n = self.ranges[j]
                out.append({'enabled': True, 'addr': '0x%08X' % a, 'size': n, 'access': access,
                            'name': (names[j] if names and j < len(names) else 'slot%d' % j)[:23]})
            else:
                out.append({'enabled': False})
        return out


def plan_slots(signals: list[Signal], max_slots: int = 2) -> SlotPlan:
    """Cover the selected value signals with at most two address ranges (the
    DTU has two comparator sets), watching as few extra bytes as possible."""
    spans = sorted((s.addr, s.end, s.id) for s in signals if s.kind == 'value' and s.size > 0)
    plan = SlotPlan()
    if not spans:
        return plan
    # Merge touching/overlapping spans first.
    groups: list[list] = []
    for lo, hi, sid in spans:
        if groups and lo <= groups[-1][1]:
            groups[-1][1] = max(groups[-1][1], hi)
            groups[-1][2].append(sid)
        else:
            groups.append([lo, hi, [sid]])
    if len(groups) <= max_slots:
        chosen = groups
    else:
        # Split the sorted groups into max_slots contiguous runs minimising the
        # covered bytes: cut at the (max_slots - 1) largest holes.
        holes = sorted(range(len(groups) - 1),
                       key=lambda i: groups[i + 1][0] - groups[i][1], reverse=True)
        cuts = sorted(holes[:max_slots - 1])
        chosen, start = [], 0
        for c in cuts + [len(groups) - 1]:
            run = groups[start:c + 1]
            chosen.append([run[0][0], run[-1][1], [sid for g in run for sid in g[2]]])
            start = c + 1
    used = sum(hi - lo for lo, hi, _ in spans)
    for lo, hi, ids in chosen:
        plan.ranges.append((lo, hi - lo))
        plan.signals.append(ids)
    plan.extra_bytes = max(0, sum(n for _, n in plan.ranges) - used)
    return plan
