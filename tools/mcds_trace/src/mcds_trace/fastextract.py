"""signals.Extractor.feed_rows compiled with numba, on decoder columns.

The kernel walks the events in order with the extractor's shadow memory (a
flat byte array that signals.Shadow shares), and emits one (signal, time,
raw bits) triple per sample - the signal's bytes as a little-endian integer
after the access - or a flag for address-only accesses (NaN) and watch-point
hits (1.0).  Turning raw bits into values (signed, bitfields, floats) is then
done per signal with numpy.  tests/test_fastextract.py holds it to the
row-by-row reference.
"""

from __future__ import annotations

import numpy as np

from .decode import K_GAP, K_LOST, K_READ, K_WPM, K_WPS, K_WRITE
from .fastdecode import HAVE_NUMBA

if HAVE_NUMBA:
    from numba import njit
    _jit = njit(cache=True, nogil=True)
else:                                                # pragma: no cover
    def _jit(f):
        return f

F_RAW, F_NAN, F_ONE, F_DATA = 0, 1, 2, 3     # F_DATA: raw, NaN if it decodes to nothing


@_jit
def _extract(cyc, kind, addr, value, size, wp, count, start, t0, hz, t_end,
             s_addr, s_end, s_off, max_size, single, single_size,
             r_base, r_len, r_off, data, known, hit_sig,
             o_sig, o_t, o_raw, o_flag, o_gap, cnt):
    """Events start.. into samples; stops when an output is full.  Returns
    (next event, samples, gap times, t_end).  cnt: events, untimed,
    unmatched, lost, gaps."""
    m = 0
    g = 0
    cap = len(o_sig)
    gcap = len(o_gap)
    nsig = len(s_addr)
    nr = len(r_base)
    i = start
    n = len(cyc)
    while i < n:
        if cap - m < max(nsig, 8) or g >= gcap:
            break
        k = kind[i]
        c = cyc[i]
        cnt[0] += 1
        if k == K_GAP:
            cnt[4] += 1
            o_gap[g] = t_end
            g += 1
            i += 1
            continue
        if k == K_LOST:
            cnt[3] += max(count[i], 0)
            o_gap[g] = t_end if c < 0 else ((c - t0) / hz if hz else float(c - t0))
            g += 1
            i += 1
            continue
        if c < 0:
            cnt[1] += 1
            i += 1
            continue
        t = (c - t0) / hz if hz else float(c - t0)
        if t > t_end:
            t_end = t
        if k <= K_READ:
            a = addr[i]
            v = value[i]
            sz = np.int64(size[i])
            if sz == 0:
                sz = 4
            if a < 0:
                # data only: attributable only when one signal is traced
                if single < 0 or v < 0:
                    cnt[2] += 1
                else:
                    nb = min(max(np.int64(size[i]), 1), single_size)
                    raw = np.uint64(v)
                    if nb < 8:
                        raw &= (np.uint64(1) << np.uint64(8 * nb)) - np.uint64(1)
                    o_sig[m] = single
                    o_t[m] = t
                    o_raw[m] = raw
                    o_flag[m] = F_DATA
                    m += 1
                i += 1
                continue
            lo = a
            hi = a + sz
            j0 = np.searchsorted(s_addr, lo - max_size)
            hit0 = -1
            hits = 0
            j = j0
            while j < nsig and s_addr[j] < hi:
                if s_end[j] > lo:
                    if hit0 < 0:
                        hit0 = j
                    hits += 1
                j += 1
            if hits == 0:
                cnt[2] += 1
                i += 1
                continue
            if v < 0:
                j = hit0
                while j < nsig and s_addr[j] < hi:
                    if s_end[j] > lo:
                        o_sig[m] = j
                        o_t[m] = t
                        o_raw[m] = 0
                        o_flag[m] = F_NAN
                        m += 1
                    j += 1
                i += 1
                continue
            # the store into the shadow
            uv = np.uint64(v)
            for r in range(nr):
                b = r_base[r]
                e = b + r_len[r]
                x0 = max(lo, b)
                x1 = min(hi, e)
                for x in range(x0, x1):
                    sh = x - lo
                    byte = (uv >> np.uint64(8 * sh)) & np.uint64(0xFF) if sh < 8 else np.uint64(0)
                    data[r_off[r] + x - b] = np.uint8(byte)
                    known[r_off[r] + x - b] = 1
            j = hit0
            while j < nsig and s_addr[j] < hi:
                if s_end[j] > lo:
                    off = s_off[j]
                    ln = s_end[j] - s_addr[j]
                    ok = True
                    raw = np.uint64(0)
                    for q in range(ln):
                        if known[off + q] == 0:
                            ok = False
                            break
                        if q < 8:
                            raw |= np.uint64(data[off + q]) << np.uint64(8 * q)
                    if ok:
                        o_sig[m] = j
                        o_t[m] = t
                        o_raw[m] = raw
                        o_flag[m] = F_RAW
                        m += 1
                j += 1
        elif k == K_WPS:
            w = wp[i]
            if 0 <= w < len(hit_sig) and hit_sig[w] >= 0:
                o_sig[m] = hit_sig[w]
                o_t[m] = t
                o_raw[m] = 0
                o_flag[m] = F_ONE
                m += 1
        elif k == K_WPM:
            w = wp[i]
            if w >= 0:
                for s in range(len(hit_sig)):
                    if hit_sig[s] >= 0 and (w >> s) & 1:
                        o_sig[m] = hit_sig[s]
                        o_t[m] = t
                        o_raw[m] = 0
                        o_flag[m] = F_ONE
                        m += 1
        i += 1
    return i, m, g, t_end


class Decoders:
    """Vectorised raw bits -> value for one signal (Signal.int_decoder's cases)."""

    def __init__(self, sig):
        self.sig = sig
        size = sig.size
        node = sig.node
        self.mode, self.shift, self.nb, self.signed = 'u', 0, 8 * size, False
        if node is None:
            self.signed = sig.signed
            self.mode = 's' if sig.signed else 'u'
        elif node.size != node.type.size or node.type.size == 0:
            self.mode = 'py'
        elif node.bit_size:
            self.mode = 'bf'
            self.shift, self.nb = node.bit_shift, node.bit_size
            self.signed = node.type.encoding == 'signed'
        elif node.type.kind == 'base' and node.type.encoding == 'float':
            self.mode = {4: 'f32', 8: 'f64'}.get(size, 'none')
        else:
            self.signed = node.type.encoding == 'signed'
            self.mode = 's' if self.signed else 'u'

    def __call__(self, raw: np.ndarray):
        """Values (float64) and a keep-mask (False: no value, as decode() None)."""
        mode = self.mode
        keep = None
        if mode == 'u':
            v = raw.astype(np.float64)
        elif mode == 's' or (mode == 'bf' and self.signed):
            x = raw >> np.uint64(self.shift) if mode == 'bf' else raw
            nb = self.nb
            if nb < 64:
                x = x & np.uint64((1 << nb) - 1)
                sign = np.uint64(1 << (nb - 1))
                v = ((x ^ sign).view(np.int64) - np.int64(1 << (nb - 1))).astype(np.float64)
            else:
                v = x.view(np.int64).astype(np.float64)
        elif mode == 'bf':
            v = ((raw >> np.uint64(self.shift)) & np.uint64((1 << self.nb) - 1)).astype(np.float64)
        elif mode == 'f32':
            with np.errstate(invalid='ignore'):             # signalling NaNs
                v = raw.astype(np.uint32).view(np.float32).astype(np.float64)
        elif mode == 'f64':
            v = raw.view(np.float64).copy()
        elif mode == 'none':
            v = np.full(len(raw), np.nan)
            keep = np.zeros(len(raw), dtype=bool)
        else:                                       # odd layouts: the Python decoder
            size = self.sig.size
            vals = [self.sig.decode(int(r).to_bytes(8, 'little')[:size]) for r in raw.tolist()]
            keep = np.array([x is not None for x in vals], dtype=bool)
            v = np.array([np.nan if x is None else float(x) for x in vals], dtype=np.float64)
        return v, keep


def warm() -> None:
    """Compile (or load from numba's cache) both kernels now, not on the first
    paragraphs of a live trace: a cold compile takes seconds."""
    if not HAVE_NUMBA:
        return
    from .fastdecode import Columns, FastDecoder
    from .signals import Extractor, Signal, SignalStore, hits_signal
    FastDecoder().decode([bytes(64)], [False], [True])
    ex = Extractor([Signal('w', 'w', 0x1000, 4), hits_signal(0, 'h')], SignalStore(), 1e8)
    ex.feed_columns(Columns.from_rows([(5, K_WRITE, 0x1000, 1, 4, -1, -1, 'DTU0', -1)]))
