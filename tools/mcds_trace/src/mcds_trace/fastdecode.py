"""The decoder (decode.Decoder) compiled with numba: batches of paragraphs in,
numpy event columns out.

decode.Decoder is the reference: one paragraph at a time, a tuple per event,
about 7 MB/s of trace in CPython.  A wide fast trace delivers up to ~12 MB/s,
so the live path runs this instead: the same framing, compression, time
unwrapping and torn/lapped paragraph handling (tests/test_fastdecode.py
holds the two to identical output), compiled, without the interpreter lock.

Differences, none of which a TC3xx trace produces: an address or counter
field wider than 64 bits keeps its low 64 bits; ERR/EVC counts are int64.

Without numba (or with MCDS_TRACE_NO_JIT=1) `make_decoder` returns
the reference decoder behind the same batch interface.
"""

from __future__ import annotations

import os

import numpy as np

from .decode import (_LEN1, _SYMLEN, _TYPES, K_EVC, K_GAP, K_LOST, K_READ, K_WPM, K_WPS,
                     K_WRITE, LAP_FACTOR, LAP_MIN_INTERVALS, GAP_ROW, DecodeStats, Decoder,
                     event_of)
from .tram import (CODE_ABSIZE, CODE_ERROR, CODE_MULTICK, CODE_TICK, CORE_DTU0, CORE_DTU1,
                   CORE_NAMES, CORE_SKIP, CORE_TSU, CORE_WTU)

try:
    if os.environ.get('MCDS_TRACE_NO_JIT'):
        raise ImportError('disabled')
    from numba import njit
    HAVE_NUMBA = True
except ImportError:                                  # pragma: no cover
    HAVE_NUMBA = False

COLUMNS = (('cycles', np.int64), ('kind', np.uint8), ('addr', np.int64), ('value', np.int64),
           ('size', np.uint8), ('wp', np.int64), ('count', np.int64), ('core', np.uint8),
           ('master', np.int8))
CORE_NAME = [CORE_NAMES.get(c, '?') for c in range(32)]
_CORE_ID = {v: k for k, v in CORE_NAMES.items()}


class Columns:
    """Decoded events as numpy columns (decode.ROW_FIELDS, `core` as the ID)."""

    __slots__ = tuple(c for c, _ in COLUMNS) + ('n',)

    def __init__(self, n: int = 0, arrays: dict | None = None):
        self.n = n
        for name, dt in COLUMNS:
            setattr(self, name, arrays[name] if arrays else np.empty(n, dtype=dt))

    def __len__(self) -> int:
        return self.n

    def head(self, n: int) -> 'Columns':
        return Columns(n, {name: getattr(self, name)[:n] for name, _ in COLUMNS})

    @classmethod
    def concat(cls, parts: list['Columns']) -> 'Columns':
        parts = [p for p in parts if p.n]
        if not parts:
            return cls(0)
        if len(parts) == 1:
            return parts[0]
        return cls(sum(p.n for p in parts),
                   {name: np.concatenate([getattr(p, name) for p in parts]) for name, _ in COLUMNS})

    @classmethod
    def from_rows(cls, rows: list[tuple]) -> 'Columns':
        c = cls(len(rows))
        if rows:
            cols = list(zip(*rows))
            for i, (name, dt) in enumerate(COLUMNS):
                if name == 'core':
                    getattr(c, name)[:] = [_CORE_ID.get(x, 0) for x in cols[i]]
                else:
                    getattr(c, name)[:] = cols[i]
        return c

    def rows(self) -> list[tuple]:
        """As decode.Decoder.rows returns them (for tests and compatibility)."""
        cols = [getattr(self, name).tolist() for name, _ in COLUMNS]
        out = []
        for c, k, a, v, s, w, n, core, m in zip(*cols):
            if k == K_GAP:
                out.append(GAP_ROW)
            else:
                out.append((c, k, a, v, s, w, n,
                            ('WTU' if k in (K_WPS, K_WPM, K_EVC) else CORE_NAME[core]), m))
        return out


# -- the compiled kernel -------------------------------------------------------

# uint64 state
_U_TSR, _U_TSA, _U_WPS, _U_WPM, _U_EVC_ID, _U_EVC_CNT, _U_D0A, _U_D0D, _U_D1A, _U_D1D = range(10)
# int64 state
(_S_TSR_SET, _S_EPOCH, _S_BASE, _S_TICKS, _S_TSR_OK, _S_TSA_OK, _S_WPS_OK, _S_WPM_OK,
 _S_EVC_OK, _S_PREV_FIRST, _S_INTERVALS, _S_D0AOK, _S_D0DOK, _S_D1AOK, _S_D1DOK,
 _S_LAST) = range(16)
# counters
(_C_PARAS, _C_MSGS, _C_UNKNOWN, _C_ERRORS, _C_LOSTM, _C_GAPS, _C_BACK, _C_UNSYNC, _C_TORN,
 _C_LAPS, _C_MALF) = range(11)
_C_TYPES = 11
(_T_DTWD, _T_DTRD, _T_DTA, _T_DTWA, _T_DTRA, _T_DTW, _T_DTR, _T_WPS, _T_WPM, _T_EVC,
 _T_TSR, _T_TSA) = range(12)

_SYMLEN_A = np.array(_SYMLEN, dtype=np.int64)
_LEN1_A = np.array(_LEN1, dtype=np.int64)
_SUBTYPE_A = np.array([_T_DTA, _T_DTWA, 0, _T_DTRA], dtype=np.int64)
_DSIZE_A = np.array([1, 2, 4, 4], dtype=np.uint8)
_DMASK_A = np.array([0xFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF], dtype=np.uint64)

if HAVE_NUMBA:
    _jit = njit(cache=True, nogil=True)
    _inline = njit(cache=True, nogil=True, inline='always')
else:                                                # pragma: no cover
    def _jit(f):
        return f
    _inline = _jit


@_inline
def _bits(buf, bit, n):
    """n (0..64) bits of buf from bit position `bit`, LSB first."""
    if n <= 0:
        return np.uint64(0)
    byte = bit >> 3
    sh = bit & 7
    lo = np.uint64(0)
    for i in range(8):
        lo |= np.uint64(buf[byte + i]) << np.uint64(8 * i)
    v = lo >> np.uint64(sh)
    if sh != 0 and n > 64 - sh:
        v |= np.uint64(buf[byte + 8]) << np.uint64(64 - sh)
    if n < 64:
        v &= (np.uint64(1) << np.uint64(n)) - np.uint64(1)
    return v


@_inline
def _emit(n, cyc, kind, addr, val, size, wp, cnt, core, master,
          o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core, o_master):
    o_cyc[n] = cyc
    o_kind[n] = kind
    o_addr[n] = addr
    o_val[n] = val
    o_size[n] = size
    o_wp[n] = wp
    o_cnt[n] = cnt
    o_core[n] = core
    o_master[n] = master
    return n + 1


@_inline
def _reset(us, st):
    us[_U_D0A] = 0
    us[_U_D0D] = 0
    us[_U_D1A] = 0
    us[_U_D1D] = 0
    st[_S_D0AOK] = 0
    st[_S_D0DOK] = 0
    st[_S_D1AOK] = 0
    st[_S_D1DOK] = 0
    us[_U_WPS] = 0
    us[_U_WPM] = 0
    us[_U_EVC_ID] = 0
    us[_U_EVC_CNT] = 0
    st[_S_WPS_OK] = 0
    st[_S_WPM_OK] = 0
    st[_S_EVC_OK] = 0
    us[_U_TSA] = 0
    st[_S_TSA_OK] = 0
    st[_S_TSR_OK] = 0
    st[_S_BASE] = -1
    st[_S_TICKS] = 0


@_jit
def _lapped(buf, start, end, us, st, fs, symlen):
    if st[_S_TSR_OK] == 0 or st[_S_TSR_SET] == 0 or st[_S_INTERVALS] < LAP_MIN_INTERVALS:
        return False
    pos = start
    k = 0
    while end - pos >= 4:
        code = _bits(buf, pos, 4)
        if code == CODE_TICK:
            pos += 4
        elif code == CODE_MULTICK:
            if end - pos < 12:
                break
            filler = _bits(buf, pos + 4, 8) == 0
            pos += 12
            if filler:
                continue
        elif code == CODE_ERROR:
            if end - pos < 16:
                break
            pos += 16
        else:
            if code == CODE_ABSIZE:
                if end - pos < 12:
                    break
                length = np.int64(_bits(buf, pos + 4, 8))
                if length == 0xFF:
                    break
                hdr = 12
            else:
                length = symlen[code]
                hdr = 4
            if length > end - pos - hdr:
                break
            bp = pos + hdr
            pos += hdr + length
            if length < 8:
                continue
            core = _bits(buf, bp, 5)
            if core == CORE_SKIP:
                break
            if core == CORE_TSU and _bits(buf, bp + 5, 3) == 0:
                dbits = length - 8
                jump = (_bits(buf, bp + 8, min(dbits, 64)) - us[_U_TSR]) & np.uint64(0xFFFFFFFF)
                return LAP_FACTOR * fs[0] < np.float64(jump) and jump < np.uint64(1 << 31)
        if k >= 16:
            break
        k += 1
    return False


@_jit
def _decode(buf, poff, flags, tick_mode, us, st, fs, cnt, malf, symlen, len1, subtype,
            dsize_a, dmask_a, o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core,
            o_master):
    """Decode paragraphs p (bytes poff[p]..poff[p+1] of buf, flags bit0 gap,
    bit1 final) into the o_* columns; returns the row count."""
    M32 = np.uint64(0xFFFFFFFF)
    n = 0
    for p in range(len(poff) - 1):
        start = poff[p] * 8
        end = poff[p + 1] * 8
        gap = (flags[p] & 1) != 0
        final = (flags[p] & 2) != 0
        if not gap and _lapped(buf, start, end, us, st, fs, symlen):
            cnt[_C_LAPS] += 1
            gap = True
        if gap:
            cnt[_C_GAPS] += 1
            _reset(us, st)
            n = _emit(n, -1, K_GAP, -1, -1, 0, -1, -1, 0, -1, o_cyc, o_kind, o_addr, o_val,
                      o_size, o_wp, o_cnt, o_core, o_master)
        cnt[_C_PARAS] += 1
        sv_tsr = us[_U_TSR]
        sv_set = st[_S_TSR_SET]
        sv_epoch = st[_S_EPOCH]
        sv_base = st[_S_BASE]
        sv_ticks = st[_S_TICKS]
        unk0 = cnt[_C_UNKNOWN]
        malf0 = cnt[_C_MALF]
        mark = n

        # -- one paragraph (decode.Decoder._scan) --
        tsr = us[_U_TSR]
        tsr_set = st[_S_TSR_SET] != 0
        epoch = st[_S_EPOCH]
        base = st[_S_BASE]
        ticks = st[_S_TICKS]
        tsr_ok = st[_S_TSR_OK] != 0
        pend_a = np.int64(-1)
        pend_m = np.int64(-1)
        have_pend = False
        suspect = False
        first = np.int64(-1)
        pos = start
        while end - pos >= 4:
            code = _bits(buf, pos, 4)
            if code == CODE_TICK:
                ticks += 1
                pos += 4
                continue
            if code == CODE_MULTICK:
                if end - pos < 12:
                    break
                ticks += np.int64(_bits(buf, pos + 4, 8))
                pos += 12
                continue
            if code == CODE_ERROR:
                if end - pos < 16:
                    break
                ec = np.int64(_bits(buf, pos + 9, 7))
                cnt[_C_ERRORS] += 1
                cnt[_C_LOSTM] += ec
                now = -1 if base < 0 else (base + ticks if tick_mode else base)
                n = _emit(n, now, K_LOST, -1, -1, 0, -1, ec, np.uint8(_bits(buf, pos + 4, 5)), -1,
                          o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core, o_master)
                pos += 16
                continue
            if code == CODE_ABSIZE:
                if end - pos < 12:
                    break
                length = np.int64(_bits(buf, pos + 4, 8))
                if length == 0xFF:                  # <endoftrace>
                    suspect = suspect or not final
                    break
                hdr = 12
            else:
                length = symlen[code]
                if length == 0:
                    suspect = True
                    break
                hdr = 4
            left = end - pos - hdr
            if length > left:
                suspect = suspect or left > 32
                break
            if length < 8:
                pos += hdr + length
                continue
            bp = pos + hdr
            pos += hdr + length
            core = np.int64(_bits(buf, bp, 5))
            if core == CORE_SKIP:
                break
            tt = np.int64(_bits(buf, bp + 5, 3))
            db = bp + 8
            dbits = length - 8
            data = _bits(buf, db, min(dbits, 64))
            cnt[_C_MSGS] += 1
            comp = (tt & 1) != 0
            now = -1 if base < 0 else (base + ticks if tick_mode else base)

            if core == CORE_DTU0 or core == CORE_DTU1:
                ia = _U_D0A if core == CORE_DTU0 else _U_D1A
                idd = ia + 1
                iaok = _S_D0AOK if core == CORE_DTU0 else _S_D1AOK
                idok = iaok + 1
                if tt <= 1:                                             # DTWD
                    cnt[_C_TYPES + _T_DTWD] += 1
                    if dbits < 2:
                        malf[core, tt, dbits] += 1
                        cnt[_C_MALF] += 1
                        continue
                    if comp and st[idok] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    dsize = np.int64(data & np.uint64(3))
                    val = data >> np.uint64(2)
                    if comp:
                        val ^= us[idd]
                    val &= dmask_a[dsize]
                    us[idd] = val
                    st[idok] = 1
                    if have_pend:
                        a, m = pend_a, pend_m
                        have_pend = False
                    else:
                        a = np.int64(-1)
                        m = np.int64(-1)
                    n = _emit(n, now, K_WRITE, a, np.int64(val), dsize_a[dsize], -1, -1, core, m,
                              o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core,
                              o_master)
                elif tt <= 3:
                    if dbits < 2:
                        malf[core, tt, dbits] += 1
                        cnt[_C_MALF] += 1
                        continue
                    sub = np.int64(data & np.uint64(3))
                    if sub == 2:                                        # DTRD
                        cnt[_C_TYPES + _T_DTRD] += 1
                        if dbits < 4:
                            malf[core, tt, dbits] += 1
                            cnt[_C_MALF] += 1
                            continue
                        if comp and st[idok] == 0:
                            cnt[_C_UNSYNC] += 1
                            continue
                        dsize = np.int64((data >> np.uint64(2)) & np.uint64(3))
                        val = data >> np.uint64(4)
                        if comp:
                            val ^= us[idd]
                        val &= dmask_a[dsize]
                        us[idd] = val
                        st[idok] = 1
                        if have_pend:
                            a, m = pend_a, pend_m
                            have_pend = False
                        else:
                            a = np.int64(-1)
                            m = np.int64(-1)
                        n = _emit(n, now, K_READ, a, np.int64(val), dsize_a[dsize], -1, -1, core,
                                  m, o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core,
                                  o_master)
                    else:                                               # DTA / DTWA / DTRA
                        cnt[_C_TYPES + subtype[sub]] += 1
                        if comp and st[iaok] == 0:
                            cnt[_C_UNSYNC] += 1
                            continue
                        full = _bits(buf, db + 2, min(dbits - 2, 64))
                        if comp:
                            full ^= us[ia]
                        us[ia] = full
                        st[iaok] = 1
                        prefix = full >> np.uint64(32)
                        a = np.int64(full & M32)
                        m = np.int64((prefix >> np.uint64(1)) & np.uint64(0xF)) if prefix else -1
                        if sub == 0:
                            pend_a, pend_m = a, m
                            have_pend = True
                        else:
                            n = _emit(n, now, K_WRITE if sub == 1 else K_READ, a, -1, 0, -1, -1,
                                      core, m, o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt,
                                      o_core, o_master)
                else:                                                   # DTW / DTR
                    cnt[_C_TYPES + (_T_DTW if tt < 6 else _T_DTR)] += 1
                    if dbits < 6:
                        malf[core, tt, dbits] += 1
                        cnt[_C_MALF] += 1
                        continue
                    n1 = len1[np.int64(data & np.uint64(0xF))]
                    if n1 > dbits - 6:
                        n1 = dbits - 6
                    if comp and st[idok] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    dsize = np.int64((data >> np.uint64(4)) & np.uint64(3))
                    val = _bits(buf, db + 6, n1)
                    if comp:
                        val ^= us[idd]
                    val &= dmask_a[dsize]
                    us[idd] = val
                    st[idok] = 1
                    if comp and st[iaok] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    full = _bits(buf, db + 6 + n1, min(dbits - 6 - n1, 64))
                    if comp:
                        full ^= us[ia]
                    us[ia] = full
                    st[iaok] = 1
                    prefix = full >> np.uint64(32)
                    m = np.int64((prefix >> np.uint64(1)) & np.uint64(0xF)) if prefix else -1
                    n = _emit(n, now, K_WRITE if tt < 6 else K_READ, np.int64(full & M32),
                              np.int64(val), dsize_a[dsize], -1, -1, core, m, o_cyc, o_kind,
                              o_addr, o_val, o_size, o_wp, o_cnt, o_core, o_master)

            elif core == CORE_TSU:
                if tt <= 1:                                             # TSR
                    cnt[_C_TYPES + _T_TSR] += 1
                    if comp and not tsr_ok:
                        cnt[_C_UNSYNC] += 1
                        continue
                    value = (data ^ tsr if comp else data) & M32
                    if tsr_set and value < tsr:
                        back = tsr - value
                        if comp and back < np.uint64(1 << 31):
                            cnt[_C_BACK] += 1
                            suspect = True
                            tsr_ok = False
                            base = -1
                            continue
                        if back > np.uint64(1 << 31):
                            epoch += np.int64(1) << 32
                        else:
                            cnt[_C_BACK] += 1
                            suspect = True
                    tsr = value
                    tsr_set = True
                    base = epoch + np.int64(value)
                    ticks = 0
                    tsr_ok = True
                    if not comp and first < 0:
                        first = base
                elif tt <= 3:                                           # TSA
                    cnt[_C_TYPES + _T_TSA] += 1
                    if comp and st[_S_TSA_OK] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    us[_U_TSA] = ((data ^ us[_U_TSA]) if comp else data) & M32
                    st[_S_TSA_OK] = 1
                else:
                    cnt[_C_UNKNOWN] += 1

            elif core == CORE_WTU:
                if tt <= 1:                                             # WPS
                    cnt[_C_TYPES + _T_WPS] += 1
                    if comp and st[_S_WPS_OK] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    us[_U_WPS] = (data ^ us[_U_WPS]) if comp else data
                    st[_S_WPS_OK] = 1
                    n = _emit(n, now, K_WPS, -1, -1, 0, np.int64(us[_U_WPS] & np.uint64(7)), -1,
                              CORE_WTU, -1, o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt,
                              o_core, o_master)
                elif tt <= 3:                                           # WPM
                    cnt[_C_TYPES + _T_WPM] += 1
                    if comp and st[_S_WPM_OK] == 0:
                        cnt[_C_UNSYNC] += 1
                        continue
                    us[_U_WPM] = (data ^ us[_U_WPM]) if comp else data
                    st[_S_WPM_OK] = 1
                    n = _emit(n, now, K_WPM, -1, -1, 0, np.int64(us[_U_WPM] & np.uint64(0xFF)),
                              -1, CORE_WTU, -1, o_cyc, o_kind, o_addr, o_val, o_size, o_wp,
                              o_cnt, o_core, o_master)
                elif tt <= 5:                                           # EVC
                    cnt[_C_TYPES + _T_EVC] += 1
                    if dbits < 4:
                        malf[core, tt, dbits] += 1
                        cnt[_C_MALF] += 1
                        continue
                    n1 = len1[np.int64(data & np.uint64(0xF))]
                    if n1 > dbits - 4:
                        n1 = dbits - 4
                    cid = _bits(buf, db + 4, n1)
                    cc = _bits(buf, db + 4 + n1, min(dbits - 4 - n1, 64))
                    if comp:
                        if st[_S_EVC_OK] == 0:
                            cnt[_C_UNSYNC] += 1
                            continue
                        cid ^= us[_U_EVC_ID]
                        cc ^= us[_U_EVC_CNT]
                    us[_U_EVC_ID] = cid
                    us[_U_EVC_CNT] = cc
                    st[_S_EVC_OK] = 1
                    n = _emit(n, now, K_EVC, -1, -1, 0, np.int64(cid), np.int64(cc), CORE_WTU, -1,
                              o_cyc, o_kind, o_addr, o_val, o_size, o_wp, o_cnt, o_core,
                              o_master)
                else:
                    cnt[_C_UNKNOWN] += 1
            else:
                cnt[_C_UNKNOWN] += 1

        us[_U_TSR] = tsr
        st[_S_TSR_SET] = 1 if tsr_set else 0
        st[_S_EPOCH] = epoch
        st[_S_BASE] = base
        st[_S_TICKS] = ticks
        st[_S_TSR_OK] = 1 if tsr_ok else 0
        if base >= 0 and base + ticks > st[_S_LAST]:
            st[_S_LAST] = base + ticks
        # -- end of the paragraph (decode.Decoder.rows) --

        if suspect or cnt[_C_UNKNOWN] != unk0 or cnt[_C_MALF] != malf0:
            cnt[_C_TORN] += 1
            n = mark
            _reset(us, st)
            us[_U_TSR] = sv_tsr
            st[_S_TSR_SET] = sv_set
            st[_S_EPOCH] = sv_epoch
            st[_S_BASE] = sv_base
            st[_S_TICKS] = sv_ticks
            st[_S_TSR_OK] = 0
            st[_S_BASE] = -1
            st[_S_PREV_FIRST] = -1
            n = _emit(n, -1, K_GAP, -1, -1, 0, -1, -1, 0, -1, o_cyc, o_kind, o_addr, o_val,
                      o_size, o_wp, o_cnt, o_core, o_master)
            continue
        pf = st[_S_PREV_FIRST]
        if first >= 0 and pf >= 0 and not gap and first > pf:
            k = min(st[_S_INTERVALS], 15)
            fs[0] = (fs[0] * k + (first - pf)) / (k + 1)
            st[_S_INTERVALS] += 1
        if first >= 0:
            st[_S_PREV_FIRST] = first
    return n


# -- the decoder ---------------------------------------------------------------

class FastDecoder:
    """decode.Decoder's state and statistics, decoding batches with the kernel."""

    def __init__(self, tick_mode: bool = False):
        self.tick_mode = tick_mode
        self._us = np.zeros(10, dtype=np.uint64)
        self._st = np.zeros(16, dtype=np.int64)
        self._st[_S_BASE] = -1
        self._st[_S_PREV_FIRST] = -1
        self._fs = np.zeros(1, dtype=np.float64)
        self._cnt = np.zeros(_C_TYPES + len(_TYPES), dtype=np.int64)
        self._malf = np.zeros((32, 8, 8), dtype=np.int64)
        self._stats = DecodeStats()
        self._stats_at = -1
        self._st[_S_LAST] = -1

    def restart(self) -> None:
        """As decode.Decoder.restart: continue the timeline after the last
        time seen, TSR history and compression bases forgotten."""
        last = int(self._st[_S_LAST])
        self._us[:] = 0
        keep = (_S_INTERVALS, _S_LAST)
        for i in range(len(self._st)):
            if i not in keep:
                self._st[i] = 0
        self._st[_S_BASE] = -1
        self._st[_S_PREV_FIRST] = -1
        self._st[_S_EPOCH] = last + 1 if last >= 0 else 0

    @property
    def stats(self) -> DecodeStats:
        c = self._cnt
        if self._stats_at != c[_C_PARAS] + c[_C_GAPS] + c[_C_TORN]:
            s = self._stats
            (s.paragraphs, s.messages, s.unknown, s.errors, s.lost_messages, s.gaps, s.backsteps,
             s.unsynced, s.torn, s.laps) = (int(x) for x in c[:_C_MALF])
            s.by_type = {_TYPES[i]: int(c[_C_TYPES + i]) for i in range(len(_TYPES))
                         if c[_C_TYPES + i]}
            s.malformed = {'%s/TT%d/%db' % (CORE_NAMES.get(int(co), int(co)), int(tt), int(b)):
                           int(self._malf[co, tt, b])
                           for co, tt, b in zip(*np.nonzero(self._malf))}
            self._stats_at = c[_C_PARAS] + c[_C_GAPS] + c[_C_TORN]
        return self._stats

    @property
    def last_cycles(self) -> int:
        return int(self._st[_S_LAST])

    def decode(self, payloads: list[bytes], gaps, finals) -> Columns:
        """Paragraphs in order (gap: data was lost before it; final: it may end
        in <endoftrace>) -> event columns."""
        k = len(payloads)
        if not k:
            return Columns(0)
        poff = np.zeros(k + 1, dtype=np.int64)
        np.cumsum([len(p) for p in payloads], out=poff[1:])
        buf = np.zeros(int(poff[-1]) + 16, dtype=np.uint8)       # _bits reads 9 bytes
        buf[:poff[-1]] = np.frombuffer(b''.join(payloads), dtype=np.uint8)
        flags = np.array([(1 if g else 0) | (2 if f else 0) for g, f in zip(gaps, finals)],
                         dtype=np.uint8)
        cap = int(poff[-1]) * 8 // 12 + 2 * k + 8
        out = Columns(cap)
        n = _decode(buf, poff, flags, self.tick_mode, self._us, self._st, self._fs, self._cnt,
                    self._malf, _SYMLEN_A, _LEN1_A, _SUBTYPE_A, _DSIZE_A, _DMASK_A,
                    out.cycles, out.kind, out.addr, out.value, out.size, out.wp, out.count,
                    out.core, out.master)
        return out.head(n)

    # decode.Decoder's per-paragraph interface (tests, small tools)
    def rows(self, par_bytes: bytes, index: int = 0, gap: bool = False,
             final: bool = True) -> list[tuple]:
        return self.decode([par_bytes], [gap], [final]).rows()

    def paragraph(self, par_bytes: bytes, index: int = 0, gap: bool = False, final: bool = True):
        for row in self.rows(par_bytes, index, gap, final):
            yield event_of(row, index)


class _RefDecoder(Decoder):
    """The reference decoder behind FastDecoder's batch interface."""

    def decode(self, payloads: list[bytes], gaps, finals) -> Columns:
        rows: list[tuple] = []
        for p, g, f in zip(payloads, gaps, finals):
            rows += self.rows(p, 0, g, f)
        return Columns.from_rows(rows)


def make_decoder(tick_mode: bool = False):
    return FastDecoder(tick_mode) if HAVE_NUMBA else _RefDecoder(tick_mode)
