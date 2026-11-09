"""Program trace (PTU_TC) decoding and program flow reconstruction (TC3xx TS 9.3.8, 9.4.3.2).

PTU_TC messages (core ID 0x0C, Table 387), LSB first after core_ID and trace_type:
    IPA  TT0/1: instruction pointer (absolute byte address)
    IPI  TT2/3: bytes executed since the last IPA or IPI, the taken branch included
    IPE  TT4/5: <length 1> (data 1 empty) data 2 = bytes executed to the end of the window
    IPC  TT6/7: subtype[2]=0 <length 1> data 1 = take-off address, data 2 = landing address
    IPR  TT6/7: subtype[2]=1, as IPC for a return
    ERR  <errormessage> with core PTU: err_cnt IPI messages were lost
Compression (odd trace types) XORs with the reconstructed value of the same
engine: IPA, IPC and IPR share one (the last IPA's address or the last
IPC/IPR's landing address, for both take-off and landing), IPI and IPE share
another.

Relative mode: an IPI reports the bytes executed from the current IP up to
and including a taken branch.  A direct branch's target is in the code (the
CodeModel), so the flow continues there; an indirect branch, a return or an
exception entry is followed by an IPA with the landing address.  An IPA that
follows nothing is a sync (start of trace, auto-sync, ptu_sync): the code
between the last known IP and it ran straight through.

Subroutine (nesting / compact function) mode, ptu_nesting without
ptu_enable: a direct CALL is reported as an IPA with the landing address, an
indirect one as an IPC, a return as an IPR; there is no flow in between (no
coverage below the function level).  The time axis then advances one step per
message instead of per byte executed.

Time: TSU messages (TSR) anchor the trace in emulation clock cycles; the
events between two anchors are spread over the bytes executed between them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from . import codemodel as cm
from .tram import CODE_ABSIZE, CODE_ERROR, CODE_MULTICK, CODE_TICK, CORE_PTU, CORE_SKIP, \
    CORE_TSU, LENGTH1_BITS, SYMLEN_BITS

_SYMLEN = [SYMLEN_BITS.get(c, 0) for c in range(16)]
_LEN1 = [LENGTH1_BITS.get(c, 0) for c in range(16)]
_M32 = 0xFFFFFFFF

# Slice events (for the call view): kinds
EV_BEGIN, EV_END = 1, 2
# Calls can outnumber returns (compact function trace drops short leaves'
# returns, an RTOS switches stacks): a return looks this far down for its
# caller, and a deeper call view starts over.
UNWIND_SCAN = 64
MAX_DEPTH = 256


@dataclass
class PtraceStats:
    paragraphs: int = 0
    messages: int = 0
    ipa: int = 0
    ipi: int = 0
    ipe: int = 0
    ipc: int = 0
    ipr: int = 0
    err: int = 0
    lost_ipi: int = 0             # sum of err_cnt
    gaps: int = 0                 # paragraphs missing between records
    unsynced: int = 0             # IPI/IPE while the IP was unknown
    invalid_ipa: int = 0          # IPA 0: the PTU had no valid IP (TCXCIP 0 = never seen one)
    desync: int = 0               # an IPI that did not end on an instruction / IPA mismatch
    syncs: int = 0                # IPAs that set or confirmed the IP
    bytes_executed: int = 0
    calls: int = 0
    returns: int = 0
    interrupts: int = 0
    unknown_ip: dict = field(default_factory=dict)   # address -> count (not in the ELF)
    max_depth: int = 0
    resets: int = 0               # call views started over past MAX_DEPTH

    def summary(self) -> str:
        s = ('%d paragraphs, %d messages: IPA %d, IPI %d, IPE %d, IPC %d, IPR %d, ERR %d '
             '(%d IPI lost), %d gaps' % (self.paragraphs, self.messages, self.ipa, self.ipi,
                                         self.ipe, self.ipc, self.ipr, self.err, self.lost_ipi,
                                         self.gaps))
        s += ('\nflow: %d bytes executed, %d syncs, %d desyncs, %d unsynced, %d calls, '
              '%d returns, %d interrupts/traps' % (self.bytes_executed, self.syncs, self.desync,
                                                   self.unsynced, self.calls, self.returns,
                                                   self.interrupts))
        if self.invalid_ipa:
            s += '\n%d IPA without a valid instruction pointer' % self.invalid_ipa
        if self.unknown_ip:
            top = sorted(self.unknown_ip.items(), key=lambda kv: -kv[1])[:5]
            s += '\naddresses outside the ELF: ' + ', '.join('0x%08x (%d)' % kv for kv in top)
        return s


class FlowDecoder:
    """Feed paragraphs in order (paragraph()); then coverage() and slices()."""

    def __init__(self, code: cm.CodeModel, emu_hz: float = 0.0, mode: str = 'flow'):
        self.code = code
        self.emu_hz = emu_hz
        self.calls_mode = mode == 'calls'
        self.stats = PtraceStats()
        n = len(code)
        self._diff = np.zeros(n + 1, dtype=np.int64)    # coverage: +1 at start, -1 past end
        self._entered = np.zeros(len(code.func_name), dtype=np.int64)   # calls/interrupts into
        # compression
        self._ref_ip = 0
        self._ref_ip_ok = False
        self._ref_inc = 0
        self._ref_inc_ok = False
        self._tsr = 0
        self._tsr_ok = False
        self._epoch = 0
        # flow
        self.ip: int | None = None
        self._expect = None              # kind of the indirect discontinuity awaiting an IPA
        self._ret_addr = 0               # for interrupts: where execution resumes
        self._pos = 0                    # bytes executed so far (the time axis between anchors)
        self.anchors_pos: list[int] = []
        self.anchors_cyc: list[int] = []
        self.anchors_seg: list[int] = []   # the time segment (index into breaks + 1)
        self.breaks: list[int] = []        # positions where time is not continuous
        self._end_stamp = False            # the first TSR after a break was taken
        # call view
        self.names: list[str] = []
        self._name_id: dict[str, int] = {}
        self.events: list[tuple[int, int, int]] = []   # (pos, EV_*, name id)
        self.event_segs: list[int] = []               # each event's time segment
        self._stack: list[tuple[int, int]] = []       # (name id, kind: 0 base, 1 call, 2 irq, 3 jl)

    # -- names and slices ---------------------------------------------------------

    def _nid(self, name: str) -> int:
        i = self._name_id.get(name)
        if i is None:
            i = self._name_id[name] = len(self.names)
            self.names.append(name)
        return i

    def _begin(self, addr: int, kind: int, prefix: str = '') -> None:
        if len(self._stack) >= MAX_DEPTH:
            self._close_all()
            self.stats.resets += 1
        if kind:                                     # a real entry, not the base frame
            f = self.code.function_at(addr)
            if f >= 0:
                self._entered[f] += 1
        nid = self._nid(prefix + self.code.function_name(addr))
        self._stack.append((nid, kind))
        self.events.append((self._pos, EV_BEGIN, nid))
        self.event_segs.append(len(self.breaks))
        if len(self._stack) > self.stats.max_depth:
            self.stats.max_depth = len(self._stack)

    def _end_top(self) -> int:
        nid, kind = self._stack.pop()
        self.events.append((self._pos, EV_END, nid))
        self.event_segs.append(len(self.breaks))
        return kind

    def _close_all(self) -> None:
        while self._stack:
            self._end_top()

    def _ensure_base(self) -> None:
        if not self._stack and self.ip is not None:
            self._begin(self.ip, 0)

    def _return_to(self, landing: int, want: int) -> None:
        """A return (RET, RFE, ji a11) landing in the caller at `landing`.

        When a frame below the top is the landing function, pop down to it:
        returns can go missing (compact function trace sends none for short
        leaf functions, overflows drop them).  Otherwise pop the matching kind
        of frame; returning past the oldest known frame replaces the base with
        the caller's function."""
        target = self._name_id.get(self.code.function_name(landing))
        if target is not None:
            for k in range(len(self._stack) - 2, max(len(self._stack) - 2 - UNWIND_SCAN, -1), -1):
                if self._stack[k][0] == target:
                    while len(self._stack) > k + 1:
                        self._end_top()
                    return
        while self._stack:
            kind = self._stack[-1][1]
            if kind == 0:
                self._end_top()
                break
            self._end_top()
            if kind == want or want == -1:
                break
        if not self._stack:
            self._begin(landing, 0)

    # -- coverage -------------------------------------------------------------------

    def _run(self, start: int, nbytes: int) -> int:
        """Execute `nbytes` straight from `start`; returns the index of the last
        instruction, or -1 when that does not land on the code."""
        i = self.code.index(start)
        j = self.code.index_ending(start + nbytes)
        if i < 0 or j < i:
            if i < 0:
                u = self.stats.unknown_ip
                u[start] = u.get(start, 0) + 1
            return -1
        self._diff[i] += 1
        self._diff[j + 1] -= 1
        self._pos += nbytes
        self.stats.bytes_executed += nbytes
        return j

    def _break(self) -> None:
        """Time stops being continuous here (end of a trace window, lost data)."""
        if not self.breaks or self.breaks[-1] != self._pos:
            self.breaks.append(self._pos)
            self._end_stamp = False

    def _lose(self) -> None:
        if self.ip is not None or self._stack:
            self._close_all()
        self.ip = None
        self._expect = None
        self._break()

    # -- messages ---------------------------------------------------------------------

    def _on_ipi(self, n: int, end_of_window: bool) -> None:
        if self.ip is None:
            if self._expect is None:
                self.stats.unsynced += 1
            return
        start = self.ip
        j = self._run(start, n) if n else -2
        if j == -1:
            self.stats.desync += 1
            self._lose()
            return
        if end_of_window:
            self._close_all()
            self.ip = None
            self._expect = None
            self._break()
            return
        if j == -2:
            # IPI 0: a second discontinuity in the same emulation clock cycle
            # (back-to-back branches), or an exception on the first instruction
            # of a branch target.  The IP does not advance; the IPA that follows
            # has the target.  A branch at the IP did execute.
            j = self.code.index(start)
            if j < 0:
                self._lose()
                return
            k = int(self.code.kind[j])
            if k == cm.K_SEQ:
                self._expect = cm.K_SEQ           # exception before it executed
            else:
                self._diff[j] += 1
                self._diff[j + 1] -= 1
                n = int(self.code.size[j])
                self._pos += n
                self.stats.bytes_executed += n
                self._expect = {cm.K_JUMP: cm.K_IJUMP, cm.K_COND: cm.K_IJUMP,
                                cm.K_CALL: cm.K_ICALL, cm.K_JL: cm.K_IJL}.get(k, k)
            self.ip = None
            self._ret_addr = start
            return
        k = int(self.code.kind[j])
        end = int(self.code.addr[j]) + int(self.code.size[j])
        if k == cm.K_JUMP or k == cm.K_COND:
            self.ip = int(self.code.target[j])
        elif k == cm.K_CALL or k == cm.K_JL:
            self._ensure_base_at(start)
            self.ip = int(self.code.target[j])
            self.stats.calls += 1
            self._begin(self.ip, 1 if k == cm.K_CALL else 3)
        else:
            # indirect, return, trap - or an instruction that is not a branch
            # at all: an interrupt or exception was taken after it.  The IPA
            # that follows has the landing address.
            self.ip = None
            self._expect = k
            self._ret_addr = end

    def _on_ipa(self, v: int) -> None:
        if v == 0:
            # No valid instruction pointer (seen when the trace re-enables
            # through an excluded range): unknown until the next real IPA.
            self.stats.invalid_ipa += 1
            self._lose()
            return
        exp = self._expect
        self._expect = None
        if exp is not None:
            self.stats.syncs += 1
            if exp in (cm.K_ICALL, cm.K_IJL):
                self.stats.calls += 1
                self._ensure_base_at(v)
                self._begin(v, 1 if exp == cm.K_ICALL else 3)
            elif exp == cm.K_RET:
                self.stats.returns += 1
                self._return_to(v, 1)
            elif exp == cm.K_JIRET:
                self.stats.returns += 1
                self._return_to(v, 3)
            elif exp == cm.K_RFE:
                self.stats.returns += 1
                self._return_to(v, 2)
            elif exp == cm.K_TRAP or exp == cm.K_SEQ:
                self.stats.interrupts += 1
                self._ensure_base_at(v)
                self._begin(v, 2, 'irq ' if exp == cm.K_SEQ else 'trap ')
            else:                                  # ji: a jump within the flow
                self._ensure_base_at(v)
            self.ip = v
            return
        if self.ip is not None and v != self.ip:
            # A sync: straight-line code from the last IP up to v ran meanwhile.
            if not (self.ip < v and self._run(self.ip, v - self.ip) >= 0):
                self.stats.desync += 1
                self._close_all()
        self.stats.syncs += 1
        self.ip = v
        self._ensure_base()

    def _ensure_base_at(self, v: int) -> None:
        if not self._stack:
            self._begin(v, 0)

    def _calls_ipa(self, v: int) -> None:
        """Subroutine mode: an IPA is a direct call (or the start of trace)."""
        if v == 0:
            self.stats.invalid_ipa += 1
            return
        self.stats.syncs += 1
        if not self._stack:
            self._begin(v, 0)                    # start of trace: where it is
            return
        self.stats.calls += 1
        self._begin(v, 1)

    def _on_ipcr(self, ret: bool, takeoff: int, landing: int) -> None:
        if ret:
            self.stats.returns += 1
            self._return_to(landing, 1)
        else:
            self.stats.calls += 1
            if not self._stack:
                self._begin(takeoff, 0)
            self._begin(landing, 1)
        if self.ip is not None:
            self.ip = landing

    def gap(self, restart: bool = False) -> None:
        """Paragraphs were lost: the flow, the compression and time restart.
        `restart`: the trace itself restarted (capfile.FLAG_RESTART), its time
        stamps start from zero again; the timeline continues after the last."""
        self.stats.gaps += 1
        self._lose()
        self._ref_ip_ok = self._ref_inc_ok = False
        self._tsr_ok = False
        if restart and self.anchors_cyc:
            self._epoch = self.anchors_cyc[-1] + 1

    def paragraph(self, par: bytes) -> None:
        st = self.stats
        st.paragraphs += 1
        big = int.from_bytes(par, 'little')
        end = len(par) * 8
        pos = 0
        while end - pos >= 4:
            code = (big >> pos) & 0xF
            if code == CODE_TICK:
                pos += 4
                continue
            if code == CODE_MULTICK:
                pos += 12
                continue
            if code == CODE_ERROR:
                if end - pos < 16:
                    break
                core = (big >> (pos + 4)) & 0x1F
                ec = (big >> (pos + 9)) & 0x7F
                pos += 16
                if core == CORE_PTU:
                    st.err += 1
                    st.lost_ipi += ec
                    self._lose()
                continue
            if code == CODE_ABSIZE:
                if end - pos < 12:
                    break
                length = (big >> (pos + 4)) & 0xFF
                if length == 0xFF:
                    break                      # <endoftrace>
                hdr = 12
            else:
                length = _SYMLEN[code]
                if not length:
                    break                      # damage
                hdr = 4
            if length > end - pos - hdr:
                break
            if length < 8:
                pos += hdr + length
                continue
            body = (big >> (pos + hdr)) & ((1 << length) - 1)
            pos += hdr + length
            core = body & 0x1F
            if core == CORE_SKIP:
                break
            tt = (body >> 5) & 7
            data = body >> 8
            dbits = length - 8
            st.messages += 1
            comp = tt & 1
            if core == CORE_PTU:
                if tt <= 1:                                        # IPA
                    st.ipa += 1
                    if comp and not self._ref_ip_ok:
                        self._lose()
                        continue
                    v = (data ^ self._ref_ip if comp else data) & _M32
                    self._ref_ip, self._ref_ip_ok = v, True
                    if self.calls_mode:
                        self._pos += 1
                        self._calls_ipa(v & ~1)
                    else:
                        self._on_ipa(v & ~1)
                elif tt <= 3:                                      # IPI
                    st.ipi += 1
                    if comp and not self._ref_inc_ok:
                        self._lose()
                        continue
                    v = data ^ self._ref_inc if comp else data
                    self._ref_inc, self._ref_inc_ok = v, True
                    if not self.calls_mode:
                        self._on_ipi(v, False)
                elif tt <= 5:                                      # IPE
                    st.ipe += 1
                    n1 = _LEN1[data & 0xF]
                    d2 = data >> (4 + n1)
                    if comp and not self._ref_inc_ok:
                        self._lose()
                        continue
                    v = d2 ^ self._ref_inc if comp else d2
                    self._ref_inc, self._ref_inc_ok = v, True
                    if self.calls_mode:
                        self._close_all()                  # the window closed
                        self._break()
                    else:
                        self._on_ipi(v, True)
                else:                                              # IPC / IPR
                    sub = data & 3
                    n1 = _LEN1[(data >> 2) & 0xF]
                    d1 = (data >> 6) & ((1 << n1) - 1)
                    d2 = data >> (6 + n1)
                    if sub == 0:
                        st.ipc += 1
                    else:
                        st.ipr += 1
                    if comp and not self._ref_ip_ok:
                        self._lose()
                        continue
                    t = (d1 ^ self._ref_ip if comp else d1) & _M32
                    lnd = (d2 ^ self._ref_ip if comp else d2) & _M32
                    self._ref_ip, self._ref_ip_ok = lnd, True
                    if self.calls_mode:
                        self._pos += 1
                    self._on_ipcr(sub == 1, t & ~1, lnd & ~1)
            elif core == CORE_TSU and tt <= 1:                     # TSR
                if comp and not self._tsr_ok:
                    continue
                v = (data ^ self._tsr if comp else data) & _M32
                if self._tsr_ok and v < self._tsr and self._tsr - v > 1 << 31:
                    self._epoch += 1 << 32
                self._tsr, self._tsr_ok = v, True
                cyc = self._epoch + v
                if not self.anchors_cyc or cyc >= self.anchors_cyc[-1]:
                    seg = len(self.breaks)
                    if (self.ip is None and self._expect is None and self.breaks
                            and self.breaks[-1] == self._pos and not self._end_stamp):
                        seg -= 1          # right after the window closed: its end stamp
                        self._end_stamp = True
                    self.anchors_pos.append(self._pos)
                    self.anchors_cyc.append(cyc)
                    self.anchors_seg.append(seg)

    def finish(self) -> None:
        self._close_all()

    # -- results ------------------------------------------------------------------------

    def counts(self) -> np.ndarray:
        """Execution count of every instruction of the code model."""
        return np.cumsum(self._diff[:-1])

    def entered(self) -> np.ndarray:
        return self._entered

    def times_ns(self, positions, segments=None) -> np.ndarray:
        """Byte positions to nanoseconds from the first anchor.

        Time is continuous only within a segment (between breaks: a trace
        window ending, lost data).  Inside one the anchors are interpolated;
        before, after or between segments without anchors the time moves on
        at the trace's typical rate (emulation clocks per byte executed)."""
        pos = np.asarray(positions, dtype=np.float64)
        if not self.anchors_pos or not self.emu_hz:
            return pos                      # no time: one byte = one nanosecond keeps the order
        ap = np.asarray(self.anchors_pos, dtype=np.float64)
        ac = np.asarray(self.anchors_cyc, dtype=np.float64)
        aseg = np.asarray(self.anchors_seg, dtype=np.int64)
        same = (aseg[1:] == aseg[:-1]) & (ap[1:] > ap[:-1])
        if same.any():
            rate = float(np.median((ac[1:] - ac[:-1])[same] / (ap[1:] - ap[:-1])[same]))
        else:
            rate = 1.0
        if segments is None:
            segments = np.full(len(pos), int(aseg[-1]), dtype=np.int64)
            aseg = np.full(len(aseg), int(aseg[-1]), dtype=np.int64)
        seg = np.asarray(segments, dtype=np.int64)
        cyc = np.empty(len(pos), dtype=np.float64)
        order = np.argsort(seg, kind='stable')
        bounds = np.flatnonzero(np.diff(seg[order])) + 1
        for grp in np.split(order, bounds):
            if not len(grp):
                continue
            s = seg[grp[0]]
            a0, a1 = np.searchsorted(aseg, [s, s + 1])      # anchors are in segment order
            p = pos[grp]
            if a1 > a0:
                sp, sc = ap[a0:a1], ac[a0:a1]
                keep = np.append(sp[1:] != sp[:-1], True)      # one anchor per position
                sp, sc = sp[keep], sc[keep]
                c = np.interp(p, sp, sc)
                lo, hi = p < sp[0], p > sp[-1]
                c[lo] = sc[0] - (sp[0] - p[lo]) * rate
                c[hi] = sc[-1] + (p[hi] - sp[-1]) * rate
            else:
                k = np.searchsorted(aseg, s) - 1                # the last anchor before it
                k = max(k, 0)
                c = ac[k] + np.maximum(p - ap[k], 0.0) * rate
            cyc[grp] = c
        return (cyc - ac[0]) * (1e9 / self.emu_hz)


def detect_mode(records: list, sample: int = 64) -> str:
    """'calls' when the PTU messages are mostly IPC/IPR (subroutine mode),
    else 'flow'."""
    counts = {'ipi': 0, 'cr': 0}
    for rec in records[:sample]:
        for msg in _ptu_kinds(rec.payload):
            counts[msg] += 1
    return 'calls' if counts['cr'] > counts['ipi'] else 'flow'


def _ptu_kinds(par: bytes):
    from .tram import parse_paragraph
    for m in parse_paragraph(par):
        if m.kind == 'msg' and m.core == CORE_PTU:
            if 2 <= m.tt <= 3:
                yield 'ipi'
            elif m.tt >= 6:
                yield 'cr'


def decode_records(records: Iterable, code: cm.CodeModel, emu_hz: float = 0.0,
                   mode: str = 'auto') -> FlowDecoder:
    records = list(records)
    if mode == 'auto':
        mode = detect_mode(records)
    dec = FlowDecoder(code, emu_hz, mode)
    prev = None
    for rec in records:
        if rec.gap or (prev is not None and rec.seq != (prev + 1) & 0xFFFFFFFF) or rec.restart:
            if prev is not None:
                dec.gap(rec.restart)
        prev = rec.seq
        dec.paragraph(rec.payload)
    dec.finish()
    return dec
