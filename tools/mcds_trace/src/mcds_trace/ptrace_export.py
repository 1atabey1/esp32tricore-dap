"""Program trace results: code coverage reports and Perfetto traces.

Coverage: per instruction execution counts (ptrace.FlowDecoder.counts()),
rolled up per function and per source line/file.  Written as a text summary,
an lcov tracefile (genhtml, VS Code Coverage Gutters, ...) and a single-file
HTML overview.  A subroutine-mode trace has no per-instruction flow; its
coverage is functions entered.

Perfetto: the call view (slices per function call, interrupts and traps on
the same track) as a native Perfetto protobuf trace (TrackEvent, interned
names), or as Chrome JSON; ui.perfetto.dev opens both.
"""

from __future__ import annotations

import html
import json
import os
from dataclasses import dataclass

import numpy as np

from . import codemodel as cm
from .ptrace import EV_BEGIN, EV_END, FlowDecoder


# -- coverage ---------------------------------------------------------------------------

@dataclass
class FuncCov:
    name: str
    start: int
    insns: int
    hit: int
    count: int            # calls (and interrupt/trap entries) into it
    file: str
    line: int

    @property
    def pct(self) -> float:
        return 100.0 * self.hit / self.insns if self.insns else 0.0


@dataclass
class FileCov:
    path: str
    lines: dict           # line -> max execution count of its instructions
    funcs: list           # FuncCov

    @property
    def lf(self) -> int:
        return len(self.lines)

    @property
    def lh(self) -> int:
        return sum(1 for v in self.lines.values() if v > 0)


@dataclass
class Coverage:
    funcs: list
    files: list
    insns: int
    hit: int
    function_level: bool   # nesting mode: functions entered only
    window: tuple | None

    @property
    def pct(self) -> float:
        return 100.0 * self.hit / self.insns if self.insns else 0.0


def coverage(dec: FlowDecoder, window: tuple[int, int] | None = None) -> Coverage:
    """Roll the decoder's counts up.  `window` (lo, hi inclusive) restricts the
    report to the code a range-qualified trace could see."""
    code = dec.code
    counts = dec.counts()
    entered = dec.entered()
    function_level = dec.stats.bytes_executed == 0 and int(entered.sum()) > 0
    addr = code.addr
    funcs, by_file = [], {}
    tot_i = tot_h = 0
    for k, name in enumerate(code.func_name):
        a0, a1 = int(code.func_start[k]), int(code.func_end[k])
        if window and (a1 <= window[0] or a0 > window[1]):
            continue
        i0, i1 = np.searchsorted(addr, [a0, a1])
        n = int(i1 - i0)
        if n <= 0:
            continue
        c = counts[i0:i1]
        if function_level:
            hit = n if entered[k] else 0
            cnt = int(entered[k])
        else:
            hit = int(np.count_nonzero(c))
            cnt = int(entered[k])
        src = code.source_of(int(i0))
        f = FuncCov(name, a0, n, hit, cnt, src[0] if src else '', src[1] if src else 0)
        funcs.append(f)
        tot_i += n
        tot_h += hit
        if src:
            fc = by_file.get(src[0])
            if fc is None:
                fc = by_file[src[0]] = FileCov(src[0], {}, [])
            fc.funcs.append(f)
            for i in range(int(i0), int(i1)):
                s = code.source_of(i)
                if s is None or s[0] != src[0]:
                    continue
                v = cnt if function_level else int(counts[i])
                if v > fc.lines.get(s[1], -1):
                    fc.lines[s[1]] = v
    files = sorted(by_file.values(), key=lambda f: f.path)
    return Coverage(funcs, files, tot_i, tot_h, function_level, window)


def summary_text(cov: Coverage, top: int = 25) -> str:
    unit = 'functions entered' if cov.function_level else 'instructions executed'
    nf = len(cov.funcs)
    nfh = sum(1 for f in cov.funcs if f.hit)
    out = ['coverage: %.1f %% of %d %s; %d of %d functions reached'
           % (cov.pct, cov.insns, unit if not cov.function_level else 'instructions (by function)',
              nfh, nf)]
    hot = sorted((f for f in cov.funcs if f.hit), key=lambda f: -f.count)[:top]
    if hot:
        out.append('most entered:')
        for f in hot:
            out.append('  %10d  %5.1f %%  %s' % (f.count, f.pct, f.name))
    return '\n'.join(out)


def hot_loops(dec: FlowDecoder, top: int = 6) -> list[tuple[int, int, int, str]]:
    """The most executed backward branches as loops: (lo, hi inclusive, count,
    function), the candidates for tdm_dap.py trace --exclude."""
    code = dec.code
    counts = dec.counts()
    back = (code.target != 0) & (code.target <= code.addr) & np.isin(code.kind, (cm.K_JUMP, cm.K_COND))
    idx = np.nonzero(back & (counts > 0))[0]
    idx = idx[np.argsort(-counts[idx])][:top]
    out = []
    for i in idx:
        lo = int(code.target[i])
        hi = int(code.addr[i]) + int(code.size[i]) - 1
        out.append((lo, hi, int(counts[i]), code.function_name(lo)))
    return out


def write_lcov(cov: Coverage, path: str, test_name: str = 'mcds_ptrace') -> None:
    with open(path, 'w', encoding='utf-8') as f:
        f.write('TN:%s\n' % test_name)
        for fc in cov.files:
            f.write('SF:%s\n' % fc.path)
            for fn in fc.funcs:
                f.write('FN:%d,%s\n' % (fn.line, fn.name))
            for fn in fc.funcs:
                f.write('FNDA:%d,%s\n' % (max(fn.count, 1) if fn.hit else 0, fn.name))
            f.write('FNF:%d\nFNH:%d\n' % (len(fc.funcs), sum(1 for x in fc.funcs if x.hit)))
            for line in sorted(fc.lines):
                f.write('DA:%d,%d\n' % (line, max(fc.lines[line], 0)))
            f.write('LF:%d\nLH:%d\nend_of_record\n' % (fc.lf, fc.lh))


_CSS = """
:root{--bg:#fbfbfa;--fg:#1d1d1f;--mut:#6b6b70;--line:#e3e3e0;--bar:#e7e7e4;--hi:#2f8f5b;
--mid:#c08a1e;--lo:#c2453d;--card:#fff}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#17181a;--fg:#ececee;
--mut:#9a9aa1;--line:#2c2d31;--bar:#2a2b2f;--card:#1f2023}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif}
main{max-width:1200px;margin:0 auto;padding:24px 16px}
h1{font-size:20px;margin:0 0 4px}.mut{color:var(--mut)}
.tiles{display:flex;gap:12px;flex-wrap:wrap;margin:16px 0}
.tile{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 14px;min-width:150px}
.tile b{display:block;font-size:22px}
input{width:100%;box-sizing:border-box;padding:8px;border:1px solid var(--line);border-radius:6px;
background:var(--card);color:var(--fg);margin:8px 0}
table{width:100%;border-collapse:collapse;background:var(--card);border:1px solid var(--line)}
th,td{text-align:left;padding:4px 8px;border-bottom:1px solid var(--line);white-space:nowrap}
th{cursor:pointer;user-select:none;position:sticky;top:0;background:var(--card)}
td.n{text-align:right;font-variant-numeric:tabular-nums}
td.name{white-space:normal;word-break:break-all}
.bar{display:inline-block;width:90px;height:8px;background:var(--bar);border-radius:4px;
vertical-align:middle;margin-right:6px;overflow:hidden}.bar i{display:block;height:100%}
.tabs button{border:1px solid var(--line);background:var(--card);color:var(--fg);padding:6px 12px;
border-radius:6px;cursor:pointer}.tabs button.on{border-color:var(--fg)}
.wrap{overflow-x:auto}
"""

_JS = """
const q=document.getElementById('q'),only=document.getElementById('only');
function filt(){const s=q.value.toLowerCase(),o=only.checked;for(const t of document.querySelectorAll('tbody'))
for(const r of t.rows){const hit=+r.dataset.hit>0;r.style.display=(r.textContent.toLowerCase().includes(s)&&(!o||hit))?'':'none'}}
q.oninput=filt;only.onchange=filt;
for(const th of document.querySelectorAll('th'))th.onclick=()=>{const tb=th.closest('table').tBodies[0],
i=[...th.parentNode.children].indexOf(th),num=th.dataset.num!==undefined,d=th.dataset.d=th.dataset.d=='1'?'0':'1';
const rows=[...tb.rows].sort((a,b)=>{let x=a.cells[i].dataset.v??a.cells[i].textContent,y=b.cells[i].dataset.v??b.cells[i].textContent;
if(num){x=+x;y=+y}return (x<y?-1:x>y?1:0)*(d=='1'?-1:1)});rows.forEach(r=>tb.appendChild(r))};
for(const b of document.querySelectorAll('.tabs button'))b.onclick=()=>{for(const x of document.querySelectorAll('.tabs button'))
x.classList.toggle('on',x==b);for(const s of document.querySelectorAll('section.view'))s.hidden=s.id!=b.dataset.v};
"""


def _bar(pct: float) -> str:
    col = 'var(--hi)' if pct >= 80 else 'var(--mid)' if pct >= 40 else 'var(--lo)'
    return '<span class="bar"><i style="width:%.1f%%;background:%s"></i></span>%.1f %%' % (pct, col, pct)


def write_html(cov: Coverage, path: str, title: str, stats_text: str = '') -> None:
    nf = len(cov.funcs)
    nfh = sum(1 for f in cov.funcs if f.hit)
    lf = sum(f.lf for f in cov.files)
    lh = sum(f.lh for f in cov.files)
    e = html.escape
    rows_f = []
    for fc in cov.files:
        p = 100.0 * fc.lh / fc.lf if fc.lf else 0.0
        fh = sum(1 for x in fc.funcs if x.hit)
        rows_f.append('<tr data-hit="%d"><td class="name">%s</td><td class="n" data-v="%.3f">%s</td>'
                      '<td class="n">%d/%d</td><td class="n" data-v="%d">%d/%d</td></tr>'
                      % (fc.lh, e(fc.path), p, _bar(p), fc.lh, fc.lf, fh, fh, len(fc.funcs)))
    rows_fn = []
    for f in cov.funcs:
        loc = '%s:%d' % (os.path.basename(f.file), f.line) if f.file else ''
        rows_fn.append('<tr data-hit="%d"><td class="name">%s</td><td class="n" data-v="%.3f">%s</td>'
                       '<td class="n">%d/%d</td><td class="n">%d</td><td class="n">0x%08x</td><td>%s</td></tr>'
                       % (f.hit, e(f.name), f.pct, _bar(f.pct), f.hit, f.insns, f.count, f.start, e(loc)))
    unit = 'by function entry' if cov.function_level else 'by instruction'
    win = ('<p class="mut">Window 0x%08x..0x%08x</p>' % cov.window) if cov.window else ''
    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{e(title)}</title>
<style>{_CSS}</style></head><body><main>
<h1>{e(title)}</h1><p class="mut">Program trace coverage ({unit}): what the trace saw execute.</p>{win}
<div class="tiles">
<div class="tile"><span class="mut">Instructions</span><b>{cov.pct:.1f} %</b><span class="mut">{cov.hit} / {cov.insns}</span></div>
<div class="tile"><span class="mut">Functions reached</span><b>{100.0 * nfh / nf if nf else 0:.1f} %</b><span class="mut">{nfh} / {nf}</span></div>
<div class="tile"><span class="mut">Source lines</span><b>{100.0 * lh / lf if lf else 0:.1f} %</b><span class="mut">{lh} / {lf}</span></div>
</div>
<pre class="mut" style="white-space:pre-wrap">{e(stats_text)}</pre>
<div class="tabs"><button class="on" data-v="vf">Files</button> <button data-v="vfn">Functions</button></div>
<input id="q" placeholder="Filter by name"><label><input type="checkbox" id="only" style="width:auto"> reached only</label>
<section class="view" id="vf"><div class="wrap"><table><thead><tr><th>File</th><th data-num>Lines</th><th>Hit</th><th data-num>Functions</th></tr></thead>
<tbody>{''.join(rows_f)}</tbody></table></div></section>
<section class="view" id="vfn" hidden><div class="wrap"><table><thead><tr><th>Function</th><th data-num>Coverage</th><th>Instr.</th>
<th data-num>Entries</th><th>Address</th><th>Source</th></tr></thead><tbody>{''.join(rows_fn)}</tbody></table></div></section>
</main><script>{_JS}</script></body></html>"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(doc)


# -- Perfetto ------------------------------------------------------------------------------

def _varint(v: int) -> bytes:
    if v < 0:
        raise ValueError('varint of a negative value: %d' % v)
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _f_varint(num: int, v: int) -> bytes:
    return _varint(num << 3) + _varint(v)


def _f_bytes(num: int, b: bytes) -> bytes:
    return _varint((num << 3) | 2) + _varint(len(b)) + b


def _f_str(num: int, s: str) -> bytes:
    return _f_bytes(num, s.encode('utf-8'))


# TracePacket / TrackEvent / TrackDescriptor field numbers (perfetto protos)
_TP_TIMESTAMP, _TP_SEQ_ID, _TP_TRACK_EVENT, _TP_INTERNED, _TP_SEQ_FLAGS, _TP_TRACK_DESC = 8, 10, 11, 12, 13, 60
_TE_TYPE, _TE_NAME_IID, _TE_TRACK_UUID = 9, 10, 11
_TD_UUID, _TD_NAME, _TD_PROCESS, _TD_THREAD = 1, 2, 3, 4
_SLICE_BEGIN, _SLICE_END = 1, 2
_SEQ_CLEARED, _SEQ_NEEDS = 1, 2
_SEQ = 1


def _slices(dec: FlowDecoder):
    """(t_ns, kind, name id) in order, ends never before their begins."""
    if not dec.events:
        return []
    pos = [p for p, _, _ in dec.events]
    t = dec.times_ns(pos, dec.event_segs)
    # in order, and nothing before the first time stamp (extrapolated back)
    t = np.maximum.accumulate(np.maximum(np.asarray(t, dtype=np.float64), 0.0))
    return [(int(t[k]), ev[1], ev[2]) for k, ev in enumerate(dec.events)]


def write_perfetto(dec: FlowDecoder, path: str, cpu: int, device: str = 'TC38x') -> int:
    """A Perfetto protobuf trace of the call view; returns the slice count."""
    pid, tid = 1, cpu + 1
    track = 0x7C000 + cpu
    out = bytearray()

    def packet(body: bytes) -> None:
        out.extend(_f_bytes(1, body))

    packet(_f_bytes(_TP_TRACK_DESC, _f_varint(_TD_UUID, 0x7C100) +
                    _f_bytes(_TD_PROCESS, _f_varint(1, pid) + _f_str(6, device))))
    packet(_f_bytes(_TP_TRACK_DESC, _f_varint(_TD_UUID, track) + _f_str(_TD_NAME, 'CPU%d' % cpu) +
                    _f_bytes(_TD_THREAD, _f_varint(1, pid) + _f_varint(2, tid) +
                             _f_str(5, 'CPU%d' % cpu))))
    interned = set()
    first = True
    n = 0
    for t, kind, nid in _slices(dec):
        te = _f_varint(_TE_TYPE, _SLICE_BEGIN if kind == EV_BEGIN else _SLICE_END)
        te += _f_varint(_TE_TRACK_UUID, track)
        body = _f_varint(_TP_TIMESTAMP, t) + _f_varint(_TP_SEQ_ID, _SEQ)
        if kind == EV_BEGIN:
            iid = nid + 1
            te += _f_varint(_TE_NAME_IID, iid)
            if iid not in interned:
                interned.add(iid)
                body += _f_bytes(_TP_INTERNED, _f_bytes(2, _f_varint(1, iid) +
                                                        _f_str(2, dec.names[nid])))
            n += 1
        body += _f_varint(_TP_SEQ_FLAGS, _SEQ_CLEARED if first else _SEQ_NEEDS)
        first = False
        packet(body + _f_bytes(_TP_TRACK_EVENT, te))
    with open(path, 'wb') as f:
        f.write(out)
    return n


def write_chrome_json(dec: FlowDecoder, path: str, cpu: int, device: str = 'TC38x') -> int:
    pid, tid = 1, cpu + 1
    ev = [{'name': 'process_name', 'ph': 'M', 'pid': pid, 'args': {'name': device}},
          {'name': 'thread_name', 'ph': 'M', 'pid': pid, 'tid': tid, 'args': {'name': 'CPU%d' % cpu}}]
    n = 0
    for t, kind, nid in _slices(dec):
        if kind == EV_BEGIN:
            ev.append({'name': dec.names[nid], 'ph': 'B', 'ts': t / 1000.0, 'pid': pid, 'tid': tid})
            n += 1
        else:
            ev.append({'ph': 'E', 'ts': t / 1000.0, 'pid': pid, 'tid': tid})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'traceEvents': ev, 'displayTimeUnit': 'ns'}, f, separators=(',', ':'))
    return n
