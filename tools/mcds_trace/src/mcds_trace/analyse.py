"""Turn decoded events into per-variable series, CSV and plots."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field

from .decode import Event


@dataclass
class Series:
    name: str
    t: list = field(default_factory=list)       # seconds (or sample index)
    value: list = field(default_factory=list)   # None for pure timestamps
    kind: list = field(default_factory=list)    # 'write' / 'read' / 'hit'


def _slots(config: dict) -> list[dict]:
    out = []
    for j, s in enumerate(config.get('slots', [])):
        if not s.get('enabled'):
            continue
        addr = int(str(s.get('addr', '0')), 0)
        out.append({'index': j, 'name': s.get('name') or 'slot%d' % j,
                    'lo': addr, 'hi': addr + int(s.get('size', 4)) - 1})
    return out


def _signed(value: int, size: int) -> int:
    bits = 8 * size
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def build_series(events: list[Event], config: dict, symbols: dict | None = None,
                 signed: bool = False) -> dict[str, Series]:
    """Group events by variable.

    Full trace: by address; inside a slot range, by `symbols` (addr -> name)
    when given, else by exact address.  Compact trace: by watch-point id.
    """
    hz = config.get('emu_hz') or 0
    slots = _slots(config)
    series: dict[str, Series] = {}
    t0 = next((e.cycles for e in events if e.cycles >= 0), 0)

    def put(name: str, ev: Event, value, kind: str, idx: int) -> None:
        s = series.setdefault(name, Series(name))
        s.t.append((ev.cycles - t0) / hz if (hz and ev.cycles >= 0) else float(idx))
        s.value.append(value)
        s.kind.append(kind)

    for idx, ev in enumerate(events):
        if ev.kind in ('write', 'read'):
            value = ev.value
            if value is not None and signed and ev.size:
                value = _signed(value, ev.size)
            if ev.addr is None:
                cands = [s for s in slots]
                name = cands[0]['name'] if len(cands) == 1 else 'data'
            else:
                slot = next((s for s in slots if s['lo'] <= ev.addr <= s['hi']), None)
                if symbols and ev.addr in symbols:
                    name = symbols[ev.addr]
                elif slot and slot['hi'] - slot['lo'] < 4:
                    name = slot['name']                  # a single variable
                elif slot:
                    name = '%s+0x%X' % (slot['name'], ev.addr - slot['lo'])
                else:
                    name = '0x%08X' % ev.addr
            put(name, ev, value, ev.kind, idx)
        elif ev.kind == 'wps':
            slot = next((s for s in slots if s['index'] == ev.wp), None)
            put(slot['name'] if slot else 'wp%d' % ev.wp, ev, None, 'hit', idx)
        elif ev.kind == 'wpm':
            for j in range(8):
                if ev.wp & (1 << j):
                    slot = next((s for s in slots if s['index'] == j), None)
                    put(slot['name'] if slot else 'wp%d' % j, ev, None, 'hit', idx)
    return series


def write_csv(path: str, events: list[Event], config: dict) -> None:
    hz = config.get('emu_hz') or 0
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['cycles', 'seconds', 'kind', 'core', 'addr', 'value', 'size', 'wp', 'count'])
        for e in events:
            secs = e.cycles / hz if hz and e.cycles >= 0 else ''
            w.writerow([e.cycles, secs, e.kind, e.core,
                        '' if e.addr is None else '0x%08X' % e.addr,
                        '' if e.value is None else e.value, e.size or '',
                        '' if e.wp is None else e.wp, '' if e.count is None else e.count])


def plot(series: dict[str, Series], title: str = '', out: str | None = None,
         timed: bool = True) -> None:
    import matplotlib
    if out:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    names = sorted(series)
    if not names:
        print('nothing to plot')
        return
    fig, axes = plt.subplots(len(names), 2 if timed else 1, squeeze=False,
                             figsize=(13, 2.6 * len(names) + 0.8),
                             gridspec_kw={'width_ratios': [3, 1]} if timed else None)
    xlabel = 'time [s]' if timed else 'sample'
    for row, name in enumerate(names):
        s = series[name]
        t = np.asarray(s.t)
        ax = axes[row][0]
        if any(v is not None for v in s.value):
            v = np.asarray([np.nan if x is None else x for x in s.value], dtype=float)
            ax.step(t, v, where='post', lw=0.9)
            ax.plot(t, v, '.', ms=2)
            ax.set_ylabel('value')
        else:
            ax.eventplot(t, lineoffsets=0.5, linelengths=0.8, lw=0.6)
            ax.set_yticks([])
        ax.set_title('%s  (%d samples)' % (name, len(t)), fontsize=10, loc='left')
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.3)
        if timed:
            hax = axes[row][1]
            if len(t) > 1:
                dt = np.diff(t) * 1e6
                hax.hist(dt, bins=60)
                hax.set_xlabel('interval [us]')
                hax.set_title('median %.2f us' % np.median(dt), fontsize=9)
            hax.grid(alpha=0.3)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=110)
        print('plot written to', out)
    else:
        plt.show()
