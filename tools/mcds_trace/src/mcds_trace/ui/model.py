"""UI state that outlives a session: settings, and the workspace (which
signals are selected, how they are plotted, how the trace is configured)."""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field

from .. import elfsyms
from ..signals import PALETTE, Signal, SlotPlan, hits_signal, leaf_signal, plan_slots, raw_signal


def config_dir() -> str:
    if sys.platform == 'win32':
        base = os.environ.get('APPDATA') or os.path.expanduser('~')
    elif sys.platform == 'darwin':
        base = os.path.expanduser('~/Library/Application Support')
    else:
        base = os.environ.get('XDG_CONFIG_HOME') or os.path.expanduser('~/.config')
    d = os.path.join(base, 'mcds-trace')
    os.makedirs(d, exist_ok=True)
    return d


def default_capture_dir() -> str:
    d = os.path.join(os.path.expanduser('~'), 'mcds-captures')
    return d


@dataclass
class Settings:
    host: str = '192.168.178.99'
    auth: str = 'admin:admin'
    capture_dir: str = field(default_factory=default_capture_dir)
    history_s: float = 600.0          # live: seconds of samples kept in memory
    window_s: float = 5.0             # live: visible time span while following
    theme: str = 'light'
    fps: int = 12
    last_elf: str = ''
    last_workspace: str = ''
    recent_captures: list = field(default_factory=list)

    @classmethod
    def load(cls) -> 'Settings':
        path = os.path.join(config_dir(), 'settings.json')
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return cls()
        s = cls()
        for k, v in data.items():
            if hasattr(s, k):
                setattr(s, k, v)
        return s

    def save(self) -> None:
        path = os.path.join(config_dir(), 'settings.json')
        tmp = path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                json.dump(asdict(self), f, indent=1)
            os.replace(tmp, path)
        except OSError:
            pass

    def add_recent(self, path: str) -> None:
        r = [p for p in self.recent_captures if p != path]
        self.recent_captures = [path] + r[:9]


# TC3xx global scratchpad segments: 0x7 CPU0, 0x6 CPU1, 0x5 CPU2, 0x4 CPU3,
# 0x3 CPU4, 0x1 CPU5 (DSPR at +0, PSPR at +0x100000).
_SEGMENT_CPU = {0x7: 0, 0x6: 1, 0x5: 2, 0x4: 3, 0x3: 4, 0x1: 5}


def owner_cpu(addrs: list[int], default: int = 0) -> int:
    """The CPU whose scratchpad holds these addresses (the first that tells)."""
    for a in addrs:
        cpu = _SEGMENT_CPU.get((a >> 28) & 0xF)
        if cpu is not None and (a & 0x0FE00000) == 0:
            return cpu
    return default


@dataclass
class SlotFilter:
    enabled: bool = False
    lo: int = 0
    hi: int = 0xFFFFFFFF
    mask: int = 0xFFFFFFFF
    signed: bool = False


@dataclass
class CaptureOptions:
    source: str = 'cpu'               # cpu | memslave | lmu0
    cpu: int = -1                     # -1: the CPU whose scratchpad holds the signals
    mode: str = 'full'                # full | compact
    payload: str = 'addr_data'        # addr_data | data | addr
    timestamps: str = 'hit'           # hit | ticks | none
    access: str = 'w'                 # w | r | rw
    masters: bool = False
    dap_div: int = 0                  # 0: 24 MHz
    wide: bool = True
    duration: float = 0.0             # stop a live trace after this many seconds (0: manual)
    filters: list = field(default_factory=lambda: [SlotFilter(), SlotFilter()])

    def to_json(self) -> dict:
        d = asdict(self)
        return d

    @classmethod
    def from_json(cls, d: dict) -> 'CaptureOptions':
        c = cls()
        for k, v in (d or {}).items():
            if k == 'filters':
                c.filters = [SlotFilter(**f) if isinstance(f, dict) else SlotFilter() for f in v][:2]
                while len(c.filters) < 2:
                    c.filters.append(SlotFilter())
            elif hasattr(c, k):
                setattr(c, k, v)
        return c


@dataclass
class SubplotModel:
    id: int
    signals: list = field(default_factory=list)      # signal ids
    style: str = 'step'                              # step | line | points
    ylim: list | None = None                         # [lo, hi] or None (auto)
    height: int = 220
    ylog: bool = False
    normalize: bool = False                          # each signal scaled to 0..1 in view
    stats: bool = False                              # statistics row under the legend


class Workspace:
    """Selected signals, their plots and the capture options."""

    def __init__(self):
        self.signals: dict[str, Signal] = {}
        self.subplots: list[SubplotModel] = []
        self.capture = CaptureOptions()
        self._next_plot = 1

    # signals -----------------------------------------------------------------

    def next_color(self) -> str:
        used = {s.color for s in self.signals.values()}
        for c in PALETTE:
            if c not in used:
                return c
        return PALETTE[len(self.signals) % len(PALETTE)]

    def add_signal(self, sig: Signal, plot: int | None = None) -> Signal:
        """Add (or return the existing) signal; plot=-1 puts it on a new plot,
        None on the last plot (or a new one if there is none)."""
        if sig.id in self.signals:
            sig = self.signals[sig.id]
        else:
            sig.color = self.next_color()
            self.signals[sig.id] = sig
        if plot == -1 or not self.subplots:
            sp = self.new_plot()
        elif plot is None:
            sp = self.subplots[-1]
        else:
            sp = self.plot(plot) or self.new_plot()
        if sig.id not in sp.signals:
            sp.signals.append(sig.id)
        return sig

    def remove_signal(self, sid: str) -> None:
        self.signals.pop(sid, None)
        for sp in self.subplots:
            if sid in sp.signals:
                sp.signals.remove(sid)

    def plot(self, pid: int) -> SubplotModel | None:
        return next((p for p in self.subplots if p.id == pid), None)

    def new_plot(self) -> SubplotModel:
        sp = SubplotModel(self._next_plot)
        self._next_plot += 1
        self.subplots.append(sp)
        return sp

    def remove_plot(self, pid: int) -> None:
        self.subplots = [p for p in self.subplots if p.id != pid]

    def split_plot(self, pid: int) -> None:
        """One plot per signal of this plot (the first keeps the plot)."""
        i = next((k for k, p in enumerate(self.subplots) if p.id == pid), None)
        if i is None:
            return
        sp = self.subplots[i]
        rest = sp.signals[1:]
        sp.signals = sp.signals[:1]
        for k, sid in enumerate(rest):
            new = SubplotModel(self._next_plot, [sid], sp.style, None, sp.height)
            self._next_plot += 1
            self.subplots.insert(i + 1 + k, new)

    def merge_up(self, pid: int) -> None:
        """Move this plot's signals into the plot above it."""
        i = next((k for k, p in enumerate(self.subplots) if p.id == pid), None)
        if not i:
            return
        up = self.subplots[i - 1]
        for sid in self.subplots[i].signals:
            if sid not in up.signals:
                up.signals.append(sid)
        del self.subplots[i]

    def move_plot(self, pid: int, delta: int) -> None:
        i = next((k for k, p in enumerate(self.subplots) if p.id == pid), None)
        if i is None:
            return
        j = max(0, min(len(self.subplots) - 1, i + delta))
        self.subplots.insert(j, self.subplots.pop(i))

    def plotted(self) -> list[Signal]:
        seen = []
        for sp in self.subplots:
            for sid in sp.signals:
                s = self.signals.get(sid)
                if s is not None and s not in seen:
                    seen.append(s)
        return seen

    # tracing -----------------------------------------------------------------

    def value_signals(self) -> list[Signal]:
        return [s for s in self.signals.values() if s.kind == 'value']

    def plan(self) -> SlotPlan:
        return plan_slots(self.value_signals())

    def probe_config(self) -> tuple[dict, SlotPlan, list[str]]:
        """The configuration to POST, its slot plan, and warnings for the user."""
        c = self.capture
        plan = self.plan()
        warnings = []
        names = []
        for ids in plan.signals:
            labels = [self.signals[i].label for i in ids if i in self.signals]
            names.append(labels[0] if len(labels) == 1 else '%s+%d' % (labels[0], len(labels) - 1)
                         if labels else 'slot')
        slots = plan.slots(c.access, names)
        for j, s in enumerate(slots):
            f = c.filters[j] if j < len(c.filters) else SlotFilter()
            # Only where the Capture tab offers it: a slot of one signal, in
            # full mode.  A filter left from another layout must not act.
            single = j < len(plan.signals) and len(plan.signals[j]) == 1
            if s.get('enabled') and f.enabled and single and c.mode == 'full':
                s['value'] = {'enabled': True, 'lo': '0x%X' % (f.lo & 0xFFFFFFFF),
                              'hi': '0x%X' % (f.hi & 0xFFFFFFFF), 'mask': '0x%X' % f.mask,
                              'signed': f.signed}
        if not plan.ranges:
            warnings.append('Select at least one signal to trace.')
        if c.payload == 'data' and any(len(ids) > 1 for ids in plan.signals):
            warnings.append('Data-only payload: accesses carry no address, so a slot with '
                            'several signals cannot tell them apart.')
        if c.mode == 'compact':
            warnings.append('Compact mode records hit times per slot only, no values.')
        if plan.extra_bytes > 256:
            warnings.append('The watched ranges include %d bytes of other variables; their '
                            'accesses cost trace bandwidth.' % plan.extra_bytes)
        cpu = c.cpu if c.cpu >= 0 else owner_cpu([a for a, _ in plan.ranges])
        cfg = {'source': c.source, 'cpu': cpu, 'mode': c.mode, 'payload': c.payload,
               'timestamps': c.timestamps, 'masters': c.masters, 'dap_div': c.dap_div,
               'wide': c.wide, 'slots': slots}
        return cfg, plan, warnings

    def trace_signals(self, plan: SlotPlan) -> list[Signal]:
        """What the extractor produces: value signals, or per-slot hits in
        compact mode."""
        if self.capture.mode == 'compact':
            out = []
            for j, ids in enumerate(plan.signals):
                labels = [self.signals[i].label for i in ids if i in self.signals]
                s = hits_signal(j, ' / '.join(labels)[:60] or 'slot %d' % j)
                s.color = self.signals[ids[0]].color if ids and ids[0] in self.signals else PALETTE[j]
                out.append(s)
            return out
        return self.value_signals()

    # persistence ---------------------------------------------------------------

    def to_json(self, elf_path: str = '') -> dict:
        return {'format': 'mcds-trace-workspace', 'version': 1, 'elf': elf_path,
                'signals': [s.to_json() for s in self.signals.values()],
                'subplots': [asdict(p) for p in self.subplots],
                'capture': self.capture.to_json()}

    @classmethod
    def from_json(cls, d: dict, table: elfsyms.SymbolTable | None) -> tuple['Workspace', list[str]]:
        ws = cls()
        missing = []
        ws.capture = CaptureOptions.from_json(d.get('capture', {}))
        for sd in d.get('signals', []):
            sid = sd.get('id', '')
            sig = None
            if sd.get('kind') == 'hits':
                sig = hits_signal(int(sd.get('slot', 0)), sd.get('label', sid))
            elif sid.startswith('raw:'):
                sig = raw_signal(int(str(sd['addr']), 0), int(sd['size']), sd.get('label'),
                                 bool(sd.get('signed')))
            elif table is not None:
                node = elfsyms.resolve_path(table.variables, sid)
                if node is not None and node.is_leaf:
                    sig = leaf_signal(node)
            elif sd.get('addr') is not None and sd.get('size'):
                # No ELF yet: keep it by its saved address (plain unsigned
                # until an ELF resolves it again).
                try:
                    sig = Signal(sid, sd.get('label') or sid, int(str(sd['addr']), 0),
                                 int(sd['size']))
                except (TypeError, ValueError):
                    sig = None
            if sig is None:
                missing.append(sid)
                continue
            sig.color = sd.get('color') or ws.next_color()
            if sd.get('label'):
                sig.label = sd['label']
            try:
                gain, offset = float(sd.get('gain', 1.0)), float(sd.get('offset', 0.0))
                if math.isfinite(gain) and math.isfinite(offset) and gain != 0:
                    sig.gain, sig.offset = gain, offset
            except (TypeError, ValueError):
                pass
            sig.unit = str(sd.get('unit', ''))[:16]
            ws.signals[sig.id] = sig
        for pd in d.get('subplots', []):
            sp = SubplotModel(int(pd.get('id', ws._next_plot)),
                              [i for i in pd.get('signals', []) if i in ws.signals],
                              pd.get('style', 'step'), pd.get('ylim'), int(pd.get('height', 220)),
                              bool(pd.get('ylog', False)), bool(pd.get('normalize', False)),
                              bool(pd.get('stats', False)))
            ws.subplots.append(sp)
            ws._next_plot = max(ws._next_plot, sp.id + 1)
        return ws, missing
