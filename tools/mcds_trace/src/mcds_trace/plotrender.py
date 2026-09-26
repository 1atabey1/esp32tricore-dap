"""Off-screen plot rendering for the UI: one matplotlib figure per subplot,
drawn with Agg into RGBA frames the UI streams through RawImage.

Figures and line artists are kept between frames and only their data, limits
and size change, which keeps a frame at a few milliseconds.  Series longer
than the plot is wide are reduced to their min/max per pixel column, so a
million samples draw as fast as a thousand and no spike disappears.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_agg import FigureCanvasAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.ticker import EngFormatter  # noqa: E402
import numpy as np  # noqa: E402

THEMES = {
    'light': {'bg': '#ffffff', 'fg': '#333333', 'grid': '#e3e3e3', 'gap': '#e53935',
              'cursor': '#555555', 'spine': '#bbbbbb'},
    'dark': {'bg': '#1e1f22', 'fg': '#d7d7d7', 'grid': '#34363b', 'gap': '#ef5350',
             'cursor': '#bbbbbb', 'spine': '#55575c'},
}


def minmax_decimate(t: np.ndarray, v: np.ndarray, x0: float, x1: float,
                    bins: int) -> tuple[np.ndarray, np.ndarray]:
    """The samples of [x0, x1] (plus one each side), reduced to at most two per
    bin - each bin's minimum and maximum, in time order."""
    if len(t) == 0:
        return t, v
    i0 = max(int(np.searchsorted(t, x0, side='left')) - 1, 0)
    i1 = min(int(np.searchsorted(t, x1, side='right')) + 1, len(t))
    t, v = t[i0:i1], v[i0:i1]
    if len(t) <= 2 * bins or bins <= 0:
        return t, v
    edges = np.linspace(t[0], t[-1], bins + 1)
    idx = np.clip(np.searchsorted(t, edges[1:-1], side='left'), 0, len(t))
    starts = np.concatenate(([0], idx))
    ends = np.concatenate((idx, [len(t)]))
    keep = ends > starts
    starts, ends = starts[keep], ends[keep]
    finite = np.where(np.isnan(v), np.inf, v)
    lo = np.minimum.reduceat(finite, starts)
    hi = np.maximum.reduceat(np.where(np.isnan(v), -np.inf, v), starts)
    # Index of the min and max inside each bin, to keep time order.
    out_t, out_v = [], []
    for s, e, a, b in zip(starts.tolist(), ends.tolist(), lo.tolist(), hi.tolist()):
        seg_v = v[s:e]
        if not np.isfinite(a):
            out_t.append(t[s])
            out_v.append(np.nan)
            continue
        ia = s + int(np.nanargmin(seg_v))
        ib = s + int(np.nanargmax(seg_v))
        if ia == ib:
            out_t.append(t[ia]); out_v.append(v[ia])
        elif ia < ib:
            out_t += [t[ia], t[ib]]; out_v += [v[ia], v[ib]]
        else:
            out_t += [t[ib], t[ia]]; out_v += [v[ib], v[ia]]
    return np.asarray(out_t), np.asarray(out_v)


def value_at(t: np.ndarray, v: np.ndarray, x: float) -> float:
    """Sample-and-hold value at time x (the last sample at or before it)."""
    if len(t) == 0:
        return math.nan
    i = int(np.searchsorted(t, x, side='right')) - 1
    return float(v[i]) if i >= 0 else math.nan


@dataclass
class Trace:
    """What one signal contributes to a frame."""
    t: np.ndarray
    v: np.ndarray
    color: str
    kind: str = 'value'           # value | hits
    label: str = ''


@dataclass
class FrameSpec:
    width: int
    height: int
    x0: float
    x1: float
    traces: list[Trace] = field(default_factory=list)
    style: str = 'step'           # step | line | points
    ylim: tuple[float, float] | None = None     # None: fit the visible data
    gaps: list[float] = field(default_factory=list)
    cursor: float | None = None
    xlabel: bool = True
    theme: str = 'light'
    scale: float = 1.0            # device pixel ratio
    ylog: bool = False


@dataclass
class FrameInfo:
    """Where the axes landed in the frame, in logical pixels, for mapping
    pointer positions back to data."""
    left: float
    right: float
    top: float
    bottom: float
    x0: float
    x1: float
    y0: float
    y1: float

    def x_of(self, px: float) -> float:
        span = max(self.right - self.left, 1.0)
        return self.x0 + (px - self.left) / span * (self.x1 - self.x0)

    def y_of(self, py: float) -> float:
        span = max(self.bottom - self.top, 1.0)
        return self.y1 - (py - self.top) / span * (self.y1 - self.y0)

    def inside(self, px: float, py: float) -> bool:
        return self.left <= px <= self.right and self.top <= py <= self.bottom


class SubplotRenderer:
    """A persistent figure for one subplot.  Not thread-safe by itself; the
    lock serialises frames of the same subplot."""

    def __init__(self):
        self.lock = threading.Lock()
        self.fig = Figure(figsize=(6, 2), dpi=100)
        self.canvas = FigureCanvasAgg(self.fig)
        self.ax = self.fig.add_subplot(111)
        self._theme = None
        self._lines: list = []
        self._gap_art = None
        self._cursor_art = None
        self.info: FrameInfo | None = None

    def _apply_theme(self, name: str) -> None:
        if name == self._theme:
            return
        th = THEMES.get(name, THEMES['light'])
        self.fig.set_facecolor(th['bg'])
        ax = self.ax
        ax.set_facecolor(th['bg'])
        ax.tick_params(colors=th['fg'], labelsize=8)
        for sp in ax.spines.values():
            sp.set_color(th['spine'])
        ax.grid(True, color=th['grid'], linewidth=0.6)
        ax.xaxis.label.set_color(th['fg'])
        self._theme = name

    def render(self, spec: FrameSpec) -> tuple[bytes, int, int]:
        with self.lock:
            return self._render(spec)

    def _render(self, spec: FrameSpec) -> tuple[bytes, int, int]:
        w_px = max(int(spec.width * spec.scale), 40)
        h_px = max(int(spec.height * spec.scale), 40)
        dpi = 100 * spec.scale
        fig, ax = self.fig, self.ax
        fig.set_dpi(dpi)
        fig.set_size_inches(w_px / dpi, h_px / dpi)
        self._apply_theme(spec.theme)
        th = THEMES.get(spec.theme, THEMES['light'])

        # Artists: reuse lines, recreate collections (cheap).
        for ln in self._lines:
            ln.remove()
        self._lines = []
        for art in (self._gap_art, self._cursor_art):
            if art is not None:
                art.remove()
        self._gap_art = self._cursor_art = None

        x0, x1 = spec.x0, spec.x1
        if not x1 > x0:
            x1 = x0 + 1e-6
        bins = max(int(spec.width), 50)
        ymin, ymax = math.inf, -math.inf
        hit_rows = [tr for tr in spec.traces if tr.kind == 'hits']
        value_traces = [tr for tr in spec.traces if tr.kind != 'hits']
        for tr in value_traces:
            t, v = minmax_decimate(tr.t, tr.v, x0, x1, bins)
            if len(t) == 0:
                continue
            if spec.style == 'step' and t[-1] < x1 and np.isfinite(v[-1]):
                # Hold the last value to the right edge of the view.
                t = np.append(t, x1)
                v = np.append(v, v[-1])
            finite = np.isfinite(v)
            if spec.style == 'points':
                ln, = ax.plot(t[finite], v[finite], '.', ms=3, color=tr.color)
            elif spec.style == 'line':
                ln, = ax.plot(t, v, '-', lw=1.0, color=tr.color)
            else:
                ln, = ax.plot(t, v, '-', lw=1.0, color=tr.color, drawstyle='steps-post')
            self._lines.append(ln)
            if (~finite).any():
                # Accesses without a value (address-only payload): ticks at the bottom.
                ln2 = ax.plot(t[~finite], np.zeros((~finite).sum()), '|', ms=6, color=tr.color,
                              transform=ax.get_xaxis_transform())[0]
                self._lines.append(ln2)
            vis = v[finite & (t >= x0) & (t <= x1)]
            if len(vis) == 0 and finite.any():
                vis = v[finite][-1:]          # a held value from before the view
            if len(vis):
                ymin = min(ymin, float(np.min(vis)))
                ymax = max(ymax, float(np.max(vis)))

        # Watch-point hits: one row of ticks per signal.
        if hit_rows:
            n = len(hit_rows)
            for k, tr in enumerate(hit_rows):
                t, _ = minmax_decimate(tr.t, tr.v, x0, x1, bins * 2)
                t = t[(t >= x0) & (t <= x1)]
                ybase = (n - k - 1) / n
                ln = ax.vlines(t, ybase + 0.1 / n, ybase + 0.9 / n, colors=tr.color, lw=0.8,
                               transform=ax.get_xaxis_transform())
                self._lines.append(ln)
            if not value_traces:
                ymin, ymax = 0.0, 1.0

        if spec.gaps:
            g = [x for x in spec.gaps if x0 <= x <= x1]
            if g:
                self._gap_art = ax.vlines(g, 0, 1, colors=th['gap'], lw=0.8, alpha=0.6,
                                          linestyles='dashed', transform=ax.get_xaxis_transform())
        if spec.cursor is not None and x0 <= spec.cursor <= x1:
            self._cursor_art = ax.axvline(spec.cursor, color=th['cursor'], lw=0.8)

        if spec.ylim is not None:
            y0, y1 = spec.ylim
        elif math.isfinite(ymin):
            if ymax == ymin:
                pad = abs(ymin) * 0.05 or 1.0
            else:
                pad = (ymax - ymin) * 0.06
            y0, y1 = ymin - pad, ymax + pad
        else:
            y0, y1 = 0.0, 1.0
        ax.set_yscale('log' if spec.ylog and y0 > 0 else 'linear')
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        if hit_rows and not value_traces:
            ax.set_yticks([])
        else:
            ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(nbins=max(3, spec.height // 45)))
        ax.xaxis.set_major_formatter(EngFormatter(unit='s', places=None, sep=''))
        if abs(y0) >= 1e5 or abs(y1) >= 1e5:
            ax.yaxis.set_major_formatter(EngFormatter(sep=''))
        else:
            fmt = matplotlib.ticker.ScalarFormatter(useOffset=False)
            fmt.set_scientific(False)
            ax.yaxis.set_major_formatter(fmt)
        ax.tick_params(axis='x', labelbottom=spec.xlabel)
        ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(nbins=max(3, spec.width // 110)))

        # Fixed margins in logical pixels, so every subplot's axes line up.
        left, right = 64.0, 12.0
        top, bottom = 6.0, (22.0 if spec.xlabel else 6.0)
        fig.subplots_adjust(left=left / spec.width, right=1 - right / spec.width,
                            top=1 - top / spec.height, bottom=bottom / spec.height)
        self.canvas.draw()
        buf = self.canvas.buffer_rgba()
        w, h = self.canvas.get_width_height()
        self.info = FrameInfo(left, spec.width - right, top, spec.height - bottom,
                              x0, x1, y0, y1)
        return bytes(buf), w, h


def export_png(path: str, subplots: list[FrameSpec], title: str = '') -> None:
    """All subplots stacked into one PNG (a snapshot of the current view)."""
    if not subplots:
        return
    width = subplots[0].width
    total_h = sum(s.height for s in subplots)
    fig = Figure(figsize=(width / 100, total_h / 100 + (0.4 if title else 0)), dpi=150)
    FigureCanvasAgg(fig)
    axes = fig.subplots(len(subplots), 1, sharex=True, squeeze=False,
                        gridspec_kw={'height_ratios': [s.height for s in subplots]})
    for ax, spec in zip(axes[:, 0], subplots):
        for tr in spec.traces:
            t, v = minmax_decimate(tr.t, tr.v, spec.x0, spec.x1, 4000)
            if tr.kind == 'hits':
                ax.vlines(t, 0, 1, colors=tr.color, lw=0.6, label=tr.label,
                          transform=ax.get_xaxis_transform())
            elif spec.style == 'points':
                ax.plot(t, v, '.', ms=2, color=tr.color, label=tr.label)
            else:
                ax.plot(t, v, '-', lw=0.9, color=tr.color, label=tr.label,
                        drawstyle='steps-post' if spec.style == 'step' else 'default')
        if spec.ylim:
            ax.set_ylim(*spec.ylim)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper left', fontsize=7, framealpha=0.7)
    axes[-1, 0].set_xlim(subplots[0].x0, subplots[0].x1)
    axes[-1, 0].xaxis.set_major_formatter(EngFormatter(unit='s', sep=''))
    if title:
        fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
