"""Off-screen plot rendering for the UI: RGBA frames the UI streams through
RawImage, drawn directly with Pillow (a frame of a few dozen traces takes a
few milliseconds, so hovering, panning and live follow stay fluid).  Tick
positions come from matplotlib's locators; exported PNGs use matplotlib.

Series longer than the plot is wide are reduced to their min/max per pixel
column, so a million samples draw as fast as a thousand and no spike
disappears.
"""

from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass, field

import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_agg import FigureCanvasAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.ticker import EngFormatter, LogLocator, MaxNLocator  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

THEMES = {
    'light': {'bg': '#ffffff', 'fg': '#333333', 'grid': '#e3e3e3', 'gap': '#e53935',
              'cursor': '#555555', 'spine': '#bbbbbb', 'marker': '#8e24aa'},
    'dark': {'bg': '#1e1f22', 'fg': '#d7d7d7', 'grid': '#34363b', 'gap': '#ef5350',
             'cursor': '#bbbbbb', 'spine': '#55575c', 'marker': '#ce93d8'},
}

# Layout in logical pixels: fixed margins so every subplot's axes line up.
MARGIN_LEFT, MARGIN_RIGHT, MARGIN_TOP = 64.0, 12.0, 6.0
MARGIN_BOTTOM, MARGIN_BOTTOM_BARE = 22.0, 6.0
PT = 100 / 72                          # logical pixels per point (matplotlib's 100 dpi)

_FONT_PATH = os.path.join(matplotlib.get_data_path(), 'fonts', 'ttf', 'DejaVuSans.ttf')
_fonts: dict[int, ImageFont.FreeTypeFont] = {}
_MINUS = '−'
_PREFIX = {-24: 'y', -21: 'z', -18: 'a', -15: 'f', -12: 'p', -9: 'n', -6: 'µ', -3: 'm',
           0: '', 3: 'k', 6: 'M', 9: 'G', 12: 'T', 15: 'P', 18: 'E', 21: 'Z', 24: 'Y'}


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
    nan = np.isnan(v)
    if nan.any():
        vmin = np.where(nan, np.inf, v)
        vmax = np.where(nan, -np.inf, v)
    else:
        vmin = vmax = v
    lo = np.minimum.reduceat(vmin, starts)
    hi = np.maximum.reduceat(vmax, starts)
    # Index of the first minimum and first maximum inside each bin, so the
    # pair keeps its time order.
    n = len(t)
    bin_of = np.repeat(np.arange(len(starts)), ends - starts)
    ar = np.arange(n)
    ia = np.minimum.reduceat(np.where(vmin == lo[bin_of], ar, n), starts)
    ib = np.minimum.reduceat(np.where(vmax == hi[bin_of], ar, n), starts)
    empty = ~np.isfinite(lo)                  # nothing but NaN (or -inf) in the bin
    ia[empty] = ib[empty] = starts[empty]
    first, second = np.minimum(ia, ib), np.maximum(ia, ib)
    pick = np.stack([first, np.where(first != second, second, -1)], axis=1).ravel()
    blank = np.repeat(empty, 2)[pick >= 0]
    pick = pick[pick >= 0]
    out_v = v[pick].astype(float)
    out_v[blank] = np.nan
    return t[pick], out_v


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
    t_end: float | None = None    # data ends here: hold step values no further
    marker: float | None = None   # reference time for delta measurements


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


# -- tick labels ---------------------------------------------------------------

def eng_label(v: float, unit: str = '') -> str:
    """matplotlib's EngFormatter(places=None, sep='') for one value."""
    if v == 0 or not math.isfinite(v):
        return ('0' if v == 0 else str(v)) + unit
    sign = _MINUS if v < 0 else ''
    v = abs(v)
    p = max(-24, min(24, int(math.floor(math.log10(v) / 3)) * 3))
    mant = v / 10.0 ** p
    if abs(float('%g' % mant)) >= 1000 and p < 24:
        mant /= 1000
        p += 3
    return '%s%g%s%s' % (sign, mant, _PREFIX[p], unit)


def scalar_labels(values: list[float]) -> list[str]:
    """Plain numbers with a common number of decimals (no offset, no
    scientific notation), as matplotlib's ScalarFormatter shows them."""
    if not values:
        return []
    diffs = [b - a for a, b in zip(values, values[1:]) if b > a]
    step = min(diffs) if diffs else (abs(values[0]) or 1.0)
    d = max(0, -int(math.floor(math.log10(step)))) if step > 0 else 0
    while d < 12 and any(abs(round(v, d) - v) > step * 1e-6 for v in values):
        d += 1
    out = []
    for v in values:
        s = '%.*f' % (d, v)
        if s.startswith('-'):
            s = '0' if float(s) == 0 else _MINUS + s[1:]
        out.append(s)
    return out


def _ticks(locator, lo: float, hi: float) -> list[float]:
    try:
        vals = locator.tick_values(lo, hi)
    except (ValueError, OverflowError, ZeroDivisionError):
        return []
    eps = (hi - lo) * 1e-9
    return [float(x) for x in vals if lo - eps <= x <= hi + eps]


def _font(px: int) -> ImageFont.FreeTypeFont:
    f = _fonts.get(px)
    if f is None:
        try:
            f = ImageFont.truetype(_FONT_PATH, px)
        except OSError:
            f = ImageFont.load_default(px)
        _fonts[px] = f
    return f


def _rgb(color: str) -> tuple[int, int, int]:
    c = color.lstrip('#')
    if len(c) == 3:
        c = ''.join(ch * 2 for ch in c)
    try:
        return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)
    except (ValueError, IndexError):
        return 128, 128, 128


def _blend(a: str, b: str, alpha: float) -> tuple[int, int, int]:
    ca, cb = _rgb(a), _rgb(b)
    return tuple(int(round(alpha * x + (1 - alpha) * y)) for x, y in zip(ca, cb))


# -- drawing -------------------------------------------------------------------

_LIMIT = 1e6                      # pixel coordinates beyond this are clamped


def _polyline(draw: ImageDraw.ImageDraw, xs: np.ndarray, ys: np.ndarray, color, width: int) -> None:
    """Connected segments through finite points; a non-finite point breaks the line."""
    ok = np.isfinite(xs) & np.isfinite(ys)
    if not ok.any():
        return
    xs = np.clip(xs, -_LIMIT, _LIMIT)
    ys = np.clip(ys, -_LIMIT, _LIMIT)
    if ok.all():
        runs = [(0, len(xs))]
    else:
        edges = np.flatnonzero(np.diff(np.concatenate(([0], ok.astype(np.int8), [0]))))
        runs = list(zip(edges[0::2].tolist(), edges[1::2].tolist()))
    for a, b in runs:
        if b - a == 1:
            draw.point((float(xs[a]), float(ys[a])), fill=color)
            continue
        pts = np.column_stack((xs[a:b], ys[a:b])).ravel().tolist()
        draw.line(pts, fill=color, width=width)


def _clip_line_ends(xs: np.ndarray, ys: np.ndarray, lo: float, hi: float):
    """Move the end points of a polyline that lie far outside [lo, hi] onto
    the line towards their neighbour, so clamping never bends a slope."""
    if len(xs) < 2:
        return xs, ys
    xs, ys = xs.copy(), ys.copy()
    for i, j in ((0, 1), (len(xs) - 1, len(xs) - 2)):
        x, xn = xs[i], xs[j]
        for bound in (lo, hi):
            if (x < bound <= xn) or (xn <= bound < x):
                f = (bound - xn) / (x - xn)
                ys[i] = ys[j] + f * (ys[i] - ys[j])
                xs[i] = bound
                break
    return xs, ys


def _vdash(draw: ImageDraw.ImageDraw, x: float, y0: float, y1: float, color, width: int,
           on: float, off: float) -> None:
    y = y0
    while y < y1:
        draw.line([(x, y), (x, min(y + on, y1))], fill=color, width=width)
        y += on + off


class SubplotRenderer:
    """Renders frames of one subplot.  The lock serialises frames of the same
    subplot; `info` describes the last frame's axes."""

    def __init__(self):
        self.lock = threading.Lock()
        self.info: FrameInfo | None = None

    def render(self, spec: FrameSpec) -> tuple[bytes, int, int]:
        with self.lock:
            return self._render(spec)

    def _render(self, spec: FrameSpec) -> tuple[bytes, int, int]:
        s = spec.scale if spec.scale and spec.scale > 0 else 1.0
        w_px = max(int(spec.width * s), 40)
        h_px = max(int(spec.height * s), 40)
        th = THEMES.get(spec.theme, THEMES['light'])
        bg, fg = th['bg'], th['fg']

        x0, x1 = spec.x0, spec.x1
        if not (math.isfinite(x0) and math.isfinite(x1)):
            x0, x1 = 0.0, 1.0
        if not x1 > x0:
            x1 = x0 + 1e-6

        # Data first: decimate, hold, and the visible y range.
        bins = max(int(spec.width), 50)
        ymin, ymax = math.inf, -math.inf
        value_traces = [tr for tr in spec.traces if tr.kind != 'hits']
        hit_rows = [tr for tr in spec.traces if tr.kind == 'hits']
        prepared = []
        for tr in value_traces:
            t, v = minmax_decimate(tr.t, tr.v, x0, x1, bins)
            if len(t) == 0:
                continue
            hold_to = x1 if spec.t_end is None else min(x1, spec.t_end)
            if spec.style == 'step' and t[-1] < hold_to and np.isfinite(v[-1]):
                # Hold the last value to the end of the data in view.
                t = np.append(t, hold_to)
                v = np.append(v, v[-1])
            finite = np.isfinite(v)
            prepared.append((tr, t, v, finite))
            inside = finite & (t >= x0) & (t <= x1)
            vis = v[inside]
            if len(vis) == 0 and spec.style == 'step':
                before = finite & (t < x0)
                vis = v[before][-1:]          # the value held into the view
            elif len(vis) == 0:
                vis = v[finite]               # a line crossing the view
            if len(vis):
                ymin = min(ymin, float(np.min(vis)))
                ymax = max(ymax, float(np.max(vis)))
        if hit_rows and not value_traces:
            ymin, ymax = 0.0, 1.0

        ylim = spec.ylim
        if ylim is not None and not (math.isfinite(ylim[0]) and math.isfinite(ylim[1])
                                     and ylim[1] > ylim[0]):
            ylim = None
        if ylim is not None:
            y0, y1 = ylim
        elif math.isfinite(ymin) and math.isfinite(ymax):
            if ymax == ymin:
                pad = max(abs(ymin) * 0.05, 0.5)
            else:
                pad = (ymax - ymin) * 0.06
            y0, y1 = ymin - pad, ymax + pad
            if not (math.isfinite(y0) and math.isfinite(y1)) or not y1 > y0:
                y0, y1 = ymin, ymin + 1.0 if math.isfinite(ymin) else 1.0
        else:
            y0, y1 = 0.0, 1.0
        log = spec.ylog and y0 > 0

        # Geometry (device pixels).
        left, right, top = MARGIN_LEFT, MARGIN_RIGHT, MARGIN_TOP
        bottom = MARGIN_BOTTOM if spec.xlabel else MARGIN_BOTTOM_BARE
        L, T = int(round(left * s)), int(round(top * s))
        R, B = w_px - int(round(right * s)), h_px - int(round(bottom * s))
        aw, ah = max(R - L, 2), max(B - T, 2)
        self.info = FrameInfo(left, spec.width - right, top, spec.height - bottom, x0, x1, y0, y1)

        img = Image.new('RGB', (w_px, h_px), bg)
        area = Image.new('RGB', (aw, ah), bg)
        d = ImageDraw.Draw(img)
        a = ImageDraw.Draw(area)
        kx = aw / (x1 - x0)
        ly0, ly1 = (math.log10(y0), math.log10(y1)) if log else (y0, y1)
        ky = ah / (ly1 - ly0) if ly1 > ly0 else 1.0

        def px(t):
            return (np.asarray(t, dtype=float) - x0) * kx

        def py(v):
            v = np.asarray(v, dtype=float)
            if log:
                with np.errstate(divide='ignore', invalid='ignore'):
                    v = np.where(v > 0, np.log10(np.where(v > 0, v, 1.0)), np.nan)
            return (ly1 - v) * ky

        lw = max(1, int(round(1.0 * PT * s * 0.8)))
        thin = max(1, int(round(0.6 * PT * s)))

        # Grid at the tick positions.
        xt = _ticks(MaxNLocator(nbins=max(3, spec.width // 110)), x0, x1)
        if hit_rows and not value_traces:
            yt = []
        elif log:
            yt = _ticks(LogLocator(base=10), y0, y1)
        else:
            yt = _ticks(MaxNLocator(nbins=max(3, spec.height // 45)), y0, y1)
        grid = _rgb(th['grid'])
        for x in px(xt).tolist():
            a.line([(x, 0), (x, ah)], fill=grid, width=thin)
        for y in py(yt).tolist():
            a.line([(0, y), (aw, y)], fill=grid, width=thin)

        # Value traces.
        for tr, t, v, finite in prepared:
            color = _rgb(tr.color)
            xs, ys = px(t), py(v)
            if spec.style == 'points':
                r = 1.5 * s
                for x, y in zip(xs[finite].tolist(), ys[finite].tolist()):
                    if -r <= x <= aw + r and -r <= y <= ah + r:
                        a.ellipse([x - r, y - r, x + r, y + r], fill=color)
            elif spec.style == 'line':
                xs, ys = _clip_line_ends(xs, ys, -2.0 * aw, 3.0 * aw)
                _polyline(a, xs, ys, color, lw)
            else:
                # steps-post: (t0,v0) (t1,v0) (t1,v1) (t2,v1) ...
                _polyline(a, np.repeat(xs, 2)[1:], np.repeat(ys, 2)[:-1], color, lw)
            if not finite.all():
                # Accesses without a value (address-only payload): ticks at the bottom.
                h = 4.2 * s
                for x in np.unique(np.round(xs[~finite])).tolist():
                    a.line([(x, ah - h), (x, ah)], fill=color, width=lw)

        # Watch-point hits: one row of ticks per signal.
        if hit_rows:
            n = len(hit_rows)
            for k, tr in enumerate(hit_rows):
                t, _ = minmax_decimate(tr.t, tr.v, x0, x1, bins * 2)
                t = t[(t >= x0) & (t <= x1)]
                ybase = (n - k - 1) / n
                yt0, yb0 = ah * (1 - (ybase + 0.9 / n)), ah * (1 - (ybase + 0.1 / n))
                color = _rgb(tr.color)
                for x in np.unique(np.round(px(t))).tolist():
                    a.line([(x, yt0), (x, yb0)], fill=color, width=1)

        # Gaps, cursor, marker.
        gap = _blend(th['gap'], bg, 0.6)
        for g in spec.gaps:
            if x0 <= g <= x1:
                _vdash(a, float(px(g)), 0, ah, gap, thin, 3.7 * 0.8 * PT * s, 1.6 * 0.8 * PT * s)
        if spec.cursor is not None and x0 <= spec.cursor <= x1:
            x = float(px(spec.cursor))
            a.line([(x, 0), (x, ah)], fill=_rgb(th['cursor']), width=thin)
        if spec.marker is not None and x0 <= spec.marker <= x1:
            _vdash(a, float(px(spec.marker)), 0, ah, _rgb(th['marker']), lw,
                   3.7 * PT * s, 1.6 * PT * s)

        img.paste(area, (L, T))

        # Frame, ticks, labels.
        fgc, spine = _rgb(fg), _rgb(th['spine'])
        d.rectangle([L - 1, T - 1, L + aw, T + ah], outline=spine, width=1)
        tick_len = 3.5 * PT * s
        pad = 3.5 * PT * s
        font = _font(max(6, int(round(8 * PT * s))))
        xl = [eng_label(x, 's') for x in xt]
        for x, lab in zip(px(xt).tolist(), xl):
            x += L
            d.line([(x, T + ah), (x, T + ah + tick_len)], fill=fgc, width=thin)
            if spec.xlabel:
                d.text((x, T + ah + tick_len + pad * 0.6), lab, font=font, fill=fgc, anchor='mt')
        if yt:
            if log or abs(y0) >= 1e5 or abs(y1) >= 1e5:
                yl = [eng_label(y) for y in yt]
            else:
                yl = scalar_labels(yt)
            for y, lab in zip(py(yt).tolist(), yl):
                y += T
                d.line([(L - 1 - tick_len, y), (L - 1, y)], fill=fgc, width=thin)
                d.text((L - 1 - tick_len - pad, y), lab, font=font, fill=fgc, anchor='rm')
        return img.convert('RGBA').tobytes(), w_px, h_px


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
        ylim = spec.ylim
        if ylim and all(math.isfinite(y) for y in ylim) and ylim[1] > ylim[0]:
            ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper left', fontsize=7, framealpha=0.7)
    axes[-1, 0].set_xlim(subplots[0].x0, subplots[0].x1)
    axes[-1, 0].xaxis.set_major_formatter(EngFormatter(unit='s', sep=''))
    if title:
        fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
