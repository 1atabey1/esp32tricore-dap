"""MCDS Trace: pick variables from an ELF, trace them live through the probe,
plot them in real time or from a capture file.

    mcds-trace-ui [--web] [--port N] [--host PROBE] [CAPTURE.mcds]
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import csv
import datetime
import json
import math
import multiprocessing
import os
import sys
import tempfile
import threading
import time
import traceback

import flet as ft
import numpy as np

from .. import elfsyms
from ..plotrender import FrameSpec, SubplotRenderer, Trace, export_png, value_at
from ..session import FileSession, LiveSession, Probe, ProbeError
from ..signals import PALETTE, Signal, hits_signal, leaf_signal, raw_signal
from .model import Settings, SubplotModel, Workspace

APP_TITLE = 'MCDS Trace'
MONO = ft.TextStyle(font_family='monospace', size=12)
NARROW = 980                      # below this width the side panel overlays the plots
TREE_PAGE = 64                    # array elements / members shown per expansion
WINDOWS = [0.01, 0.05, 0.1, 0.5, 1, 2, 5, 10, 30, 60, 300]
DIVS = [(0, '24 MHz'), (1, '12 MHz'), (2, '8 MHz'), (3, '6 MHz'), (5, '4 MHz'), (11, '2 MHz')]


def fmt_time(t: float) -> str:
    a = abs(t)
    if a >= 1:
        return '%.6f s' % t
    if a >= 1e-3:
        return '%.3f ms' % (t * 1e3)
    if a >= 1e-6 or a == 0:
        return '%.1f us' % (t * 1e6)
    return '%.1f ns' % (t * 1e9)


def close_dialog(dlg) -> None:
    """Close this dialog (page.pop_dialog closes whatever is on top - a toast
    shown meanwhile, say)."""
    dlg.open = False
    try:
        dlg.update()
    except (RuntimeError, AssertionError):
        pass


def fmt_window(w: float) -> str:
    if w >= 60:
        return '%g min' % (w / 60)
    if w >= 1:
        return '%g s' % w
    return '%g ms' % (w * 1e3)


def fmt_bytes(n: float) -> str:
    for unit in ('B', 'kB', 'MB', 'GB'):
        if abs(n) < 1024 or unit == 'GB':
            return ('%.0f %s' if unit == 'B' else '%.1f %s') % (n, unit)
        n /= 1024
    return str(n)


def hexaddr(a: int) -> str:
    return '0x%08X' % a


def parse_int(text: str, default: int | None = None) -> int | None:
    try:
        return int(str(text).strip(), 0)
    except (TypeError, ValueError):
        return default


def short_path(path: str) -> str:
    """'ns::Class::var.member[2]' -> 'var.member[2]' (the tooltip keeps the rest);
    plain variables keep their qualified name.  std::array elements show as
    plain indices."""
    head, sep, tail = path.partition('.')
    if not sep:
        return path
    return (head.rsplit('::', 1)[-1] + sep + tail).replace('._M_elems[', '[')


def normalized(t: np.ndarray, v: np.ndarray, x0: float, x1: float) -> np.ndarray:
    """v scaled so its visible range (plus the value held into the view) is 0..1."""
    i0 = max(int(np.searchsorted(t, x0)) - 1, 0)
    i1 = int(np.searchsorted(t, x1, side='right'))
    seg = v[i0:max(i1, i0 + 1)]
    seg = seg[np.isfinite(seg)]
    if not len(seg):
        return v
    lo, hi = float(seg.min()), float(seg.max())
    if hi == lo:
        return np.where(np.isfinite(v), 0.5, v)
    return (v - lo) / (hi - lo)


def pos(e) -> tuple[float, float] | None:
    """Local pointer position of a gesture event, whatever its type."""
    lp = getattr(e, 'local_position', None)
    if lp is not None:
        return float(lp.x), float(lp.y)
    x, y = getattr(e, 'local_x', None), getattr(e, 'local_y', None)
    if x is not None and y is not None:
        return float(x), float(y)
    return None


class View:
    """The shared time window of all plots."""

    def __init__(self, window: float = 5.0):
        self.x0 = 0.0
        self.x1 = window
        self.window = window
        self.follow = True
        self.cursor: float | None = None
        self.marker: float | None = None     # reference for delta readouts
        self.version = 0

    def set(self, x0: float, x1: float) -> None:
        if not (math.isfinite(x0) and math.isfinite(x1)):
            return
        if x1 - x0 < 1e-9:
            mid = (x0 + x1) / 2
            x0, x1 = mid - 5e-10, mid + 5e-10
        self.x0, self.x1 = x0, x1
        self.version += 1

    def zoom(self, factor: float, around: float | None = None) -> None:
        c = (self.x0 + self.x1) / 2 if around is None else around
        self.set(c + (self.x0 - c) * factor, c + (self.x1 - c) * factor)

    def pan(self, dt: float) -> None:
        self.set(self.x0 + dt, self.x1 + dt)


# -- plots --------------------------------------------------------------------

class PlotCard:
    """One subplot: legend header, rendered frame, pointer interaction."""

    def __init__(self, app: 'App', model: SubplotModel):
        self.app = app
        self.model = model
        self.renderer = SubplotRenderer()
        self.size = (800.0, float(model.height))
        self.last_key = None
        self.rendering = False
        self.drag_x: float | None = None
        self.legend_values: dict[str, ft.Text] = {}

        # In a stretched Row the image gets tight constraints, so FILL maps a frame
        # onto exactly the plot box and its size is reported back.
        self.raw = ft.RawImage(fit=ft.BoxFit.FILL, expand=True,
                               on_size_change=self._on_size, size_change_interval=100)
        self.gesture = ft.GestureDetector(
            content=ft.Row([self.raw], spacing=0,
                           vertical_alignment=ft.CrossAxisAlignment.STRETCH),
            mouse_cursor=ft.MouseCursor.PRECISE,
            drag_interval=16, hover_interval=30,
            on_scroll=self._on_scroll, on_pan_start=self._on_pan_start,
            on_pan_update=self._on_pan_update, on_pan_end=self._on_pan_end,
            on_hover=self._on_hover, on_exit=self._on_exit,
            on_tap=self._on_tap, on_secondary_tap=lambda e: app.set_marker(None),
            on_double_tap=lambda e: app.fit_or_follow())
        self.legend = ft.Row(wrap=True, spacing=4, run_spacing=2, expand=True)
        self.empty_hint = ft.Text('Drop signals here, or use "Add to plot" in the Signals tab.',
                                  italic=True, size=12, color=ft.Colors.ON_SURFACE_VARIANT)
        self.body = ft.Container(content=self.gesture, height=model.height)
        self.menu = ft.PopupMenuButton(icon=ft.Icons.MORE_VERT, tooltip='Plot options')
        self.stats_row = ft.Row(wrap=True, spacing=14, run_spacing=0, visible=model.stats)
        self.header = ft.Row([self.legend, self.menu], vertical_alignment=ft.CrossAxisAlignment.START)
        self.card = ft.Container(
            content=ft.Column([self.header, self.stats_row, self.body], spacing=0),
            padding=ft.Padding.only(left=8, right=4, top=4, bottom=4),
            border=ft.Border.all(1, ft.Colors.OUTLINE_VARIANT), border_radius=8)
        self.control = ft.DragTarget(group='signal', content=self.card,
                                     on_accept=self._on_drop, on_will_accept=self._on_will,
                                     on_leave=self._on_leave)
        self.rebuild_legend()

    def _menu_items(self) -> list:
        app, model = self.app, self.model
        return [
            ft.PopupMenuItem(content='Step (sample and hold)', checked=model.style == 'step',
                             on_click=lambda e: self._style('step')),
            ft.PopupMenuItem(content='Lines', checked=model.style == 'line',
                             on_click=lambda e: self._style('line')),
            ft.PopupMenuItem(content='Points', checked=model.style == 'points',
                             on_click=lambda e: self._style('points')),
            ft.PopupMenuItem(),
            ft.PopupMenuItem(content='Normalize (each signal 0..1)', checked=model.normalize,
                             on_click=lambda e: self._toggle('normalize')),
            ft.PopupMenuItem(content='Statistics of the view', checked=model.stats,
                             on_click=lambda e: self._toggle('stats')),
            ft.PopupMenuItem(content='Y range...', icon=ft.Icons.HEIGHT,
                             on_click=lambda e: self._ylim_dialog()),
            ft.PopupMenuItem(content='One plot per signal', icon=ft.Icons.VIEW_AGENDA_OUTLINED,
                             on_click=lambda e: app.split_plot(model.id)),
            ft.PopupMenuItem(content='Merge into the plot above', icon=ft.Icons.MERGE,
                             on_click=lambda e: app.merge_up(model.id)),
            ft.PopupMenuItem(content='Taller', icon=ft.Icons.EXPAND,
                             on_click=lambda e: self._resize(+80)),
            ft.PopupMenuItem(content='Shorter', icon=ft.Icons.COMPRESS,
                             on_click=lambda e: self._resize(-80)),
            ft.PopupMenuItem(content='Move up', icon=ft.Icons.ARROW_UPWARD,
                             on_click=lambda e: app.move_plot(model.id, -1)),
            ft.PopupMenuItem(content='Move down', icon=ft.Icons.ARROW_DOWNWARD,
                             on_click=lambda e: app.move_plot(model.id, +1)),
            ft.PopupMenuItem(),
            ft.PopupMenuItem(content='Remove plot', icon=ft.Icons.DELETE_OUTLINE,
                             on_click=lambda e: app.remove_plot(model.id)),
        ]

    # legend ------------------------------------------------------------------

    def rebuild_legend(self) -> None:
        self.menu.items = self._menu_items()
        self.stats_row.visible = self.model.stats
        self.legend.controls.clear()
        self.legend_values.clear()
        ws = self.app.ws
        if not self.model.signals:
            self.legend.controls.append(self.empty_hint)
        for sid in self.model.signals:
            s = ws.signals.get(sid)
            if s is None:
                continue
            val = ft.Text('-', style=MONO, color=ft.Colors.ON_SURFACE_VARIANT)
            self.legend_values[sid] = val
            tip = '%s\n%s  %s  %d bytes' % (s.id, s.node.type_name() if s.node else 'raw',
                                            hexaddr(s.addr), s.size)
            chip = ft.Container(
                content=ft.Row([
                    ft.Container(width=10, height=10, bgcolor=s.color, border_radius=5),
                    ft.Text(s.label, size=12, weight=ft.FontWeight.W_500),
                    val,
                    ft.IconButton(ft.Icons.CLOSE, icon_size=14, width=24, height=24,
                                  style=ft.ButtonStyle(padding=0),
                                  tooltip='Remove from this plot',
                                  on_click=lambda e, i=sid: self.app.unplot(self.model.id, i)),
                ], spacing=4, tight=True),
                padding=ft.Padding.only(left=6, right=0, top=0, bottom=0),
                border_radius=12, bgcolor=ft.Colors.SURFACE_CONTAINER_HIGH, tooltip=tip)
            self.legend.controls.append(chip)
        self.last_key = None

    def update_values(self, at: float | None) -> None:
        store = self.app.store()
        for sid, txt in self.legend_values.items():
            s = self.app.ws.signals.get(sid)
            if s is None or store is None:
                txt.value = '-'
                continue
            ser = store.series.get(self.app.trace_id(s))
            if ser is None or ser.n == 0:
                txt.value = '-'
                continue
            if at is None:
                v = ser.last
            else:
                t, vv = ser.view()
                v = value_at(t, vv, at)
            if s.kind == 'hits' or (self.app.hits_mode()):
                txt.value = '%d hits' % ser.n if at is None else ''
            else:
                txt.value = s.format(v)

    # rendering ---------------------------------------------------------------

    def spec(self) -> FrameSpec:
        app = self.app
        store = app.store()
        traces = []
        if store is not None:
            for sid in self.model.signals:
                s = app.ws.signals.get(sid)
                if s is None:
                    continue
                t, v = store.window(app.trace_id(s), app.view.x0, app.view.x1)
                kind = 'hits' if (s.kind == 'hits' or app.hits_mode()) else 'value'
                if self.model.normalize and kind == 'value' and len(v):
                    v = normalized(t, v, app.view.x0, app.view.x1)
                traces.append(Trace(t, v, s.color, kind, s.label))
        w, h = self.size
        last = app.plot_cards and app.plot_cards[-1] is self
        return FrameSpec(int(w), int(h), app.view.x0, app.view.x1, traces, self.model.style,
                         tuple(self.model.ylim) if self.model.ylim else None,
                         list(store.gaps) if store is not None else [], app.view.cursor,
                         xlabel=bool(last) or len(app.plot_cards) == 1,
                         theme=app.theme_name(), scale=app.pixel_ratio(), ylog=self.model.ylog,
                         t_end=store.t_end if store is not None else None,
                         marker=app.view.marker)

    def key(self):
        app = self.app
        store = app.store()
        return (app.view.version, app.view.cursor, app.view.marker, self.size, self.model.style,
                tuple(self.model.ylim or ()), self.model.ylog, self.model.normalize,
                tuple(self.model.signals),
                app.theme_name(), id(store), store.samples if store else 0,
                len(store.gaps) if store else 0,
                self.app.plot_cards[-1] is self if self.app.plot_cards else True)

    def schedule(self) -> None:
        """Render a new frame in the background if anything changed.  Each
        plot has at most one frame in flight, and a plot that cannot show one
        (off screen, window hidden) holds up nothing else."""
        if self.rendering:
            return
        k = self.key()
        if k == self.last_key:
            return
        self.rendering = True
        self.app.page.run_task(self._render, k)

    async def _render(self, k) -> None:
        try:
            spec = self.spec()
            buf, w, h = await asyncio.to_thread(self.renderer.render, spec)
            await self.raw.render_rgba(w, h, buf)
            self.last_key = k
        except (TimeoutError, RuntimeError):
            pass                          # not attached or not visible yet; next tick
        except Exception:
            traceback.print_exc()
            self.last_key = k             # do not retry a failing frame every tick
        finally:
            self.rendering = False

    # events ------------------------------------------------------------------

    def _on_size(self, e) -> None:
        w, h = float(e.width), float(e.height)
        if w > 10 and h > 10:
            self.size = (w, h)
            self.measured = True

    def estimate_size(self, width: float) -> None:
        """Until the client reports the real size: the width the layout gives."""
        if not getattr(self, 'measured', False):
            self.size = (max(200.0, width), float(self.model.height))

    def _x_at(self, px: float) -> float | None:
        info = self.renderer.info
        if info is None:
            return None
        return info.x_of(px)

    def _on_scroll(self, e) -> None:
        p = pos(e)
        dy = getattr(getattr(e, 'scroll_delta', None), 'y', 0.0) or 0.0
        if p is None or not dy:
            return
        x = self._x_at(p[0])
        self.app.view.follow = False
        self.app.view.zoom(1.15 ** (dy / 100.0), x)
        self.app.view_changed()

    def _on_pan_start(self, e) -> None:
        p = pos(e)
        self.drag_x = p[0] if p else None

    def _on_pan_update(self, e) -> None:
        p = pos(e)
        info = self.renderer.info
        if p is None or self.drag_x is None or info is None:
            return
        dx = p[0] - self.drag_x
        self.drag_x = p[0]
        span = max(info.right - info.left, 1.0)
        self.app.view.follow = False
        self.app.view.pan(-dx / span * (self.app.view.x1 - self.app.view.x0))
        self.app.view_changed()

    def _on_pan_end(self, e) -> None:
        self.drag_x = None

    def _on_hover(self, e) -> None:
        p = pos(e)
        info = self.renderer.info
        if p is None or info is None:
            return
        self.app.set_cursor(info.x_of(p[0]) if info.left <= p[0] <= info.right else None)

    def _on_exit(self, e) -> None:
        self.app.set_cursor(None)

    def _on_tap(self, e) -> None:
        p = pos(e)
        info = self.renderer.info
        if p is not None and info is not None and info.left <= p[0] <= info.right:
            self.app.set_marker(info.x_of(p[0]))

    def _on_will(self, e) -> None:
        self.card.border = ft.Border.all(2, ft.Colors.PRIMARY)
        self.card.update()

    def _on_leave(self, e) -> None:
        self.card.border = ft.Border.all(1, ft.Colors.OUTLINE_VARIANT)
        self.card.update()

    def _on_drop(self, e) -> None:
        self._on_leave(e)
        src = getattr(e, 'src', None)
        sid = getattr(src, 'data', None)
        if sid:
            self.app.plot_signal(sid, self.model.id)

    def _style(self, style: str) -> None:
        self.model.style = style
        self.app.rebuild_plots()

    def _toggle(self, attr: str) -> None:
        setattr(self.model, attr, not getattr(self.model, attr))
        self.app.rebuild_plots()

    def update_stats(self) -> None:
        """Count, min, max, mean and typical interval of each signal in view."""
        if not self.model.stats:
            return
        store = self.app.store()
        x0, x1 = self.app.view.x0, self.app.view.x1
        sigs = [s for s in (self.app.ws.signals.get(sid) for sid in self.model.signals)
                if s is not None]
        layout = tuple((s.id, s.color) for s in sigs)
        if layout != getattr(self, '_stats_layout', None):
            # Controls only when the signals change; the texts are reused.
            self._stats_layout = layout
            self._stats_texts = [ft.Text('', size=11, style=MONO) for _ in sigs]
            self.stats_row.controls = [
                ft.Row([ft.Container(width=8, height=8, bgcolor=s.color, border_radius=4), txt],
                       spacing=4, tight=True) for s, txt in zip(sigs, self._stats_texts)]
        for s, txt in zip(sigs, self._stats_texts):
            if store is None:
                txt.value = 'n 0'
                continue
            t, v = store.window(self.app.trace_id(s), x0, x1)
            i0, i1 = int(np.searchsorted(t, x0)), int(np.searchsorted(t, x1, side='right'))
            tt, vv = t[i0:i1], v[i0:i1]
            fin = vv[np.isfinite(vv)]
            parts = ['n %d' % len(tt)]
            if len(fin) and not (s.kind == 'hits' or self.app.hits_mode()):
                parts += ['min %s' % s.format(float(fin.min())), 'max %s' % s.format(float(fin.max())),
                          'mean %.6g' % float(fin.mean())]
            if len(tt) > 1:
                parts.append('dt %s' % fmt_time(float(np.median(np.diff(tt)))))
            txt.value = '  '.join(parts)

    def push_readouts(self) -> None:
        """Send the legend values and statistics (not the whole page)."""
        for ctl in (self.legend, self.stats_row):
            try:
                ctl.update()
            except (RuntimeError, AssertionError):
                pass                      # not on the page (yet)

    def _resize(self, delta: int) -> None:
        self.model.height = int(max(100, min(900, self.model.height + delta)))
        self.body.height = self.model.height
        self.body.update()

    def _ylim_dialog(self) -> None:
        lo = ft.TextField(label='Minimum', width=140, dense=True,
                          value='' if not self.model.ylim else '%g' % self.model.ylim[0])
        hi = ft.TextField(label='Maximum', width=140, dense=True,
                          value='' if not self.model.ylim else '%g' % self.model.ylim[1])
        log = ft.Checkbox(label='Logarithmic', value=self.model.ylog)

        def apply(e):
            if not (lo.value or '').strip() and not (hi.value or '').strip():
                self.model.ylim = None
            else:
                try:
                    a, b = float(lo.value), float(hi.value)
                except (TypeError, ValueError):
                    a = b = math.nan
                if not (math.isfinite(a) and math.isfinite(b)) or a == b:
                    lo.error = 'two different numbers' if not math.isfinite(a) or a == b else None
                    hi.error = None if math.isfinite(b) and a != b else 'a number'
                    dlg.update()
                    return
                self.model.ylim = [min(a, b), max(a, b)]
            self.model.ylog = bool(log.value)
            close_dialog(dlg)
            self.last_key = None

        def auto(e):
            self.model.ylim = None
            self.model.ylog = bool(log.value)
            close_dialog(dlg)
            self.last_key = None

        dlg = ft.AlertDialog(
            title=ft.Text('Y range'),
            content=ft.Column([ft.Row([lo, hi]), log,
                               ft.Text('Leave empty for automatic scaling.', size=12)],
                              tight=True),
            actions=[ft.TextButton('Automatic', on_click=auto),
                     ft.FilledButton('Apply', on_click=apply)])
        self.app.page.show_dialog(dlg)


# -- the application ------------------------------------------------------------

class App:
    def __init__(self, page: ft.Page, args):
        self.page = page
        self.args = args
        self.settings = Settings.load()
        # --host applies to this run only; editing the field saves it.
        self.host = args.host or self.settings.host
        self.ws = Workspace()
        self.table: elfsyms.SymbolTable | None = None
        self.session: LiveSession | FileSession | None = None
        self.mode = 'idle'                  # idle | starting | live | stopping | review | loading
        self.view = View(self.settings.window_s)
        self.plot_cards: list[PlotCard] = []
        self.busy_text = ''
        self.progress: float | None = None
        self.probe_ok: bool | None = None
        self.side_visible = True
        self.last_status_poll = 0.0
        self._search_task: asyncio.Task | None = None
        self.expanded: set[str] = set()
        self.shown: dict[str, int] = {}     # path -> children shown
        self.search_results: list[elfsyms.Node] = []
        self.member_results: list[elfsyms.Node] = []
        self.closed = False
        self._auto_stopping = False
        self.warned: set[str] = set()       # signals already reported as not traced
        self._pending: LiveSession | None = None    # a session still starting
        self._leaf_counts: dict[tuple, int] = {}      # tree rows: numeric members inside

    # -- small helpers ---------------------------------------------------------

    def store(self):
        return self.session.store if self.session is not None else None

    def hits_mode(self) -> bool:
        cfg = self.session_config()
        return cfg.get('mode') == 'compact'

    def session_config(self) -> dict:
        if self.session is None:
            return {}
        return getattr(self.session, 'config', {}) or {}

    def trace_id(self, s: Signal) -> str:
        """The series a workspace signal reads: its own, or its slot's hits in
        a compact session."""
        if s.kind == 'hits' or not self.hits_mode():
            return s.id
        for j, sl in enumerate(self.session_config().get('slots', [])):
            if not sl.get('enabled'):
                continue
            lo = parse_int(sl.get('addr'), 0)
            if lo <= s.addr < lo + int(sl.get('size', 0)):
                return 'hits:%d' % j
        return s.id

    def pixel_ratio(self) -> float:
        """Device pixels per logical pixel (frames render at physical resolution,
        capped to keep frames small)."""
        try:
            r = float(self.page.media.device_pixel_ratio or 1.0)
        except (AttributeError, TypeError, ValueError):
            r = 1.0
        return max(1.0, min(r, 2.0))

    def theme_name(self) -> str:
        return 'dark' if self.page.theme_mode == ft.ThemeMode.DARK else 'light'

    def toast(self, msg: str, error: bool = False) -> None:
        self.page.show_dialog(ft.SnackBar(
            content=ft.Text(msg, color=ft.Colors.ON_ERROR if error else None),
            bgcolor=ft.Colors.ERROR if error else None, duration=6000 if error else 3500,
            show_close_icon=True))

    def guard(self, fn):
        """Wrap an event handler: exceptions become an error toast, not a crash."""
        async def run(e=None):
            try:
                r = fn(e)
                if asyncio.iscoroutine(r):
                    await r
            except Exception as ex:
                traceback.print_exc()
                known = (RuntimeError, ProbeError, ValueError, OSError)
                self.toast(str(ex) if isinstance(ex, known) and str(ex) else
                           '%s: %s' % (type(ex).__name__, ex), error=True)
            try:
                self.page.update()
            except Exception:
                pass
        return run

    # -- build -----------------------------------------------------------------

    def build(self) -> None:
        page = self.page
        page.title = APP_TITLE
        page.theme_mode = ft.ThemeMode.DARK if self.settings.theme == 'dark' else ft.ThemeMode.LIGHT
        page.theme = ft.Theme(color_scheme_seed=ft.Colors.INDIGO)
        page.dark_theme = ft.Theme(color_scheme_seed=ft.Colors.INDIGO)
        page.padding = 0
        page.spacing = 0
        page.on_resize = self._on_resize
        page.on_keyboard_event = self._on_key

        self._build_topbar()
        self._build_symbols()
        self._build_signals()
        self._build_capture()
        self._build_main()
        self._build_status()

        self.tabs = ft.Tabs(
            length=3, selected_index=0, expand=True,
            content=ft.Column([
                ft.TabBar(tabs=[ft.Tab(label='Symbols', icon=ft.Icons.DATA_OBJECT),
                                ft.Tab(label='Signals', icon=ft.Icons.SHOW_CHART),
                                ft.Tab(label='Capture', icon=ft.Icons.SETTINGS_INPUT_ANTENNA)]),
                ft.TabBarView(expand=True, controls=[self.symbols_view, self.signals_view,
                                                     self.capture_view]),
            ], expand=True, spacing=0))
        self.side = ft.Container(self.tabs, width=380,
                                 border=ft.Border.only(right=ft.BorderSide(1, ft.Colors.OUTLINE_VARIANT)))
        self.body = ft.Row([self.side, self.main], expand=True, spacing=0,
                           vertical_alignment=ft.CrossAxisAlignment.STRETCH)
        page.add(ft.Column([self.topbar, self.body, self.statusbar], expand=True, spacing=0))
        self._layout(page.width or 1280)

    def _build_topbar(self) -> None:
        self.host_field = ft.TextField(value=self.host, width=190, dense=True,
                                       label='Probe', text_size=13,
                                       on_submit=self.guard(self._host_changed),
                                       on_blur=self.guard(self._host_changed))
        self.probe_dot = ft.Container(width=10, height=10, border_radius=5,
                                      bgcolor=ft.Colors.OUTLINE, tooltip='probe not checked yet')
        self.probe_text = ft.Text('', size=12, color=ft.Colors.ON_SURFACE_VARIANT)
        btn = lambda icon, tip, fn: ft.IconButton(icon, tooltip=tip, on_click=self.guard(fn))
        self.open_menu = ft.PopupMenuButton(icon=ft.Icons.FOLDER_OPEN, tooltip='Open a capture file')
        self._refresh_recent()
        self.topbar = ft.Container(
            content=ft.Row([
                btn(ft.Icons.MENU, 'Show or hide the side panel', self._toggle_side),
                ft.Icon(ft.Icons.MULTILINE_CHART, color=ft.Colors.PRIMARY),
                ft.Text(APP_TITLE, size=18, weight=ft.FontWeight.BOLD),
                ft.Container(width=16),
                self.host_field, self.probe_dot, self.probe_text,
                ft.Container(expand=True),
                btn(ft.Icons.MEMORY, 'Open an ELF (symbols)', self.pick_elf),
                self.open_menu,
                btn(ft.Icons.SAVE_OUTLINED, 'Save the workspace (signals, plots, settings)',
                    self.save_workspace),
                btn(ft.Icons.FILE_OPEN_OUTLINED, 'Load a workspace', self.load_workspace),
                btn(ft.Icons.DARK_MODE_OUTLINED, 'Light / dark theme', self._toggle_theme),
                btn(ft.Icons.HELP_OUTLINE, 'Help', self._help),
            ], spacing=4, vertical_alignment=ft.CrossAxisAlignment.CENTER),
            padding=ft.Padding.symmetric(horizontal=8, vertical=4),
            border=ft.Border.only(bottom=ft.BorderSide(1, ft.Colors.OUTLINE_VARIANT)))

    # symbols -----------------------------------------------------------------

    def _build_symbols(self) -> None:
        self.elf_text = ft.Text('No ELF loaded', size=12, expand=True, no_wrap=False,
                                color=ft.Colors.ON_SURFACE_VARIANT, selectable=True)
        self.elf_progress = ft.ProgressBar(visible=False)
        self.search = ft.TextField(hint_text='Search variables (name or part of it)',
                                   prefix_icon=ft.Icons.SEARCH, dense=True,
                                   on_change=self._on_search_change, disabled=True, expand=True)
        self.tree = ft.ListView(expand=True, spacing=0)
        self.raw_addr = ft.TextField(label='Address', hint_text='0x5000220C', dense=True,
                                     width=130, text_size=13)
        self.raw_size = ft.Dropdown(label='Bytes', width=90, dense=True, value='4',
                                    options=[ft.DropdownOption(k) for k in ('1', '2', '4', '8')])
        self.raw_signed = ft.Checkbox(label='signed', value=False)
        self.symbols_view = ft.Container(
            content=ft.Column([
                ft.Row([self.elf_text,
                        ft.FilledTonalButton('Open ELF', icon=ft.Icons.MEMORY,
                                             on_click=self.guard(self.pick_elf))]),
                self.elf_progress,
                ft.Row([self.search]),
                ft.Container(self.tree, expand=True,
                             border=ft.Border.all(1, ft.Colors.OUTLINE_VARIANT), border_radius=6),
                ft.Text('Raw address (no ELF needed)', size=12, weight=ft.FontWeight.W_500),
                ft.Row([self.raw_addr, self.raw_size, self.raw_signed,
                        ft.IconButton(ft.Icons.ADD, tooltip='Add this address as a signal',
                                      on_click=self.guard(self._add_raw))], spacing=4),
            ], expand=True, spacing=8),
            padding=10)
        self._render_tree()

    def _on_search_change(self, e) -> None:
        if self._search_task is not None:
            self._search_task.cancel()

        async def later():
            await asyncio.sleep(0.25)
            text = self.search.value or ''
            if self.table is None:
                return
            # The member index is built on the first search: off the UI loop.
            vars_, members = await asyncio.to_thread(self._search, text)
            if (self.search.value or '') != text:
                return                       # typed on meanwhile
            self._show_search(vars_, members)
            self.tree.update()
        self._search_task = self.page.run_task(later)

    def _search(self, text: str):
        vars_ = self.table.search(text, limit=300)
        members = []
        if text.strip():
            for p in self.table.search_members(text, limit=150):
                n = elfsyms.resolve_path(self.table.variables, p)
                if n is not None:
                    members.append(n)
        return vars_, members

    def _show_search(self, vars_, members) -> None:
        self.search_results = [elfsyms.node_of(v) for v in vars_]
        self.member_results = members
        self.expanded.clear()
        self.shown.clear()
        self._render_tree()

    def _do_search(self, text: str) -> None:
        if self.table is None:
            return
        self._show_search(self.table.search(text, limit=300), [])

    def _render_tree(self) -> None:
        rows = []
        if self.table is None:
            rows.append(ft.Container(ft.Text('Open the ELF of the traced application to pick '
                                             'variables by name.\nOr add a raw address below.',
                                             size=12, color=ft.Colors.ON_SURFACE_VARIANT),
                                     padding=10))
        elif not self.search_results and not self.member_results:
            rows.append(ft.Container(ft.Text('No match.', size=12, italic=True), padding=10))
        else:
            for n in self.search_results:
                self._tree_rows(n, 0, rows)
            if len(self.search_results) >= 300:
                rows.append(ft.Container(ft.Text('Showing the first 300 variables; refine the '
                                                 'search.', size=11, italic=True), padding=6))
            if self.member_results:
                rows.append(ft.Container(ft.Text('Members', size=12, weight=ft.FontWeight.W_600,
                                                 color=ft.Colors.PRIMARY),
                                         padding=ft.Padding.only(left=8, top=8, bottom=2)))
                for n in self.member_results:
                    self._tree_rows(n, 0, rows)
        self.tree.controls = rows

    def _tree_rows(self, node: elfsyms.Node, depth: int, rows: list) -> None:
        exp = node.expandable
        open_ = node.path in self.expanded
        plotted = node.path in self.ws.signals
        chevron = ft.IconButton(
            ft.Icons.EXPAND_MORE if open_ else ft.Icons.CHEVRON_RIGHT, icon_size=18,
            width=28, height=28, style=ft.ButtonStyle(padding=0),
            on_click=self.guard(lambda e, n=node: self._toggle_node(n))) if exp else \
            ft.Container(width=28)
        name = ft.Text(node.name if depth else short_path(node.path), size=13,
                       weight=ft.FontWeight.W_600 if depth == 0 else None,
                       color=ft.Colors.PRIMARY if plotted else None, no_wrap=True,
                       overflow=ft.TextOverflow.ELLIPSIS, expand=True,
                       tooltip='%s\n%s at %s, %d bytes' % (node.path, node.type_name(),
                                                           hexaddr(node.addr), node.size))
        typ = ft.Text(node.type_name(), size=11, color=ft.Colors.ON_SURFACE_VARIANT,
                      no_wrap=True, overflow=ft.TextOverflow.ELLIPSIS, width=92)
        if node.is_leaf:
            add = ft.IconButton(ft.Icons.ADD_CIRCLE_OUTLINE if not plotted else ft.Icons.CHECK_CIRCLE,
                                icon_size=18, width=30, height=30, style=ft.ButtonStyle(padding=0),
                                tooltip='Add to the plot' if not plotted else 'Already selected',
                                on_click=self.guard(lambda e, n=node: self.add_leaf(n)))
        else:
            key = (node.path, node.addr, id(node.type))
            count = self._leaf_counts.get(key)
            if count is None:
                count = self._leaf_counts[key] = sum(1 for _ in node.leaves(1025))
            add = ft.IconButton(ft.Icons.PLAYLIST_ADD, icon_size=18, width=30, height=30,
                                style=ft.ButtonStyle(padding=0),
                                tooltip='Add all %s%d numeric members' % ('1024+ ' if count > 1024 else '', min(count, 1024))
                                if count else 'Nothing traceable inside',
                                disabled=count == 0,
                                on_click=self.guard(lambda e, n=node: self.add_all(n)))
        row = ft.Container(
            content=ft.Row([chevron, name, typ, add], spacing=2),
            padding=ft.Padding.only(left=4 + depth * 14, right=2, top=0, bottom=0),
            on_click=self.guard(lambda e, n=node: self.add_leaf(n) if n.is_leaf
                                else self._toggle_node(n)),
            ink=True)
        rows.append(row)
        if exp and open_:
            limit = self.shown.get(node.path, TREE_PAGE)
            kids = node.children(0, limit)
            for c in kids:
                self._tree_rows(c, depth + 1, rows)
            total = node.child_count()
            if total > limit:
                rows.append(ft.Container(
                    ft.TextButton('show %d more of %d' % (min(TREE_PAGE, total - limit), total - limit),
                                  on_click=self.guard(lambda e, n=node: self._more(n))),
                    padding=ft.Padding.only(left=32 + depth * 14)))

    def _toggle_node(self, node: elfsyms.Node) -> None:
        if node.path in self.expanded:
            self.expanded.discard(node.path)
        else:
            self.expanded.add(node.path)
        self._render_tree()

    def _more(self, node: elfsyms.Node) -> None:
        self.shown[node.path] = self.shown.get(node.path, TREE_PAGE) + TREE_PAGE
        self._render_tree()

    # signals -----------------------------------------------------------------

    def _build_signals(self) -> None:
        self.signal_list = ft.ListView(expand=True, spacing=2)
        self.signals_title = ft.Text('', size=13, weight=ft.FontWeight.W_500)
        self.signals_view = ft.Container(
            content=ft.Column([
                ft.Row([self.signals_title, ft.Container(expand=True),
                        ft.TextButton('New plot', icon=ft.Icons.ADD_CHART,
                                      on_click=self.guard(lambda e: self.add_plot())),
                        ft.TextButton('Clear', icon=ft.Icons.DELETE_SWEEP_OUTLINED,
                                      on_click=self.guard(lambda e: self.clear_signals()))]),
                ft.Text('Drag a signal onto a plot, or use its menu.', size=11,
                        color=ft.Colors.ON_SURFACE_VARIANT),
                self.signal_list,
            ], expand=True, spacing=6),
            padding=10)
        self._render_signals()

    def _render_signals(self) -> None:
        rows = []
        for s in self.ws.signals.values():
            plots = [str(i + 1) for i, p in enumerate(self.ws.subplots) if s.id in p.signals]
            dot = ft.Container(width=14, height=14, bgcolor=s.color, border_radius=7,
                               tooltip='Change colour',
                               on_click=self.guard(lambda e, sid=s.id: self._cycle_color(sid)))
            items = [ft.PopupMenuItem(content='Add to plot %d' % (i + 1),
                                      on_click=self.guard(lambda e, sid=s.id, pid=p.id:
                                                          self.plot_signal(sid, pid)))
                     for i, p in enumerate(self.ws.subplots)]
            items += [ft.PopupMenuItem(content='New plot with it', icon=ft.Icons.ADD_CHART,
                                       on_click=self.guard(lambda e, sid=s.id: self.plot_signal(sid, -1))),
                      ft.PopupMenuItem(content='Rename...', icon=ft.Icons.EDIT_OUTLINED,
                                       on_click=self.guard(lambda e, sid=s.id: self._rename(sid))),
                      ft.PopupMenuItem(),
                      ft.PopupMenuItem(content='Remove', icon=ft.Icons.DELETE_OUTLINE,
                                       on_click=self.guard(lambda e, sid=s.id: self.remove_signal(sid)))]
            desc = '%s  %s  %dB' % (s.node.type_name() if s.node else ('signed' if s.signed else 'raw'),
                                    hexaddr(s.addr), s.size) if s.kind == 'value' else 'watch-point hits'
            body = ft.Container(
                content=ft.Row([
                    ft.Icon(ft.Icons.DRAG_INDICATOR, size=16, color=ft.Colors.ON_SURFACE_VARIANT),
                    dot,
                    ft.Column([ft.Text(s.label, size=13, weight=ft.FontWeight.W_500, no_wrap=True,
                                       overflow=ft.TextOverflow.ELLIPSIS),
                               ft.Text(desc, size=11, color=ft.Colors.ON_SURFACE_VARIANT,
                                       no_wrap=True)], spacing=0, expand=True, tight=True),
                    ft.Text(('plot ' + ','.join(plots)) if plots else 'not plotted', size=11,
                            color=ft.Colors.ON_SURFACE_VARIANT),
                    ft.PopupMenuButton(icon=ft.Icons.MORE_VERT, items=items),
                ], spacing=6),
                padding=ft.Padding.symmetric(horizontal=6, vertical=4), border_radius=6,
                bgcolor=ft.Colors.SURFACE_CONTAINER_LOW, tooltip=s.id)
            rows.append(ft.Draggable(
                group='signal', data=s.id, content=body,
                content_feedback=ft.Container(
                    ft.Row([ft.Container(width=12, height=12, bgcolor=s.color, border_radius=6),
                            ft.Text(s.label, size=13)], tight=True),
                    padding=8, border_radius=8, bgcolor=ft.Colors.SURFACE_CONTAINER_HIGHEST)))
        if not rows:
            rows.append(ft.Container(ft.Text('No signals yet: add them from the Symbols tab.',
                                             size=12, italic=True), padding=10))
        self.signal_list.controls = rows
        n = len(self.ws.signals)
        self.signals_title.value = '%d signal%s' % (n, '' if n == 1 else 's')

    def _cycle_color(self, sid: str) -> None:
        s = self.ws.signals.get(sid)
        if s is None:
            return
        i = PALETTE.index(s.color) if s.color in PALETTE else -1
        s.color = PALETTE[(i + 1) % len(PALETTE)]
        self.selection_changed(retrace=False)

    def _rename(self, sid: str) -> None:
        s = self.ws.signals.get(sid)
        if s is None:
            return
        field = ft.TextField(value=s.label, autofocus=True, width=320)

        def ok(e):
            s.label = (field.value or s.label).strip()[:60] or s.label
            close_dialog(dlg)
            self.selection_changed(retrace=False)
            self.page.update()
        field.on_submit = ok
        dlg = ft.AlertDialog(title=ft.Text('Signal name'), content=field,
                             actions=[ft.FilledButton('OK', on_click=ok)])
        self.page.show_dialog(dlg)

    # capture panel -------------------------------------------------------------

    def _dd(self, label: str, value: str, options: list[tuple[str, str]], on, width=None):
        return ft.Dropdown(label=label, value=value, dense=True, width=width, expand=width is None,
                           options=[ft.DropdownOption(key=k, text=t) for k, t in options],
                           on_select=self.guard(on))

    def _build_capture(self) -> None:
        c = self.ws.capture
        self.cap_source = self._dd('Traced by', c.source,
                                   [('cpu', 'CPU pipeline'), ('memslave', 'CPU memory slave (any master)'),
                                    ('lmu0', 'LMU0 slave')], self._cap_changed)
        self.cap_cpu = self._dd('CPU', str(c.cpu), [('-1', 'Auto')] + [(str(i), 'CPU%d' % i) for i in range(4)],
                                self._cap_changed, width=120)
        self.cap_mode = self._dd('Record', c.mode, [('full', 'Values'),
                                                    ('compact', 'Hit times only')],
                                 self._cap_changed)
        self.cap_access = self._dd('Accesses', c.access, [('w', 'Writes'), ('r', 'Reads'),
                                                          ('rw', 'Reads and writes')],
                                   self._cap_changed, width=170)
        self.cap_payload = self._dd('Per access', c.payload,
                                    [('addr_data', 'Address + value'), ('data', 'Value only'),
                                     ('addr', 'Address only')], self._cap_changed)
        self.cap_ts = self._dd('Timestamps', c.timestamps,
                               [('hit', 'Every access (exact)'),
                                ('ticks', 'Tick stream (exact, heavier)'),
                                ('none', 'Per paragraph only')], self._cap_changed)
        self.cap_div = self._dd('DAP clock', str(c.dap_div), [(str(d), t) for d, t in DIVS],
                                self._cap_changed, width=130)
        self.cap_wide = ft.Switch(label='Wide mode (DAP2)', value=c.wide,
                                  on_change=self.guard(self._cap_changed))
        self.cap_masters = ft.Checkbox(label='Keep bus master in addresses', value=c.masters,
                                       on_change=self.guard(self._cap_changed))
        self.cap_dir = ft.TextField(label='Capture folder', value=self.settings.capture_dir,
                                    dense=True, expand=True, text_size=13,
                                    on_blur=self.guard(self._dir_changed),
                                    on_submit=self.guard(self._dir_changed))
        self.cap_duration = ft.TextField(label='Stop after (s, 0 = manual)', dense=True, width=190,
                                         text_size=13, value='%g' % self.ws.capture.duration,
                                         on_blur=self.guard(self._cap_changed),
                                         on_submit=self.guard(self._cap_changed))
        self.cap_history = ft.TextField(label='Keep (s)', value='%g' % self.settings.history_s,
                                        dense=True, width=90, text_size=13,
                                        on_blur=self.guard(self._dir_changed))
        self.plan_view = ft.Column(spacing=4)
        self.warn_view = ft.Column(spacing=2)
        self.cap_start = ft.FilledButton('Start tracing', icon=ft.Icons.PLAY_ARROW,
                                         on_click=self.guard(self.start_stop), height=44)
        self.stats_view = ft.Text('', size=12, style=MONO)
        section = lambda t: ft.Text(t, size=13, weight=ft.FontWeight.W_600, color=ft.Colors.PRIMARY)
        self.capture_view = ft.Container(
            content=ft.Column([
                section('What is traced'),
                ft.Row([self.cap_source, self.cap_cpu]),
                ft.Row([self.cap_mode, self.cap_access]),
                ft.Row([self.cap_payload]),
                ft.Row([self.cap_ts]),
                self.cap_masters,
                section('Watch ranges'),
                ft.Text('Chosen from the selected signals; the trace hardware has two.', size=11,
                        color=ft.Colors.ON_SURFACE_VARIANT),
                self.plan_view,
                self.warn_view,
                section('Link'),
                ft.Row([self.cap_div, self.cap_wide]),
                section('Recording'),
                ft.Row([self.cap_dir,
                        ft.IconButton(ft.Icons.FOLDER_OUTLINED, tooltip='Choose the folder',
                                      on_click=self.guard(self._pick_dir)), self.cap_history]),
                self.cap_duration,
                ft.Row([self.cap_start], alignment=ft.MainAxisAlignment.CENTER),
                self.stats_view,
            ], spacing=8, scroll=ft.ScrollMode.AUTO, expand=True),
            padding=10)
        self._render_plan()

    def _cap_changed(self, e) -> None:
        c = self.ws.capture
        c.source = self.cap_source.value or 'cpu'
        c.cpu = parse_int(self.cap_cpu.value, -1)
        c.mode = self.cap_mode.value or 'full'
        c.access = self.cap_access.value or 'w'
        c.payload = self.cap_payload.value or 'addr_data'
        c.timestamps = self.cap_ts.value or 'hit'
        c.dap_div = parse_int(self.cap_div.value, 0)
        c.wide = bool(self.cap_wide.value)
        c.masters = bool(self.cap_masters.value)
        try:
            c.duration = max(0.0, float(self.cap_duration.value or 0))
        except ValueError:
            c.duration = 0.0
        self.cap_cpu.disabled = c.source == 'lmu0'
        self.cap_payload.disabled = c.mode == 'compact'
        self._render_plan()

    def _dir_changed(self, e) -> None:
        self.settings.capture_dir = (self.cap_dir.value or '').strip() or self.settings.capture_dir
        hist = parse_int(self.cap_history.value)
        try:
            self.settings.history_s = max(1.0, float(self.cap_history.value))
        except (TypeError, ValueError):
            if hist:
                self.settings.history_s = float(hist)
        self.settings.save()

    def _render_plan(self) -> None:
        cfg, plan, warnings = self.ws.probe_config()
        rows = []
        if plan.ranges and cfg['source'] != 'lmu0':
            rows.append(ft.Text('Traced core: CPU%d%s' % (cfg['cpu'], ' (auto: owner of the memory)'
                                                          if self.ws.capture.cpu < 0 else ''),
                                size=12))
        for j in range(2):
            f = self.ws.capture.filters[j]
            if j < len(plan.ranges):
                a, n = plan.ranges[j]
                ids = plan.signals[j]
                labels = ', '.join(self.ws.signals[i].label for i in ids if i in self.ws.signals)
                head = ft.Text('Slot %d  %s  %d bytes' % (j, hexaddr(a), n), style=MONO)
                inner = [head, ft.Text(labels, size=11, color=ft.Colors.ON_SURFACE_VARIANT)]
                if len(ids) == 1 and self.ws.capture.mode == 'full':
                    en = ft.Checkbox(label='Only values in', value=f.enabled)
                    lo = ft.TextField(value=str(f.lo if not f.signed or f.lo < 2 ** 31 else f.lo - 2 ** 32),
                                      width=90, dense=True, text_size=12)
                    hi = ft.TextField(value=str(f.hi if not f.signed or f.hi < 2 ** 31 else f.hi - 2 ** 32),
                                      width=100, dense=True, text_size=12)
                    sg = ft.Checkbox(label='signed', value=f.signed)

                    def upd(e, f=f, en=en, lo=lo, hi=hi, sg=sg):
                        f.enabled = bool(en.value)
                        f.signed = bool(sg.value)
                        a_ = parse_int(lo.value, 0)
                        b_ = parse_int(hi.value, 0xFFFFFFFF)
                        f.lo, f.hi = a_ & 0xFFFFFFFF, b_ & 0xFFFFFFFF
                    for ctl in (en, sg):
                        ctl.on_change = self.guard(upd)
                    for ctl in (lo, hi):
                        ctl.on_blur = self.guard(upd)
                        ctl.on_submit = self.guard(upd)
                    inner.append(ft.Row([en, lo, ft.Text('..'), hi, sg], spacing=4, wrap=True))
                rows.append(ft.Container(ft.Column(inner, spacing=2, tight=True), padding=6,
                                         border_radius=6, bgcolor=ft.Colors.SURFACE_CONTAINER_LOW))
            else:
                rows.append(ft.Text('Slot %d  unused' % j, size=12,
                                    color=ft.Colors.ON_SURFACE_VARIANT))
        self.plan_view.controls = rows
        self.warn_view.controls = [ft.Row([ft.Icon(ft.Icons.INFO_OUTLINE, size=14,
                                                   color=ft.Colors.TERTIARY),
                                           ft.Text(w, size=11, expand=True)],
                                          vertical_alignment=ft.CrossAxisAlignment.START)
                                   for w in warnings if not w.startswith('Select')]

    # main area ---------------------------------------------------------------

    def _build_main(self) -> None:
        self.start_btn = ft.FilledButton('Start', icon=ft.Icons.PLAY_ARROW,
                                         on_click=self.guard(self.start_stop))
        self.follow_sw = ft.Switch(label='Follow', value=True,
                                   on_change=self.guard(self._follow_changed))
        self.window_dd = ft.Dropdown(
            value='%g' % self.settings.window_s, width=130, dense=True, label='Window',
            options=[ft.DropdownOption(key='%g' % w, text=fmt_window(w))
                     for w in WINDOWS],
            on_select=self.guard(self._window_changed))
        self.cursor_text = ft.Text('', style=MONO, color=ft.Colors.ON_SURFACE_VARIANT)
        ib = lambda icon, tip, fn: ft.IconButton(icon, tooltip=tip, on_click=self.guard(fn))
        gap = lambda: ft.Container(width=12)
        self.toolbar = ft.Row([
            self.start_btn, gap(),
            self.follow_sw, self.window_dd,
            ib(ft.Icons.FIT_SCREEN, 'Show everything (double-click a plot)', lambda e: self.fit()),
            ib(ft.Icons.ZOOM_IN, 'Zoom in (mouse wheel over a plot)', lambda e: self._zoom(0.5)),
            ib(ft.Icons.ZOOM_OUT, 'Zoom out', lambda e: self._zoom(2.0)),
            gap(),
            ib(ft.Icons.ADD_CHART, 'Add a plot', lambda e: self.add_plot()),
            ib(ft.Icons.IMAGE_OUTLINED, 'Export the view as PNG', self.export_png),
            ib(ft.Icons.TABLE_VIEW_OUTLINED, 'Export the visible samples as CSV', self.export_csv),
            gap(),
            self.cursor_text,
        ], spacing=4, wrap=True, vertical_alignment=ft.CrossAxisAlignment.CENTER)
        self.plots = ft.ListView(expand=True, spacing=8, padding=ft.Padding.only(right=8, bottom=8))
        self.empty_state = ft.Container(
            content=ft.Column([
                ft.Icon(ft.Icons.MULTILINE_CHART, size=64, color=ft.Colors.OUTLINE),
                ft.Text('Nothing to plot yet', size=18, weight=ft.FontWeight.W_500),
                ft.Text('1. Open the ELF of your application (Symbols tab)\n'
                        '2. Search a variable and add its members\n'
                        '3. Press Start - or open a capture file to analyse it offline',
                        size=13, color=ft.Colors.ON_SURFACE_VARIANT),
                ft.Row([ft.FilledTonalButton('Open ELF', icon=ft.Icons.MEMORY,
                                             on_click=self.guard(self.pick_elf)),
                        ft.OutlinedButton('Open capture', icon=ft.Icons.FOLDER_OPEN,
                                          on_click=self.guard(self.pick_capture))],
                       alignment=ft.MainAxisAlignment.CENTER),
            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, spacing=10, tight=True),
            alignment=ft.Alignment.CENTER, expand=True)
        self.main = ft.Container(
            content=ft.Column([self.toolbar, self.plots, self.empty_state], expand=True, spacing=6),
            padding=ft.Padding.only(left=8, top=6), expand=True)

    def _build_status(self) -> None:
        self.status_left = ft.Text('Ready', size=12)
        self.status_right = ft.Text('', size=12, style=MONO, color=ft.Colors.ON_SURFACE_VARIANT)
        self.busy_bar = ft.ProgressBar(width=120, visible=False)
        self.statusbar = ft.Container(
            content=ft.Row([self.status_left, self.busy_bar, ft.Container(expand=True),
                            self.status_right], spacing=10),
            padding=ft.Padding.symmetric(horizontal=10, vertical=4),
            bgcolor=ft.Colors.SURFACE_CONTAINER_LOW,
            border=ft.Border.only(top=ft.BorderSide(1, ft.Colors.OUTLINE_VARIANT)))

    # -- layout ------------------------------------------------------------------

    def _on_resize(self, e) -> None:
        self._layout(e.width)
        for c in self.plot_cards:
            c.measured = False
        self._estimate_plot_width()
        self.page.update()

    def _layout(self, width: float) -> None:
        narrow = width < NARROW
        if narrow:
            self.side.width = None
            self.side.expand = True
            self.side.visible = self.side_visible
            self.main.visible = not self.side_visible
        else:
            self.side.width = 380
            self.side.expand = False
            self.side.visible = self.side_visible
            self.main.visible = True
        self.narrow = narrow
        self.probe_text.visible = width >= 1100
        self.host_field.width = 150 if width < 700 else 190

    def _estimate_plot_width(self) -> None:
        width = self.page.width or 1280
        side = 0 if getattr(self, 'narrow', False) or not self.side_visible else 380
        for c in self.plot_cards:
            c.estimate_size(width - side - 30)

    def _toggle_side(self, e) -> None:
        self.side_visible = not self.side_visible
        self._layout(self.page.width or 1280)

    def _toggle_theme(self, e) -> None:
        dark = self.page.theme_mode != ft.ThemeMode.DARK
        self.page.theme_mode = ft.ThemeMode.DARK if dark else ft.ThemeMode.LIGHT
        self.settings.theme = 'dark' if dark else 'light'
        self.settings.save()
        for c in self.plot_cards:
            c.last_key = None

    def _on_key(self, e) -> None:
        key = getattr(e, 'key', '')
        if getattr(e, 'ctrl', False) and key in ('O', 'o'):
            self.page.run_task(self.guard(self.pick_capture))
        elif getattr(e, 'ctrl', False) and key in ('S', 's'):
            self.page.run_task(self.guard(self.save_workspace))
        elif key == 'F5':
            self.page.run_task(self.guard(self.start_stop))

    def _help(self, e) -> None:
        text = (
            '**Pick signals.** Open the ELF of the traced application, search a variable, '
            'expand structs and arrays, add numeric members (or a whole struct). Raw addresses '
            'work without an ELF.\n\n'
            '**Plot.** Every signal lands on a plot; drag signals between plots, add plots '
            'with the chart button, pick step/line/points and the Y range from a plot\'s menu.\n\n'
            '**Trace live.** Press Start. The probe traces up to two address ranges, chosen '
            'from the selected signals (Capture tab). Every capture is written to the capture '
            'folder while the plots follow the newest data. The first values come from a '
            'memory snapshot taken at start.\n\n'
            '**Navigate.** Mouse wheel over a plot zooms time around the pointer, dragging pans, '
            'double-click shows everything (or resumes following while live). Hover shows '
            'values at that instant in the plot headers. Click sets a marker (the toolbar shows '
            'the time to it), right-click removes it. A plot\'s menu has statistics of the view, '
            'normalize, and splitting or merging plots.\n\n'
            '**Offline.** Open a capture file (.mcds) to analyse it; change the selected '
            'signals at any time. Export PNG or CSV from the toolbar.\n\n'
            '**Keys.** F5 start/stop, Ctrl+O open capture, Ctrl+S save workspace.')
        dlg = ft.AlertDialog(
            title=ft.Text('How to use MCDS Trace'),
            content=ft.Container(ft.Markdown(text), width=560),
            actions=[ft.TextButton('Close', on_click=lambda e: close_dialog(dlg))])
        self.page.show_dialog(dlg)

    # -- files -----------------------------------------------------------------

    async def _pick_path(self, title: str, exts: list[str]) -> str | None:
        files = await ft.FilePicker().pick_files(
            dialog_title=title, file_type=ft.FilePickerFileType.CUSTOM if exts else None,
            allowed_extensions=exts or None, with_data=self.page.web)
        if not files:
            return None
        f = files[0]
        if getattr(f, 'path', None):
            return f.path
        data = getattr(f, 'bytes', None)
        if data:
            # Web mode: the browser sent the contents; keep them next to the server.
            d = os.path.join(tempfile.gettempdir(), 'mcds-trace-uploads')
            os.makedirs(d, exist_ok=True)
            p = os.path.join(d, os.path.basename(f.name))
            with open(p, 'wb') as fh:
                fh.write(data)
            return p
        self.toast('The file could not be read (no path and no contents).', error=True)
        return None

    async def pick_elf(self, e=None) -> None:
        path = await self._pick_path('Open the application ELF', ['elf', 'out', 'axf'])
        if path:
            await self.load_elf(path)

    async def load_elf(self, path: str) -> None:
        self.elf_progress.visible = True
        self.elf_progress.value = 0
        self.elf_text.value = 'Reading %s ...' % os.path.basename(path)
        self.page.update()
        state = {'p': 0.0}
        try:
            table = await asyncio.to_thread(elfsyms.load, path, lambda p: state.__setitem__('p', p))
        except Exception as ex:
            self.elf_progress.visible = False
            self.elf_text.value = 'No ELF loaded'
            raise RuntimeError('reading %s: %s' % (path, ex)) from None
        self.table = table
        self._leaf_counts.clear()
        self.settings.last_elf = path
        self.settings.save()
        self.elf_progress.visible = False
        self.elf_text.value = '%s  (%d variables)' % (os.path.basename(path), len(table.variables))
        self.elf_text.tooltip = path
        self.search.disabled = False
        # Every ELF signal again: a workspace loaded before the ELF has only
        # addresses, and a rebuilt ELF may have moved variables.
        gone, moved = [], 0
        for sid, s in list(self.ws.signals.items()):
            if sid.startswith(('raw:', 'hits:')):
                continue
            node = elfsyms.resolve_path(table.variables, sid)
            if node is None or not node.is_leaf:
                gone.append(s.label)
                continue
            if (node.addr, node.size) != (s.addr, s.size):
                moved += 1
            s.node, s.addr, s.size = node, node.addr, node.size
        self._do_search(self.search.value or '')
        self.tabs.selected_index = 0
        msg = 'Loaded %d variables from %s' % (len(table.variables), os.path.basename(path))
        if moved:
            msg += '; %d signal(s) moved' % moved
        if gone:
            self.toast('%s; not in this ELF: %s%s' % (msg, ', '.join(gone[:3]),
                                                     ' ...' if len(gone) > 3 else ''), error=True)
        else:
            self.toast(msg)
        if moved or gone:
            self.selection_changed()

    def _refresh_recent(self) -> None:
        items = [ft.PopupMenuItem(content='Open a capture file...', icon=ft.Icons.FOLDER_OPEN,
                                  on_click=self.guard(self.pick_capture))]
        recent = [p for p in self.settings.recent_captures if os.path.exists(p)]
        if recent:
            items.append(ft.PopupMenuItem())
            for p in recent[:8]:
                items.append(ft.PopupMenuItem(
                    content=os.path.basename(p), icon=ft.Icons.HISTORY,
                    on_click=self.guard(lambda e, p=p: self.open_capture(p))))
        self.open_menu.items = items

    async def pick_capture(self, e=None) -> None:
        path = await self._pick_path('Open a capture', ['mcds', 'dtrp'])
        if path:
            await self.open_capture(path)

    async def open_capture(self, path: str) -> None:
        if self.mode in ('live', 'starting', 'stopping'):
            self.toast('Stop the live trace first.', error=True)
            return
        if self.mode == 'loading':
            self.toast('Still loading the previous capture.', error=True)
            return
        before = self.mode
        self.mode = 'loading'
        self.busy_text = 'Loading %s' % os.path.basename(path)
        self.progress = 0.0
        fs = FileSession(path)
        try:
            config, _ = await asyncio.to_thread(self._peek_config, path)
            self._signals_for_capture(config)
            self.rebuild_plots()
            self._render_signals()
            self.page.update()
            await asyncio.to_thread(fs.load, self._extract_signals(config),
                                    lambda p: setattr(self, 'progress', p))
        except Exception as ex:
            self.mode = before
            self.progress = None
            self.busy_text = ''
            await asyncio.to_thread(fs.close)
            raise RuntimeError('opening %s: %s' % (path, ex)) from None
        if self.closed:
            await asyncio.to_thread(fs.close)
            return
        self._replace_session(fs)
        self.warned.clear()
        self.mode = 'review'
        self.progress = None
        self.busy_text = ''
        self.settings.add_recent(path)
        self.settings.save()
        self._refresh_recent()
        self.view.follow = False
        self.follow_sw.value = False
        self.fit()
        self.rebuild_plots()
        st = fs.pipe.decoder.stats
        self.toast('%s: %d paragraphs, %d events%s' % (
            os.path.basename(path), fs.stats.records, fs.pipe.log.n,
            ', %d gaps' % st.gaps if st.gaps else ''))
        await self._offer_traced(fs)

    async def _offer_traced(self, fs: FileSession) -> None:
        """Variables the capture has accesses to but the workspace does not show:
        offer them (ticked) in a dialog."""
        if self.table is None or fs.config.get('mode') == 'compact':
            return
        accessed = await asyncio.to_thread(fs.pipe.log.addresses)
        if not accessed:
            return
        spans = sorted((a, a + n) for a, n in accessed)
        starts = np.array([a for a, _ in spans], dtype=np.int64)
        ends = np.maximum.accumulate(np.array([b for _, b in spans], dtype=np.int64))

        def touched(lo: int, hi: int) -> bool:
            """Some access overlaps [lo, hi)."""
            i = int(np.searchsorted(starts, hi)) - 1     # last access starting before hi
            return i >= 0 and ends[i] > lo
        cands = []
        slots = [(parse_int(s.get('addr'), 0), int(s.get('size', 0)))
                 for s in fs.config.get('slots', []) if s.get('enabled')]
        for v in self.table.variables.values():
            if not any(v.addr < lo + n and lo < v.addr + max(v.type.size, 1) for lo, n in slots):
                continue
            for leaf in elfsyms.node_of(v).leaves(256):
                if leaf.path in self.ws.signals:
                    continue
                if touched(leaf.addr, leaf.addr + leaf.size):
                    cands.append(leaf)
            if len(cands) >= 64:
                break
        if not cands:
            return
        boxes = [ft.Checkbox(label='%s  (%s)' % (short_path(n.path), n.type_name()), value=True,
                             data=n) for n in cands]

        def add(e):
            close_dialog(dlg)
            chosen = [b.data for b in boxes if b.value]
            for n in chosen:
                self.ws.add_signal(leaf_signal(n), -1)
            if chosen:
                self.selection_changed()
            self.page.update()
        dlg = ft.AlertDialog(
            title=ft.Text('More traced variables'),
            content=ft.Container(ft.Column(
                [ft.Text('This capture also has accesses to:', size=13)] + boxes,
                scroll=ft.ScrollMode.AUTO, tight=True), width=480, height=min(420, 60 + 36 * len(boxes))),
            actions=[ft.TextButton('Not now', on_click=lambda e: close_dialog(dlg)),
                     ft.FilledButton('Add', on_click=add)])
        self.page.show_dialog(dlg)

    @staticmethod
    def _peek_config(path: str) -> tuple[dict, int]:
        from ..capfile import FILE_MAGIC
        import struct as _struct
        with open(path, 'rb') as f:
            head = f.read(len(FILE_MAGIC) + 4)
            if not head.startswith(FILE_MAGIC):
                return {}, 0
            (n,) = _struct.unpack_from('<I', head, len(FILE_MAGIC))
            return json.loads(f.read(n)), n

    def _signals_for_capture(self, config: dict) -> None:
        """Make sure the workspace shows something of this capture: keep the
        current signals that fall inside its watch ranges, otherwise derive
        signals from the ranges (ELF members, or raw words)."""
        slots = [(j, parse_int(s.get('addr'), 0), int(s.get('size', 0)), s.get('name', ''))
                 for j, s in enumerate(config.get('slots', [])) if s.get('enabled')]
        inside = [s for s in self.ws.value_signals()
                  if any(lo <= s.addr < lo + n for _, lo, n, _ in slots)]
        if config.get('mode') == 'compact':
            for j, lo, n, name in slots:
                if not any(lo <= s.addr < lo + n for s in inside):
                    self.ws.add_signal(hits_signal(j, name or 'slot %d' % j),
                                       self.ws.subplots[0].id if self.ws.subplots else None)
            return
        if inside:
            if not any(s.id in p.signals for p in self.ws.subplots for s in inside):
                for s in inside:
                    self.ws.add_signal(s)
            return
        for j, lo, n, name in slots:
            added = 0
            if self.table is not None:
                for v in self.table.variables.values():
                    if v.addr < lo + n and lo < v.addr + max(v.type.size, 1):
                        for leaf in elfsyms.node_of(v).leaves(64):
                            if lo <= leaf.addr < lo + n and added < 12:
                                self.ws.add_signal(leaf_signal(leaf), -1)   # own scale each
                                added += 1
            if not added:
                size = n if n in (1, 2, 4) else 4
                self.ws.add_signal(raw_signal(lo, size, name or None), -1)

    def _extract_signals(self, config: dict) -> list[Signal]:
        sigs = [s for s in self.ws.signals.values() if s.kind == 'value']
        if config.get('mode') == 'compact':
            for j, sl in enumerate(config.get('slots', [])):
                if sl.get('enabled'):
                    sigs.append(hits_signal(j, sl.get('name') or 'slot %d' % j))
        return sigs

    async def save_workspace(self, e=None) -> None:
        data = json.dumps(self.ws.to_json(self.table.path if self.table else ''), indent=1).encode()
        name = 'workspace.mcdsws'
        if self.page.web:
            await ft.FilePicker().save_file(file_name=name, src_bytes=data)
            return
        path = await ft.FilePicker().save_file(dialog_title='Save the workspace', file_name=name,
                                               initial_directory=os.path.dirname(
                                                   self.settings.last_workspace) or None)
        if not path:
            return
        with open(path, 'wb') as f:
            f.write(data)
        self.settings.last_workspace = path
        self.settings.save()
        self.toast('Workspace saved to %s' % path)

    async def load_workspace(self, e=None) -> None:
        path = await self._pick_path('Load a workspace', ['mcdsws', 'json'])
        if path:
            await self.open_workspace(path)

    async def open_workspace(self, path: str) -> None:
        with open(path) as f:
            d = json.load(f)
        if d.get('format') != 'mcds-trace-workspace':
            raise ValueError('%s is not an MCDS Trace workspace' % path)
        elf = d.get('elf')
        if elf and (self.table is None or self.table.path != elf) and os.path.exists(elf):
            await self.load_elf(elf)
        ws, missing = Workspace.from_json(d, self.table)
        self.ws = ws
        self.settings.last_workspace = path
        self.settings.save()
        self._sync_capture_controls()
        self.selection_changed()
        if missing:
            self.toast('%d signal(s) not found in the ELF: %s' % (len(missing), ', '.join(missing[:3])),
                       error=True)

    def _sync_capture_controls(self) -> None:
        c = self.ws.capture
        self.cap_source.value, self.cap_cpu.value, self.cap_mode.value = c.source, str(c.cpu), c.mode
        self.cap_access.value, self.cap_payload.value, self.cap_ts.value = c.access, c.payload, c.timestamps
        self.cap_div.value, self.cap_wide.value, self.cap_masters.value = str(c.dap_div), c.wide, c.masters
        self.cap_duration.value = '%g' % c.duration

    async def _pick_dir(self, e=None) -> None:
        if self.page.web:
            self.toast('Type the folder path (browsers cannot pick server folders).')
            return
        d = await ft.FilePicker().get_directory_path(dialog_title='Capture folder')
        if d:
            self.cap_dir.value = d
            self._dir_changed(None)

    # -- signals & plots ---------------------------------------------------------

    def add_leaf(self, node: elfsyms.Node) -> None:
        if not node.is_leaf:
            return
        if node.path in self.ws.signals:
            self.toast('%s is already selected' % node.path)
            return
        # Its own plot: separately picked variables rarely share a scale.
        # (Drag it onto another plot, or merge plots, to overlay them.)
        empty = next((p for p in self.ws.subplots if not p.signals), None)
        self.ws.add_signal(leaf_signal(node), empty.id if empty else -1)
        self.selection_changed()

    def add_all(self, node: elfsyms.Node) -> None:
        leaves = list(node.leaves(1025))
        if len(leaves) > 64:
            def go(e):
                close_dialog(dlg)
                self._add_leaves(leaves[:1024], node)
                self.page.update()
            dlg = ft.AlertDialog(
                title=ft.Text('Add %d signals?' % len(leaves)),
                content=ft.Text('%s has %d numeric members. They go on one new plot.'
                                % (node.path, len(leaves))),
                actions=[ft.TextButton('Cancel', on_click=lambda e: close_dialog(dlg)),
                         ft.FilledButton('Add', on_click=go)])
            self.page.show_dialog(dlg)
            return
        self._add_leaves(leaves, node)

    def _add_leaves(self, leaves, node) -> None:
        first = True
        for leaf in leaves:
            if leaf.path in self.ws.signals:
                continue
            self.ws.add_signal(leaf_signal(leaf), -1 if first else None)
            first = False
        self.selection_changed()

    def _add_raw(self, e) -> None:
        a = parse_int(self.raw_addr.value)
        if a is None:
            self.toast('Enter an address, e.g. 0x5000220C', error=True)
            return
        size = parse_int(self.raw_size.value, 4)
        if size not in (1, 2, 4, 8) or a < 0 or a + size > 1 << 32:
            self.toast('The address must lie in 0x00000000..0xFFFFFFFF (size 1, 2, 4 or 8)',
                       error=True)
            return
        empty = next((p for p in self.ws.subplots if not p.signals), None)
        self.ws.add_signal(raw_signal(a, size, signed=bool(self.raw_signed.value)),
                           empty.id if empty else -1)
        self.selection_changed()

    def remove_signal(self, sid: str) -> None:
        self.ws.remove_signal(sid)
        self.selection_changed()

    def clear_signals(self) -> None:
        self.ws.signals.clear()
        for p in self.ws.subplots:
            p.signals.clear()
        self.selection_changed()

    def plot_signal(self, sid: str, pid: int) -> None:
        s = self.ws.signals.get(sid)
        if s is None:
            return
        self.ws.add_signal(s, pid)
        self.selection_changed(retrace=False)

    def unplot(self, pid: int, sid: str) -> None:
        p = self.ws.plot(pid)
        if p and sid in p.signals:
            p.signals.remove(sid)
        self.selection_changed(retrace=False)

    def add_plot(self) -> None:
        self.ws.new_plot()
        self.rebuild_plots()
        self._render_signals()

    def remove_plot(self, pid: int) -> None:
        self.ws.remove_plot(pid)
        self.rebuild_plots()
        self._render_signals()

    def move_plot(self, pid: int, delta: int) -> None:
        self.ws.move_plot(pid, delta)
        self.rebuild_plots()

    def split_plot(self, pid: int) -> None:
        self.ws.split_plot(pid)
        self.selection_changed(retrace=False)

    def merge_up(self, pid: int) -> None:
        self.ws.merge_up(pid)
        self.selection_changed(retrace=False)

    def selection_changed(self, retrace: bool = True) -> None:
        """Signals or their plots changed: refresh lists, plan and data."""
        self._render_signals()
        self._render_plan()
        self._render_tree()
        self.rebuild_plots()
        if retrace and self.session is not None and self.session.pipe is not None:
            self.page.run_task(self._reselect)

    async def _reselect(self) -> None:
        sess = self.session
        if sess is None or sess.pipe is None:
            return
        sigs = self._extract_signals(sess.config)
        # Signals the session did not watch (completely) will stay empty: say so once.
        ranges = [(parse_int(s.get('addr'), 0), int(s.get('size', 0)))
                  for s in sess.config.get('slots', []) if s.get('enabled')]
        outside = [s for s in sigs if s.kind == 'value' and s.id not in self.warned
                   and not any(lo <= s.addr and s.addr + s.size <= lo + n for lo, n in ranges)]
        if outside:
            self.warned.update(s.id for s in outside)
            names = ', '.join(s.label for s in outside[:3])
            self.toast(('%s: not in this trace\'s watch ranges - restart to trace %s.'
                        if isinstance(sess, LiveSession) else
                        '%s: outside the watch ranges of this capture (no data for %s).')
                       % (names, 'them' if len(outside) > 1 else 'it'))
        # The worker answers in order, so the newest selection is applied
        # last; only its completion clears the status.
        self._select_gen = gen = getattr(self, '_select_gen', 0) + 1
        self.busy_text = 'Re-extracting signals'
        try:
            await asyncio.to_thread(sess.pipe.reselect, sigs)
        finally:
            if gen == self._select_gen:
                self.busy_text = ''
        for c in self.plot_cards:
            c.last_key = None

    def rebuild_plots(self) -> None:
        cards = {c.model.id: c for c in self.plot_cards}
        new = []
        for sp in self.ws.subplots:
            c = cards.get(sp.id)
            if c is None:
                c = PlotCard(self, sp)
            else:
                c.model = sp
                c.body.height = sp.height
                c.rebuild_legend()
            new.append(c)
        self.plot_cards = new
        self._estimate_plot_width()
        self.plots.controls = [c.control for c in new]
        empty = not new or not any(sp.signals for sp in self.ws.subplots)
        self.empty_state.visible = empty
        self.plots.visible = not empty
        for c in new:
            c.last_key = None

    # -- view ------------------------------------------------------------------

    def view_changed(self) -> None:
        self.follow_sw.value = self.view.follow
        try:
            self.follow_sw.update()
        except Exception:
            pass

    def set_cursor(self, x: float | None) -> None:
        if x == self.view.cursor:
            return
        self.view.cursor = x
        self._cursor_readout()

    def set_marker(self, x: float | None) -> None:
        """Click a plot: a reference line; the readout then shows the delta to it.
        Right-click removes it."""
        self.view.marker = x
        self._cursor_readout()

    def _cursor_readout(self) -> None:
        x, m = self.view.cursor, self.view.marker
        parts = []
        if x is not None:
            parts.append('t %s' % fmt_time(x))
        if m is not None:
            parts.append('marker %s' % fmt_time(m))
            if x is not None:
                d = x - m
                parts.append('dt %s (%.6g Hz)' % (fmt_time(d), 1 / abs(d)) if d else 'dt 0')
        self.cursor_text.value = '   '.join(parts)

    def fit(self) -> None:
        store = self.store()
        if store is None:
            return
        lo, hi = store.span()
        if hi <= lo:
            hi = lo + 1e-3
        pad = (hi - lo) * 0.02
        self.view.set(lo - pad, hi + pad)

    def fit_or_follow(self) -> None:
        if self.mode == 'live':
            self.view.follow = True
            self.view_changed()
        else:
            self.fit()

    def _zoom(self, factor: float) -> None:
        if self.mode == 'live' and self.view.follow:
            w = max(1e-4, self.view.window * factor)
            self.view.window = w
            self.window_dd.value = min(('%g' % x for x in WINDOWS), key=lambda s: abs(float(s) - w))
            return
        self.view.zoom(factor)

    def _follow_changed(self, e) -> None:
        self.view.follow = bool(self.follow_sw.value)

    def _window_changed(self, e) -> None:
        try:
            w = float(self.window_dd.value)
        except (TypeError, ValueError):
            return
        self.view.window = w
        self.settings.window_s = w
        self.settings.save()
        if self.mode != 'live' or not self.view.follow:
            c = self.view.cursor if self.view.cursor is not None else (self.view.x0 + self.view.x1) / 2
            self.view.set(c - w / 2, c + w / 2)

    # -- probe & capture -------------------------------------------------------

    def _host_changed(self, e) -> None:
        host = (self.host_field.value or '').strip()
        if host and host != self.host:
            self.host = host
            self.settings.host = host
            self.settings.save()
            self.probe_ok = None
            self.last_status_poll = 0

    def probe(self) -> Probe:
        return Probe(self.host, self.settings.auth)

    async def _poll_probe(self) -> None:
        try:
            st = await asyncio.to_thread(self.probe().status)
            self.probe_ok = True
            cores = ' '.join(p for p in st.split() if p.startswith('cpu'))
            self.probe_text.value = cores.replace('=', ' ')[:60]
        except ProbeError as ex:
            self.probe_ok = False
            self.probe_text.value = 'unreachable'
            self.probe_dot.tooltip = str(ex)
        self.probe_dot.bgcolor = {True: ft.Colors.GREEN, False: ft.Colors.RED}.get(self.probe_ok,
                                                                                   ft.Colors.OUTLINE)
        if self.probe_ok:
            self.probe_dot.tooltip = 'probe reachable'

    async def start_stop(self, e=None) -> None:
        if self.mode == 'live':
            await self.stop_trace()
        elif self.mode in ('idle', 'review'):
            await self.start_trace()

    async def start_trace(self) -> None:
        cfg, plan, warnings = self.ws.probe_config()
        if not plan.ranges:
            self.toast('Select at least one signal first (Symbols tab).', error=True)
            self.tabs.selected_index = 0
            return
        os.makedirs(self.settings.capture_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        path = os.path.join(self.settings.capture_dir, 'trace-%s.mcds' % stamp)
        sess = LiveSession(self.probe(), cfg, self.ws.trace_signals(plan), path)
        self.mode = 'starting'
        self.busy_text = 'Starting the trace (DAP calibration, memory snapshot)'
        self._update_start_buttons()
        self.page.update()
        self._pending = sess                   # shutdown stops it even while starting
        try:
            await asyncio.to_thread(sess.start)
        except Exception as ex:
            self.mode = 'idle'
            self.busy_text = ''
            self._update_start_buttons()
            raise RuntimeError(str(ex)) from None
        finally:
            self._pending = None
        if self.closed:
            await asyncio.to_thread(sess.close, 15)
            return
        self._replace_session(sess)
        self.warned.clear()
        self._auto_stopping = False
        self.mode = 'live'
        self.busy_text = ''
        self.view.follow = True
        self.follow_sw.value = True
        self._update_start_buttons()
        self.rebuild_plots()
        wide = sess.config.get('wide_active')
        self.toast('Tracing%s - recording to %s' % ('' if wide else ' (narrow DAP)', path))

    async def stop_trace(self) -> None:
        sess = self.session
        if not isinstance(sess, LiveSession):
            return
        self.mode = 'stopping'
        self.busy_text = 'Stopping: collecting the last paragraphs'
        self._update_start_buttons()
        self.page.update()
        await asyncio.to_thread(sess.stop)
        self.mode = 'review'
        self.busy_text = ''
        self._update_start_buttons()
        self.view.follow = False
        self.follow_sw.value = False
        self.fit()
        self.settings.add_recent(sess.out_path)
        self.settings.save()
        self._refresh_recent()
        msg = 'Stopped. %s in %s' % (fmt_bytes(sess.stats.bytes), sess.out_path)
        if sess.error:
            self.toast('%s (%s)' % (msg, sess.error), error=True)
        else:
            self.toast(msg)
        # The live view keeps a window; the file has everything.  Review that.
        store = sess.store
        partial = (sess.stats.decode_skipped > 0 or (sess.pipe and sess.pipe.log.dropped > 0)
                   or (store is not None and store.trimmed))
        if partial and not sess.error and os.path.exists(sess.out_path):
            await self.open_capture(sess.out_path)

    def _update_start_buttons(self) -> None:
        live = self.mode == 'live'
        busy = self.mode in ('starting', 'stopping', 'loading')
        for b in (self.start_btn, self.cap_start):
            b.content = 'Stop' if live else ('Start tracing' if b is self.cap_start else 'Start')
            b.icon = ft.Icons.STOP if live else ft.Icons.PLAY_ARROW
            b.disabled = busy
            b.style = ft.ButtonStyle(bgcolor=ft.Colors.ERROR if live else None,
                                     color=ft.Colors.ON_ERROR if live else None)
        for ctl in (self.cap_source, self.cap_cpu, self.cap_mode, self.cap_access,
                    self.cap_payload, self.cap_ts, self.cap_div, self.cap_wide, self.cap_masters):
            ctl.disabled = live or busy
        if not (live or busy):
            self._cap_changed(None)

    # -- export ----------------------------------------------------------------

    async def export_png(self, e=None) -> None:
        if not self.plot_cards or self.store() is None:
            self.toast('Nothing to export yet.', error=True)
            return
        specs = [c.spec() for c in self.plot_cards if c.model.signals]
        for s in specs:
            s.cursor = None
            s.width = max(s.width, 1000)
        fd, tmp = tempfile.mkstemp(suffix='.png')
        os.close(fd)
        await asyncio.to_thread(export_png, tmp, specs, APP_TITLE)
        with open(tmp, 'rb') as f:
            data = f.read()
        os.unlink(tmp)
        await self._save_bytes(data, 'mcds-plot.png', 'Export PNG')

    async def export_csv(self, e=None) -> None:
        store = self.store()
        if store is None:
            self.toast('Nothing to export yet.', error=True)
            return
        x0, x1 = self.view.x0, self.view.x1
        rows = []
        for s in self.ws.plotted():
            t, v = store.snapshot(self.trace_id(s))
            m = (t >= x0) & (t <= x1)
            for tt, vv in zip(t[m].tolist(), v[m].tolist()):
                rows.append((tt, s.label, '' if vv != vv else (int(vv) if float(vv).is_integer() else vv)))
        rows.sort(key=lambda r: r[0])
        import io
        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(['time_s', 'signal', 'value'])
        for r in rows:
            w.writerow(['%.9f' % r[0], r[1], r[2]])
        await self._save_bytes(buf.getvalue().encode(), 'mcds-samples.csv', 'Export CSV')

    async def _save_bytes(self, data: bytes, name: str, title: str) -> None:
        if self.page.web:
            await ft.FilePicker().save_file(file_name=name, src_bytes=data)
            return
        path = await ft.FilePicker().save_file(dialog_title=title, file_name=name,
                                               initial_directory=self.settings.capture_dir)
        if not path:
            return
        with open(path, 'wb') as f:
            f.write(data)
        self.toast('Saved %s' % path)

    # -- periodic refresh ------------------------------------------------------

    async def tick(self) -> None:
        """UI loop: view window, plot frames, legends, status (runs forever)."""
        last_legend = last_full = 0.0
        last_trim = time.monotonic()
        # MCDS_TRACE_PERF=1: frame times on stderr every 5 s.
        perf = [] if os.environ.get('MCDS_TRACE_PERF') else None
        phases: list[tuple] = []
        perf_t = time.monotonic()
        while not self.closed:
            t_start = time.monotonic()
            try:
                store = self.store()
                if self.mode == 'live' and store is not None and self.view.follow:
                    end = store.t_end
                    self.view.set(end - self.view.window, end + self.view.window * 0.02)
                if self.mode == 'live' and isinstance(self.session, LiveSession) and \
                        self.ws.capture.duration > 0 and not self._auto_stopping and \
                        t_start - self.session.stats.started >= self.ws.capture.duration:
                    self._auto_stopping = True
                    self.page.run_task(self.guard(self.start_stop))
                if self.mode == 'live' and store is not None and t_start - last_trim > 2.0:
                    last_trim = t_start
                    await asyncio.to_thread(store.trim, self.settings.history_s)
                t_a = time.monotonic()
                for c in self.plot_cards:
                    c.schedule()
                t_b = time.monotonic()
                if t_start - last_legend > 0.2:
                    last_legend = t_start
                    for c in self.plot_cards:
                        c.update_values(self.view.cursor)
                        c.update_stats()
                    self._update_status()
                    if t_start - self.last_status_poll > (5.0 if self.probe_ok else 3.0) \
                            and self.mode not in ('live', 'starting', 'stopping'):
                        self.last_status_poll = t_start
                        self.page.run_task(self._poll_probe)
                    t_c = time.monotonic()
                    # Only what the loop changed; a whole-page diff costs tens
                    # of milliseconds with a big symbol tree.  A full update
                    # now and then catches anything else.
                    if t_start - last_full > 2.0:
                        last_full = t_start
                        self.page.update()
                    else:
                        for c in self.plot_cards:
                            c.push_readouts()
                        for ctl in (self.statusbar, self.stats_view):
                            try:
                                ctl.update()
                            except (RuntimeError, AssertionError):
                                pass
                    if perf is not None:
                        phases.append((t_a - t_start, t_b - t_a, t_c - t_b,
                                       time.monotonic() - t_c))
            except Exception:
                traceback.print_exc()
            dt = time.monotonic() - t_start
            if perf is not None:
                perf.append(dt)
                if t_start - perf_t >= 5.0:
                    print('tick: %.1f frames/s, %.1f ms mean, %.1f ms max (%s)' % (
                        len(perf) / (t_start - perf_t), 1e3 * sum(perf) / len(perf),
                        1e3 * max(perf), self.mode), file=sys.stderr, flush=True)
                    if phases:
                        print('  phases ms: pre %.1f  plots %.1f  legend+stats %.1f  update %.1f'
                              % tuple(1e3 * sum(p[i] for p in phases) / len(phases)
                                      for i in range(4)), file=sys.stderr, flush=True)
                        phases.clear()
                    perf.clear()
                    perf_t = t_start
            await asyncio.sleep(max(0.01, 1.0 / max(1, self.settings.fps) - dt))

    def _update_status(self) -> None:
        sess = self.session
        self.busy_bar.visible = bool(self.busy_text) or self.progress is not None
        self.busy_bar.value = self.progress
        if self.busy_text:
            self.status_left.value = self.busy_text + (' %d%%' % (self.progress * 100)
                                                       if self.progress is not None else '...')
        elif isinstance(sess, LiveSession) and self.mode == 'live':
            st = sess.stats
            p = st.probe or {}
            lost = int(p.get('lost', 0))
            self.status_left.value = 'LIVE  %s/s  %s recorded  lost %d  %s' % (
                fmt_bytes(st.rate), fmt_bytes(st.bytes), lost,
                ('decoder behind by %s' % fmt_bytes(st.backlog)) if st.backlog > 2 << 20 else '')
            if sess.error:
                self.status_left.value += '  ERROR: %s' % sess.error
        elif sess is not None:
            path = getattr(sess, 'path', None) or getattr(sess, 'out_path', '')
            store = sess.store
            self.status_left.value = '%s  %s  %s' % (
                os.path.basename(path), fmt_bytes(sess.stats.bytes),
                ('%d samples' % store.samples) if store else '')
        else:
            self.status_left.value = 'Ready - %s' % ('ELF: ' + os.path.basename(self.table.path)
                                                     if self.table else 'no ELF loaded')
        store = self.store()
        if store is not None:
            ex = sess.pipe.extractor.stats if sess and sess.pipe else None
            self.status_right.value = 'view %s .. %s%s' % (
                fmt_time(self.view.x0), fmt_time(self.view.x1),
                ('  gaps %d' % len(store.gaps)) if store.gaps else '')
            if ex and ex.lost:
                self.status_right.value += '  target overflow: %d msgs' % ex.lost
        else:
            self.status_right.value = ''
        if isinstance(sess, LiveSession):
            p = sess.stats.probe or {}
            self.stats_view.value = (
                'received   %s (%s/s)\nparagraphs %s  lost %s  laps %s\n'
                'decoded    %d records, backlog %s' % (
                    fmt_bytes(sess.stats.bytes), fmt_bytes(sess.stats.rate),
                    p.get('paragraphs', '-'), p.get('lost', '-'), p.get('laps', '-'),
                    sess.stats.records, fmt_bytes(max(0, sess.stats.backlog))))

    # -- lifecycle -------------------------------------------------------------

    async def start(self) -> None:
        self.build()
        self.page.update()
        self.page.run_task(self.tick)
        elf = self.args.elf or self.settings.last_elf
        if elf and os.path.exists(elf):
            try:
                await self.load_elf(elf)
            except Exception as ex:
                self.toast(str(ex), error=True)
        if self.args.workspace:
            await self.guard(lambda e: self.open_workspace(self.args.workspace))()
        if self.args.capture:
            await self.guard(lambda e: self.open_capture(self.args.capture))()
        self.page.update()

    def _replace_session(self, new) -> None:
        """Make `new` the session; the one before goes (its worker ends)."""
        old, self.session = self.session, new
        if old is not None and old is not new:
            threading.Thread(target=_close_session, args=(old,), daemon=True).start()

    def shutdown(self, wait: bool = True) -> None:
        """The window or browser session closed: stop a live trace (the file
        is kept complete), end the workers, save the settings."""
        if self.closed:
            return
        self.closed = True
        try:
            atexit.unregister(self.shutdown)
        except Exception:
            pass
        sessions = [s for s in (self.session, self._pending) if s is not None]

        def stop_all():
            for sess in sessions:
                if sess is self._pending:
                    # Still starting: wait for it, then stop it.
                    for _ in range(300):
                        if sess.running or sess.error:
                            break
                        time.sleep(0.1)
                _close_session(sess)

        if wait:
            stop_all()
        else:
            threading.Thread(target=stop_all, name='shutdown').start()
        self.settings.save()


def _close_session(sess) -> None:
    try:
        if isinstance(sess, LiveSession):
            sess.close(timeout=15)
        else:
            sess.close()
    except Exception:
        if isinstance(sess, LiveSession):
            try:
                sess.abort()
            except Exception:
                pass


def main(argv=None) -> int:
    multiprocessing.freeze_support()        # frozen builds start their workers through here
    p = argparse.ArgumentParser(prog='mcds-trace-ui', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('capture', nargs='?', help='open this capture file')
    p.add_argument('--host', help='probe address (saved)')
    p.add_argument('--elf', help='load symbols from this ELF')
    p.add_argument('--workspace', help='load this workspace (.mcdsws)')
    p.add_argument('--web', action='store_true', help='serve in the browser instead of a window')
    p.add_argument('--serve', action='store_true',
                   help='like --web, but only serve (open http://HOST:PORT yourself)')
    p.add_argument('--port', type=int, default=8550, help='web port (with --web / --serve)')
    p.add_argument('--bind', default=None, help='web interface to listen on (default: all)')
    args = p.parse_args(argv)

    web = args.web or args.serve

    async def app_main(page: ft.Page):
        app = App(page, args)
        # A browser reload or a network drop only disconnects: the session
        # reconnects and carries on.  Closing ends it (and a live trace).
        page.on_close = lambda e: app.shutdown(wait=False)
        if not web:
            atexit.register(app.shutdown)   # desktop: the window closed
        await app.start()

    if web:
        if args.serve:
            os.environ['BROWSER'] = 'true'      # webbrowser.open: a no-op command
        ft.run(app_main, view=ft.AppView.WEB_BROWSER, port=args.port, host=args.bind)
    else:
        ft.run(app_main)
    return 0


if __name__ == '__main__':
    sys.exit(main())
