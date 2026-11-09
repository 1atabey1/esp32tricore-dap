"""Program trace recorder: a small window over `tdm_dap.py trace --program`
and `mcds-trace program`.

Records a PTU_TC program trace from the tdm probe on USB into a capture file,
with the options of the command line (CPU, flow or calls mode, window,
exclude, bursts, compact function trace, DAP clock), and analyses it against
the ELF (coverage HTML and lcov, Perfetto trace).  An ELF browser lists the
functions (demangled); the selected ones become the window or the excluded
range.  The hardware has one comparator for each, so a selection of several
functions becomes the one range that spans them, and the window says which
other functions that range takes in.

    uv run mcds-trace-ptrace
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass

SETTINGS = os.path.join(os.environ.get('XDG_CACHE_HOME') or
                        os.path.join(os.path.expanduser('~'), '.cache'),
                        'mcds-trace', 'ptrace_ui.json')


# -- ranges from functions (no Tk: tested on their own) -------------------------------

@dataclass
class Span:
    lo: int
    hi: int                      # inclusive
    selected: int                # functions picked
    covered: list                # (name, start) of the other functions inside the span


def span_of(code, picked: list[int]) -> Span | None:
    """The one address range that holds the functions `picked` (indices into
    the code model), and what else it takes in."""
    if not picked:
        return None
    lo = min(int(code.func_start[k]) for k in picked)
    hi = max(int(code.func_end[k]) for k in picked) - 1
    chosen = set(picked)
    covered = [(code.func_name[k], int(code.func_start[k]))
               for k in range(len(code.func_name))
               if k not in chosen and int(code.func_start[k]) <= hi
               and int(code.func_end[k]) > lo]
    return Span(lo, hi, len(picked), covered)


def parse_range(text: str) -> tuple[int, int] | None:
    """'0x80051d06:0x80051d07' (or '-') -> (lo, hi); None when empty."""
    text = text.strip()
    if not text:
        return None
    sep = ':' if ':' in text else '-'
    lo, _, hi = text.partition(sep)
    lo, hi = int(lo.strip(), 0), int(hi.strip(), 0)
    if hi < lo:
        raise ValueError('%s: the end is below the start' % text)
    return lo, hi


def fmt_range(lo: int, hi: int) -> str:
    return '0x%08x:0x%08x' % (lo, hi)


def build_config(cpu: int, mode: str, window, exclude, bursts, cft) -> dict:
    """The JSON configuration tdm.pack_config takes, program trace only."""
    prog = {'mode': mode, 'window': list(window) if window else None,
            'exclude': list(exclude) if exclude else None,
            'cft_vshort': cft[0] if cft else 0, 'cft_short': cft[1] if cft else 0,
            'duty_on_us': bursts[0] if bursts else 0,
            'duty_period_us': bursts[1] if bursts else 0}
    return {'source': 'cpu', 'cpu': cpu, 'mode': 'full', 'timestamps': 'hit',
            'wide': True, 'fast': True, 'slots': [], 'program': prog}


# -- the work, off the window's thread ----------------------------------------------------

def record(cfg: dict, out_path: str, seconds: float, clock: str, stop: threading.Event,
           q: queue.Queue):
    """Post the configuration, start, write the stream into a capture file until
    `stop` or `seconds` (0: until stop), stop, write what is still in flight."""
    from .capfile import write_header
    from .tdm import TdmProbe
    try:
        with TdmProbe() as p:
            p.drain()
            p.post_config(cfg)
            auto = clock == 'auto'
            q.put(('log', 'starting (the auto clock tries the bins: up to a second)' if auto
                   else 'starting on the %s MHz bin' % clock))
            reply = p.start(auto=auto, bin_mhz=None if auto else int(clock))
            if not reply.startswith('started'):
                raise RuntimeError('the probe did not start the trace: %s' % reply)
            info = p.config()
            q.put(('log', 'tracing: %s, %s MHz bin%s' % (
                'wide' if info.get('wide_active') else 'narrow', info.get('bin_mhz', '?'),
                ', fast' if info.get('fast_active') else '')))
            n = 0
            t0 = time.monotonic()
            last = t0
            with open(out_path, 'wb') as f:
                write_header(f, info)
                try:
                    while not stop.is_set() and not (seconds and time.monotonic() - t0 >= seconds):
                        b = p.read(200)
                        if b:
                            f.write(b)
                            n += len(b)
                        if time.monotonic() - last >= 0.25:
                            last = time.monotonic()
                            q.put(('progress', n, last - t0))
                finally:
                    stats = p.stop()
                    quiet = time.monotonic()
                    while time.monotonic() - quiet < 1.5:
                        b = p.read(200)
                        if b:
                            f.write(b)
                            n += len(b)
                            quiet = time.monotonic()
            dt = max(time.monotonic() - t0, 1e-6)
            q.put(('recorded', out_path, n, dt, stats))
    except Exception as e:                   # noqa: BLE001 - shown in the window
        q.put(('error', 'recording: %s' % e))


def analyse(path: str, elf: str, code, window, q: queue.Queue):
    from . import capfile, codemodel, ptrace
    from . import ptrace_export as px
    try:
        if code is None:
            q.put(('log', 'reading %s ...' % os.path.basename(elf)))
            code = codemodel.load(elf)
        config, stream = capfile.read_file(path)
        recs = list(capfile.records(stream))
        hz = float(config.get('emu_hz') or 0)
        t0 = time.monotonic()
        dec = ptrace.decode_records(recs, code, hz)
        cov = px.coverage(dec, window)
        base = os.path.splitext(path)[0]
        title = '%s - %s' % (os.path.basename(path), os.path.basename(elf))
        px.write_lcov(cov, base + '.info')
        px.write_html(cov, base + '.html', title, dec.stats.summary())
        cpu = int(config.get('cpu', 0) or 0)
        n = px.write_perfetto(dec, base + '.pftrace', cpu)
        text = '%s mode, %d records, decoded in %.1f s\n%s\n%s\n%d slices -> %s.pftrace' % (
            'calls' if dec.calls_mode else 'flow', len(recs), time.monotonic() - t0,
            dec.stats.summary(), px.summary_text(cov, 10), n, os.path.basename(base))
        q.put(('analysed', base, text, px.hot_loops(dec, 8)))
    except Exception as e:                   # noqa: BLE001
        q.put(('error', 'analysis: %s' % e))


def load_code(elf: str, q: queue.Queue):
    from . import codemodel
    try:
        q.put(('code', elf, codemodel.load(elf)))
    except Exception as e:                   # noqa: BLE001
        q.put(('error', 'reading %s: %s' % (elf, e)))


def probe_call(what: str, q: queue.Queue):
    from .tdm import TdmProbe
    try:
        with TdmProbe() as p:
            if what == 'power':
                st, d = p.cmd('POWER24', bytes([1]))
                time.sleep(1.0)
            q.put(('status', p.status()))
    except Exception as e:                   # noqa: BLE001
        q.put(('status', 'no probe: %s' % e))


def open_path(path: str) -> None:
    if sys.platform == 'win32':
        os.startfile(path)                   # noqa: S606
    else:
        subprocess.Popen(['xdg-open', path])


# -- the window ----------------------------------------------------------------------

class App:
    def __init__(self):
        import tkinter as tk
        from tkinter import ttk
        if sys.platform == 'win32':
            try:                             # sharp text, and sizes in real pixels
                import ctypes
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except (AttributeError, OSError):
                pass
        self.tk, self.ttk = tk, ttk
        self.root = tk.Tk()
        self.root.title('Program Trace')
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        w, h = min(int(sw * 0.85), 1700), min(int(sh * 0.85), 1050)
        self.root.geometry('%dx%d+%d+%d' % (w, h, (sw - w) // 2, (sh - h) // 3))
        self.q: queue.Queue = queue.Queue()
        self.code = None
        self.code_elf = ''
        self.rows: list[int] = []            # tree item -> function index
        self.loops: list = []
        self.stop_ev: threading.Event | None = None
        self.last_base = ''
        self.settings = self._load_settings()
        self._build()
        self.root.after(100, self._pump)
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        if self.settings.get('elf') and os.path.exists(self.settings['elf']):
            self._load_elf(self.settings['elf'])
        threading.Thread(target=probe_call, args=('status', self.q), daemon=True).start()

    # settings ----------------------------------------------------------------
    def _load_settings(self) -> dict:
        try:
            with open(SETTINGS, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _save_settings(self) -> None:
        s = {k: v.get() for k, v in self.vars.items()}
        s['elf'] = self.elf_var.get()
        try:
            os.makedirs(os.path.dirname(SETTINGS), exist_ok=True)
            with open(SETTINGS, 'w', encoding='utf-8') as f:
                json.dump(s, f, indent=1)
        except OSError:
            pass

    # layout ------------------------------------------------------------------
    def _build(self):
        tk, ttk = self.tk, self.ttk
        st = self.settings
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)
        panes = ttk.PanedWindow(root, orient='horizontal')
        panes.grid(row=1, column=0, sticky='nsew', padx=6, pady=3)

        # probe bar
        bar = ttk.Frame(root, padding=6)
        bar.grid(row=0, column=0, sticky='ew')
        self.status_var = tk.StringVar(value='probe: ...')
        ttk.Label(bar, textvariable=self.status_var).pack(side='left')
        ttk.Button(bar, text='24 V on', command=lambda: self._probe('power')).pack(side='right')
        ttk.Button(bar, text='Refresh', command=lambda: self._probe('status')).pack(side='right',
                                                                                  padx=4)

        # ELF browser
        lf = ttk.LabelFrame(panes, text='ELF functions', padding=6)
        panes.add(lf, weight=3)
        lf.columnconfigure(1, weight=1)
        lf.rowconfigure(2, weight=1)
        self.elf_var = tk.StringVar(value=st.get('elf', ''))
        ttk.Label(lf, text='ELF').grid(row=0, column=0, sticky='w')
        ttk.Entry(lf, textvariable=self.elf_var).grid(row=0, column=1, sticky='ew', padx=4)
        ttk.Button(lf, text='Browse...', command=self._pick_elf).grid(row=0, column=2)
        self.find_var = tk.StringVar()
        self.find_var.trace_add('write', lambda *a: self._fill())
        ttk.Label(lf, text='Find').grid(row=1, column=0, sticky='w', pady=4)
        ttk.Entry(lf, textvariable=self.find_var).grid(row=1, column=1, columnspan=2,
                                                       sticky='ew', padx=4)
        from tkinter import font as tkfont
        line = tkfont.nametofont('TkDefaultFont').metrics('linespace')
        ttk.Style(self.root).configure('Treeview', rowheight=int(line * 1.35))   # descenders
        tv = ttk.Treeview(lf, columns=('addr', 'size'), selectmode='extended')
        tv.heading('#0', text='Function')
        tv.heading('addr', text='Address')
        tv.heading('size', text='Bytes')
        tv.column('#0', width=300, minwidth=150, stretch=True)
        tv.column('addr', width=110, minwidth=100, anchor='e', stretch=False)
        tv.column('size', width=60, minwidth=50, anchor='e', stretch=False)
        sb = ttk.Scrollbar(lf, orient='vertical', command=tv.yview)
        tv.configure(yscrollcommand=sb.set)
        tv.grid(row=2, column=0, columnspan=3, sticky='nsew')
        sb.grid(row=2, column=3, sticky='ns')
        self.tree = tv
        bf = ttk.Frame(lf)
        bf.grid(row=3, column=0, columnspan=3, sticky='ew', pady=(6, 0))
        ttk.Button(bf, text='Window = selection', command=lambda: self._use('window')).pack(
            side='left')
        ttk.Button(bf, text='Exclude = selection', command=lambda: self._use('exclude')).pack(
            side='left', padx=4)
        self.sel_var = tk.StringVar(value='')
        ttk.Label(bf, textvariable=self.sel_var).pack(side='left', padx=8)
        self.tree.bind('<<TreeviewSelect>>', lambda e: self._show_selection())

        # options
        of = ttk.LabelFrame(panes, text='Recording', padding=6)
        panes.add(of, weight=2)
        of.columnconfigure(1, weight=1)
        self.vars = {}

        def var(key, default):
            v = tk.StringVar(value=str(st.get(key, default)))
            self.vars[key] = v
            return v

        r = 0
        ttk.Label(of, text='CPU').grid(row=r, column=0, sticky='w')
        ttk.Combobox(of, textvariable=var('cpu', '2'), values=[str(i) for i in range(6)],
                     width=4, state='readonly').grid(row=r, column=1, sticky='w')
        r += 1
        ttk.Label(of, text='Mode').grid(row=r, column=0, sticky='w')
        mf = ttk.Frame(of)
        mf.grid(row=r, column=1, sticky='w')
        mode = var('mode', 'flow')
        ttk.Radiobutton(mf, text='flow (coverage, calls)', value='flow', variable=mode).pack(
            side='left')
        ttk.Radiobutton(mf, text='calls only', value='calls', variable=mode).pack(side='left',
                                                                                  padx=6)
        for key, label, hint in (
                ('window', 'Window', 'LO:HI: only inside'),
                ('exclude', 'Exclude', 'LO:HI: never inside'),
                ('bursts', 'Bursts', 'ON:PERIOD us (<= ~430)'),
                ('cft', 'CFT', 'VSHORT:SHORT (calls)')):
            r += 1
            ttk.Label(of, text=label).grid(row=r, column=0, sticky='w', pady=2)
            ttk.Entry(of, textvariable=var(key, '')).grid(row=r, column=1, sticky='ew')
            ttk.Label(of, text=hint, foreground='gray').grid(row=r, column=2, sticky='w', padx=4)
        r += 1
        self.span_var = tk.StringVar(value='')
        ttk.Label(of, textvariable=self.span_var, wraplength=520, justify='left').grid(
            row=r, column=0, columnspan=3, sticky='w', pady=4)
        r += 1
        ttk.Label(of, text='DAP clock').grid(row=r, column=0, sticky='w', pady=2)
        cf = ttk.Frame(of)
        cf.grid(row=r, column=1, sticky='w')
        ttk.Combobox(cf, textvariable=var('clock', '120'), values=['120', '100', '48', 'auto'],
                     width=6, state='readonly').pack(side='left')
        ttk.Label(cf, text=' MHz bin (120: the reliable ceiling)', foreground='gray').pack(
            side='left')
        r += 1
        ttk.Label(of, text='Seconds').grid(row=r, column=0, sticky='w', pady=2)
        ttk.Entry(of, textvariable=var('seconds', '5'), width=8).grid(row=r, column=1, sticky='w')
        ttk.Label(of, text='0: until Stop', foreground='gray').grid(row=r, column=2, sticky='w',
                                                                  padx=4)
        r += 1
        ttk.Label(of, text='File').grid(row=r, column=0, sticky='w', pady=2)
        ff = ttk.Frame(of)
        ff.grid(row=r, column=1, columnspan=2, sticky='ew')
        ff.columnconfigure(0, weight=1)
        ttk.Entry(ff, textvariable=var('out', os.path.join(os.path.expanduser('~'),
                                                           'ptrace.mcds'))).grid(
            row=0, column=0, sticky='ew')
        ttk.Button(ff, text='...', width=3, command=self._pick_out).grid(row=0, column=1)
        r += 1
        rf = ttk.Frame(of)
        rf.grid(row=r, column=0, columnspan=3, sticky='ew', pady=(8, 2))
        self.start_btn = ttk.Button(rf, text='Record', command=self._start)
        self.start_btn.pack(side='left')
        self.stop_btn = ttk.Button(rf, text='Stop', command=self._stop, state='disabled')
        self.stop_btn.pack(side='left', padx=4)
        ttk.Button(rf, text='Analyse file...', command=self._analyse_file).pack(side='left')
        r += 1
        self.prog_var = tk.StringVar(value='')
        ttk.Label(of, textvariable=self.prog_var).grid(row=r, column=0, columnspan=3, sticky='w')
        r += 1
        lp = ttk.LabelFrame(of, text='Hottest loops (last analysis)', padding=4)
        lp.grid(row=r, column=0, columnspan=3, sticky='nsew', pady=(6, 0))
        of.rowconfigure(r, weight=1, minsize=150)
        lp.columnconfigure(0, weight=1)
        lp.rowconfigure(0, weight=1)
        self.loop_list = tk.Listbox(lp, height=6)
        self.loop_list.grid(row=0, column=0, sticky='nsew')
        ttk.Button(lp, text='Exclude this loop', command=self._exclude_loop).grid(
            row=1, column=0, sticky='w', pady=(4, 0))
        r += 1
        of2 = ttk.Frame(of)
        of2.grid(row=r, column=0, columnspan=3, sticky='ew', pady=(6, 0))
        self.open_cov = ttk.Button(of2, text='Coverage', state='disabled',
                                   command=lambda: webbrowser.open(
                                       'file:///' + (self.last_base + '.html').replace('\\', '/')))
        self.open_cov.pack(side='left')
        self.open_dir = ttk.Button(of2, text='Folder', state='disabled',
                                   command=lambda: open_path(os.path.dirname(self.last_base)))
        self.open_dir.pack(side='left', padx=4)
        ttk.Button(of2, text='Perfetto UI',
                   command=lambda: webbrowser.open('https://ui.perfetto.dev')).pack(side='left')

        # log
        lg = ttk.Frame(root, padding=(6, 0, 6, 6))
        lg.grid(row=2, column=0, sticky='nsew')
        lg.columnconfigure(0, weight=1)
        self.log = tk.Text(lg, height=11, wrap='word')
        self.log.grid(row=0, column=0, sticky='nsew')
        ls = ttk.Scrollbar(lg, orient='vertical', command=self.log.yview)
        self.log.configure(yscrollcommand=ls.set)
        ls.grid(row=0, column=1, sticky='ns')
        for key in ('window', 'exclude'):
            self.vars[key].trace_add('write', lambda *a: self._show_spans())

    # helpers -----------------------------------------------------------------
    def say(self, text: str) -> None:
        self.log.insert('end', text.rstrip() + '\n')
        self.log.see('end')

    def _probe(self, what: str) -> None:
        if self.stop_ev is not None:
            return                           # the recording holds the probe
        self.status_var.set('probe: ...')
        threading.Thread(target=probe_call, args=(what, self.q), daemon=True).start()

    def _pick_elf(self) -> None:
        from tkinter import filedialog
        p = filedialog.askopenfilename(title='ELF', filetypes=[('ELF', '*.elf'), ('all', '*')])
        if p:
            self.elf_var.set(p)
            self._load_elf(p)

    def _pick_out(self) -> None:
        from tkinter import filedialog
        p = filedialog.asksaveasfilename(title='Recording', defaultextension='.mcds',
                                         filetypes=[('capture', '*.mcds'), ('all', '*')])
        if p:
            self.vars['out'].set(p)

    def _load_elf(self, path: str) -> None:
        self.say('reading %s (the first time takes a few seconds) ...' % path)
        threading.Thread(target=load_code, args=(path, self.q), daemon=True).start()

    def _fill(self) -> None:
        if self.code is None:
            return
        tv = self.tree
        tv.delete(*tv.get_children())
        self.rows = []
        f = self.find_var.get().strip().lower()
        code = self.code
        for k, name in enumerate(code.func_name):
            if f and f not in name.lower():
                continue
            a = int(code.func_start[k])
            tv.insert('', 'end', iid=str(k), text=name,
                      values=('0x%08x' % a, int(code.func_end[k]) - a))
            self.rows.append(k)

    def _picked(self) -> list[int]:
        return [int(i) for i in self.tree.selection()]

    def _show_selection(self) -> None:
        if self.code is None:
            return
        sp = span_of(self.code, self._picked())
        if sp is None:
            self.sel_var.set('')
            return
        self.sel_var.set('%d selected: %s%s' % (
            sp.selected, fmt_range(sp.lo, sp.hi),
            ', spans %d more' % len(sp.covered) if sp.covered else ''))

    def _use(self, which: str) -> None:
        if self.code is None:
            return
        sp = span_of(self.code, self._picked())
        if sp is None:
            return
        self.vars[which].set(fmt_range(sp.lo, sp.hi))
        if which == 'window' and self.vars['mode'].get() != 'flow':
            self.say('note: the window works in calls mode too (it gates the calls trace)')

    def _show_spans(self) -> None:
        parts = []
        for key in ('window', 'exclude'):
            try:
                r = parse_range(self.vars[key].get())
            except ValueError as e:
                parts.append('%s: %s' % (key, e))
                continue
            if r is None:
                continue
            what = ''
            if self.code is not None:
                lo, hi = r
                inside = [n for k, n in enumerate(self.code.func_name)
                          if int(self.code.func_start[k]) <= hi and int(self.code.func_end[k]) > lo]
                if inside:
                    what = ': %d function(s): %s%s' % (
                        len(inside), ', '.join(inside[:4]), ' ...' if len(inside) > 4 else '')
            parts.append('%s %s%s' % (key, fmt_range(*r), what))
        self.span_var.set('\n'.join(parts))

    def _gather(self):
        v = self.vars
        window = parse_range(v['window'].get())
        exclude = parse_range(v['exclude'].get())
        bursts = parse_range(v['bursts'].get())
        cft = parse_range(v['cft'].get())
        if bursts and (bursts[1] > 430 or bursts[0] >= bursts[1] or bursts[0] == 0):
            raise ValueError('bursts: ON below PERIOD, PERIOD up to ~430 us')
        cfg = build_config(int(v['cpu'].get()), v['mode'].get(), window, exclude, bursts, cft)
        return cfg, window, float(v['seconds'].get() or 0), v['out'].get()

    # actions -----------------------------------------------------------------
    def _start(self) -> None:
        try:
            cfg, _, seconds, out = self._gather()
        except ValueError as e:
            self.say('error: %s' % e)
            return
        if not out:
            self.say('error: no file')
            return
        self._save_settings()
        self.stop_ev = threading.Event()
        self.start_btn.configure(state='disabled')
        self.stop_btn.configure(state='normal')
        p = cfg['program']
        self.say('recording CPU%d %s%s%s%s -> %s' % (
            cfg['cpu'], p['mode'],
            ', window ' + fmt_range(*p['window']) if p['window'] else '',
            ', exclude ' + fmt_range(*p['exclude']) if p['exclude'] else '',
            ', bursts %d/%d us' % (p['duty_on_us'], p['duty_period_us'])
            if p['duty_period_us'] else '', out))
        threading.Thread(target=record, args=(cfg, out, seconds, self.vars['clock'].get(),
                                              self.stop_ev, self.q), daemon=True).start()

    def _stop(self) -> None:
        if self.stop_ev is not None:
            self.stop_ev.set()
            self.stop_btn.configure(state='disabled')

    def _analyse(self, path: str) -> None:
        elf = self.elf_var.get()
        if not elf:
            self.say('error: no ELF to analyse against')
            return
        try:
            window = parse_range(self.vars['window'].get())
        except ValueError:
            window = None
        code = self.code if self.code_elf == elf else None
        self.say('analysing %s ...' % os.path.basename(path))
        threading.Thread(target=analyse, args=(path, elf, code, window, self.q),
                         daemon=True).start()

    def _analyse_file(self) -> None:
        from tkinter import filedialog
        p = filedialog.askopenfilename(title='Recording',
                                       filetypes=[('recordings', '*.mcds *.dtrp'), ('all', '*')])
        if p:
            self._analyse(p)

    def _exclude_loop(self) -> None:
        sel = self.loop_list.curselection()
        if sel:
            lo, hi, _, _ = self.loops[sel[0]]
            self.vars['exclude'].set(fmt_range(lo, hi))

    def _pump(self) -> None:
        try:
            while True:
                m = self.q.get_nowait()
                kind = m[0]
                if kind == 'log':
                    self.say(m[1])
                elif kind == 'error':
                    self.say('error: ' + m[1])
                    self._recording_done()
                elif kind == 'status':
                    self.status_var.set('probe: ' + m[1])
                elif kind == 'code':
                    self.code, self.code_elf = m[2], m[1]
                    self._fill()
                    self._show_spans()
                    self.say('%s: %d functions, %d instructions' % (
                        os.path.basename(m[1]), len(m[2].func_name), len(m[2])))
                elif kind == 'progress':
                    n, dt = m[1], m[2]
                    self.prog_var.set('%.1f s, %.1f MB, %.2f MB/s' % (dt, n / 1e6, n / 1e6 / dt
                                                                    if dt else 0))
                elif kind == 'recorded':
                    _, path, n, dt, stats = m
                    self.say('recorded %.1f MB in %.1f s (%.2f MB/s); probe: %s' % (
                        n / 1e6, dt, n / 1e6 / dt, stats))
                    self._recording_done()
                    self._analyse(path)
                elif kind == 'analysed':
                    _, base, text, loops = m
                    self.last_base = base
                    self.say(text)
                    self.loops = loops
                    self.say('%d hot loops listed: pick one and Exclude this loop' % len(loops))
                    self.loop_list.delete(0, 'end')
                    for lo, hi, n, fn in loops:
                        self.loop_list.insert('end', '%9d  %s  %s' % (n, fmt_range(lo, hi), fn))
                    self.open_cov.configure(state='normal')
                    self.open_dir.configure(state='normal')
        except queue.Empty:
            pass
        self.root.after(100, self._pump)

    def _recording_done(self) -> None:
        self.stop_ev = None
        self.start_btn.configure(state='normal')
        self.stop_btn.configure(state='disabled')

    def _close(self) -> None:
        if self.stop_ev is not None:
            self.stop_ev.set()
        self._save_settings()
        self.root.after(300, self.root.destroy)

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    App().run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
