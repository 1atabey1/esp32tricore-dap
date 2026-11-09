"""mcds-trace: capture, decode and plot miniMCDS data / watch-point traces.

    mcds-trace capture HOST OUT.mcds --watch ADDR:SIZE[:ACCESS[:NAME]] [--cpu N] [--seconds N]
    mcds-trace capture HOST OUT.mcds --config CFG.json [--seconds N]
    mcds-trace capture HOST OUT.mcds --start          # config already on the probe
    mcds-trace capture tdm OUT.mcds --watch ...       # tricore-debug-machine on USB
    mcds-trace decode FILE.mcds [--csv OUT] [--plot [PNG]] [--elf APP.elf] [--signed]
"""

from __future__ import annotations

import argparse
import json
import sys

from . import analyse, capfile, capture, decode


def _symbols(elf: str) -> dict:
    """addr -> name for data objects, via tricore-elf-nm (or nm)."""
    import shutil
    import subprocess
    nm = shutil.which('tricore-elf-nm') or shutil.which('nm')
    if not nm:
        return {}
    out = subprocess.run([nm, '-S', elf], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2].lower() in 'bdgrs':
            addr = int(parts[0], 16)
            syms.setdefault(addr, parts[3])
    return syms


def _watch(spec: str) -> dict:
    """ADDR:SIZE[:ACCESS[:NAME]] -> one slot, e.g. 0x5000220C:2:w:mCyclesUntilSecond."""
    parts = spec.split(':')
    if len(parts) < 2:
        raise SystemExit('--watch needs ADDR:SIZE[:ACCESS[:NAME]], got %r' % spec)
    access = parts[2] if len(parts) > 2 and parts[2] else 'w'
    if access not in ('r', 'w', 'rw'):
        raise SystemExit('--watch access must be r, w or rw, got %r' % access)
    return {'enabled': True, 'addr': parts[0], 'size': int(parts[1], 0), 'access': access,
            'name': parts[3] if len(parts) > 3 else parts[0]}


def _build_config(a) -> dict | None:
    """The configuration to POST, from --config and/or --watch; None if neither."""
    if not a.config and not a.watch:
        return None
    cfg = {}
    if a.config:
        with open(a.config) as f:
            cfg = json.load(f)
    for key in ('source', 'cpu', 'mode', 'payload', 'timestamps', 'dap_div', 'wide', 'fast'):
        val = getattr(a, key)
        if val is not None:
            cfg[key] = val
    if a.watch:
        if len(a.watch) > 2:
            raise SystemExit('at most two --watch slots (one DTU comparator set each)')
        slots = [_watch(w) for w in a.watch]
        cfg['slots'] = slots + [{'enabled': False}] * (2 - len(slots))
    return cfg


def cmd_capture(a) -> int:
    cfg = _build_config(a)
    from .tdm import TdmError, is_tdm
    if is_tdm(a.host):
        try:
            res = capture.capture_tdm(a.host, a.out, cfg, a.seconds)
        except (RuntimeError, TdmError, OSError) as e:
            print('error:', e, file=sys.stderr)
            return 1
        print('%(bytes)d bytes in %(frames)d frames, %(seconds).1f s; probe: %(probe)s' % res)
        return 0
    if cfg is not None:
        try:
            applied = capture.post_config(a.host, a.auth, cfg)
        except (RuntimeError, OSError) as e:
            print('error:', e, file=sys.stderr)
            return 1
        for s in applied.get('slots', []):
            if s.get('enabled'):
                print('watch %-24s %s size %s %s' % (s['name'], s['addr'], s['size'], s['access']))
        print('source %s cpu %s, %s, timestamps %s' % (applied.get('source'), applied.get('cpu'),
                                                        applied.get('mode'), applied.get('timestamps')))
    res = capture.capture(a.host, a.out, a.auth, a.seconds, a.start or cfg is not None)
    print('%(bytes)d bytes in %(frames)d frames, %(seconds).1f s' % res)
    return 0


def cmd_decode(a) -> int:
    config, stream = capfile.read_file(a.file)
    recs = list(capfile.records(stream))
    events, st = decode.decode_records(recs, tick_mode=config.get('timestamps') == 'ticks')
    print('%d records, %d messages, %d events' % (len(recs), st.messages, len(events)))
    print('by type:', ', '.join('%s %d' % kv for kv in sorted(st.by_type.items())) or '-')
    lost = sum(r.lost for r in recs)
    print('gaps %d (%d paragraphs lost in the probe), ERR %d (%d messages lost on target), '
          'unknown %d, time backsteps %d' % (st.gaps, lost, st.errors, st.lost_messages,
                                             st.unknown, st.backsteps))
    if st.torn or st.laps:
        print('reader fell a lap behind: %d torn paragraphs dropped, %d lapped'
              % (st.torn, st.laps))
    if st.malformed:
        print('malformed:', ', '.join('%s %d' % kv for kv in sorted(st.malformed.items())))
    hz = config.get('emu_hz') or 0
    timed = [e for e in events if e.cycles >= 0]
    if hz and timed:
        span = (timed[-1].cycles - timed[0].cycles) / hz
        print('emulation clock %.2f MHz, %.6f s covered' % (hz / 1e6, span))
    if a.dump:
        for e in events[:a.dump]:
            print(e)
    if a.csv:
        analyse.write_csv(a.csv, events, config)
        print('csv written to', a.csv)
    if a.plot is not None:
        syms = _symbols(a.elf) if a.elf else None
        series = analyse.build_series(events, config, syms, a.signed)
        if a.window:
            lo, hi = (float(x) for x in a.window.split(':'))
            for s in series.values():
                keep = [i for i, t in enumerate(s.t) if lo <= t <= hi]
                s.t, s.value, s.kind = ([s.t[i] for i in keep], [s.value[i] for i in keep],
                                        [s.kind[i] for i in keep])
        for name, s in sorted(series.items()):
            print('  %-28s %8d samples' % (name, len(s.t)))
        analyse.plot(series, a.file, a.plot or None, timed=bool(hz))
    return 0


def _emu_hz(a, config: dict) -> float:
    if a.emu_hz:
        return float(a.emu_hz)
    if config.get('emu_hz'):
        return float(config['emu_hz'])
    import os
    import struct
    info = a.file + '.info'                   # tdm_dap.py trace: dap_mcds_info_t next to it
    if os.path.exists(info):
        with open(info, 'rb') as f:
            raw = f.read(4)
        if len(raw) == 4:
            return float(struct.unpack('<I', raw)[0])
    return 0.0


def cmd_program(a) -> int:
    """Program trace: flow reconstruction against the ELF, coverage, Perfetto."""
    import os
    import time
    from . import codemodel, ptrace, ptrace_export as px
    t0 = time.time()
    code = codemodel.load(a.elf)
    print('%s: %d instructions, %d functions (%.1f s)' % (os.path.basename(a.elf), len(code),
                                                          len(code.func_name), time.time() - t0))
    config, stream = capfile.read_file(a.file)
    recs = list(capfile.records(stream))
    hz = _emu_hz(a, config)
    t0 = time.time()
    dec = ptrace.decode_records(recs, code, hz, a.mode)
    print('mode: %s' % ('calls (subroutine trace)' if dec.calls_mode else 'flow'))
    st = dec.stats
    lost = sum(r.lost for r in recs)
    print('%d records (%d paragraphs lost in the probe), decoded in %.1f s'
          % (len(recs), lost, time.time() - t0))
    print(st.summary())
    if len(dec.anchors_cyc) >= 2 and hz:
        span = (dec.anchors_cyc[-1] - dec.anchors_cyc[0]) / hz
        print('time: %.6f s between the first and last time stamp (emulation clock %.2f MHz)'
              % (span, hz / 1e6))
    window = None
    if a.window:
        lo, _, hi = a.window.partition(':')
        window = (int(lo, 0), int(hi, 0))
    cov = px.coverage(dec, window)
    print(px.summary_text(cov, a.top))
    loops = px.hot_loops(dec)
    if loops:
        print('hottest loops (trace them away with tdm_dap.py trace --exclude LO:HI):')
        for lo, hi, n, fn in loops:
            print('  %10d  --exclude 0x%08x:0x%08x  %s' % (n, lo, hi, fn))
    base = a.out or os.path.splitext(a.file)[0]
    title = '%s - %s' % (os.path.basename(a.file), os.path.basename(a.elf))
    if not a.no_coverage:
        px.write_lcov(cov, base + '.info')
        px.write_html(cov, base + '.html', title, st.summary())
        print('coverage: %s.html, %s.info (lcov)' % (base, base))
    cpu = a.cpu if a.cpu is not None else int(config.get('cpu', 0) or 0)
    if not a.no_perfetto:
        n = px.write_perfetto(dec, base + '.pftrace', cpu)
        print('perfetto: %s.pftrace (%d slices; open in ui.perfetto.dev)' % (base, n))
        if a.json:
            px.write_chrome_json(dec, base + '.json', cpu)
            print('chrome json: %s.json' % base)
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog='mcds-trace', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='stream /ws/trace into a capture file')
    c.add_argument('host', help='esp32jtag address, or tdm (tdm:SERIAL) for the '
                   'tricore-debug-machine on USB')
    c.add_argument('out')
    c.add_argument('--auth', default='admin:admin')
    c.add_argument('--seconds', type=float)
    c.add_argument('--start', action='store_true',
                   help='start with the configuration stored on the probe, stop at the end')
    g = c.add_argument_group('configuration (posted to the probe; implies --start)')
    g.add_argument('--config', help='JSON file, as the web page sends it')
    g.add_argument('--watch', action='append', metavar='ADDR:SIZE[:ACCESS[:NAME]]',
                   help='a watch slot, e.g. 0x5000220C:2:w:mCyclesUntilSecond (repeat for slot 1)')
    g.add_argument('--source', choices=['cpu', 'memslave', 'lmu0'])
    g.add_argument('--cpu', type=int, help='CPU for the cpu / memslave sources')
    g.add_argument('--mode', choices=['full', 'compact'])
    g.add_argument('--payload', choices=['addr_data', 'data', 'addr'])
    g.add_argument('--timestamps', choices=['hit', 'ticks', 'none'])
    g.add_argument('--dap-div', dest='dap_div', type=int, help='DAP clock divider (0 = 24 MHz)')
    g.add_argument('--wide', action=argparse.BooleanOptionalAction, help='DAP wide mode (default on)')
    g.add_argument('--fast', action=argparse.BooleanOptionalAction,
                   help='fast mode: a DAP bit every 48 MHz fabric clock (default on)')
    c.set_defaults(fn=cmd_capture)

    d = sub.add_parser('decode', help='decode (and plot) a capture file')
    d.add_argument('file')
    d.add_argument('--csv')
    d.add_argument('--plot', nargs='?', const='', help='show a plot, or write it to PNG')
    d.add_argument('--window', help='plot only START:END seconds, e.g. 0.1:0.105')
    d.add_argument('--elf', help='name addresses from this ELF')
    d.add_argument('--signed', action='store_true', help='show values as signed')
    d.add_argument('--dump', type=int, default=0, help='print the first N events')
    d.set_defaults(fn=cmd_decode)

    g = sub.add_parser('program', help='analyse a program trace: coverage and a Perfetto trace',
                       description='Reconstruct the program flow of a PTU_TC program trace '
                       '(tdm_dap.py trace --program) against the ELF that ran: code coverage '
                       '(HTML overview, lcov) and the call view as a Perfetto trace.')
    g.add_argument('file', help='the recording (.dtrp from tdm_dap.py, or a capture file)')
    g.add_argument('--elf', required=True, help='the ELF the traced core ran')
    g.add_argument('--out', help='output base name (default: the recording without extension)')
    g.add_argument('--cpu', type=int, help='the traced core, for the track name')
    g.add_argument('--emu-hz', dest='emu_hz', type=float,
                   help='emulation clock (default: from FILE.info or the capture header)')
    g.add_argument('--mode', choices=['auto', 'flow', 'calls'], default='auto',
                   help='what the recording is (default: told by its messages)')
    g.add_argument('--window', metavar='LO:HI', help='report coverage only for this address range')
    g.add_argument('--top', type=int, default=20, help='functions in the summary')
    g.add_argument('--json', action='store_true', help='also write Chrome JSON')
    g.add_argument('--no-coverage', action='store_true')
    g.add_argument('--no-perfetto', action='store_true')
    g.set_defaults(fn=cmd_program)

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
