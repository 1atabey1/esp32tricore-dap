"""mcds-trace: capture, decode and plot miniMCDS data / watch-point traces.

    mcds-trace capture HOST OUT.mcds --watch ADDR:SIZE[:ACCESS[:NAME]] [--cpu N] [--seconds N]
    mcds-trace capture HOST OUT.mcds --config CFG.json [--seconds N]
    mcds-trace capture HOST OUT.mcds --start          # config already on the probe
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
    for key in ('source', 'cpu', 'mode', 'payload', 'timestamps', 'dap_div'):
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


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog='mcds-trace', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='stream /ws/trace into a capture file')
    c.add_argument('host')
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

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
