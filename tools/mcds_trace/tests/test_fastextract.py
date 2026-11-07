"""Extractor.feed_columns (compiled) against feed_rows (the reference)."""

import math
import os
import random

import numpy as np
import pytest

from mcds_trace import capfile
from mcds_trace.decode import K_EVC, K_GAP, K_LOST, K_READ, K_WPM, K_WPS, K_WRITE
from mcds_trace.elfsyms import CType, Node
from mcds_trace.fastdecode import HAVE_NUMBA, Columns, FastDecoder
from mcds_trace.signals import Extractor, Signal, SignalStore, hits_signal

pytestmark = pytest.mark.skipif(not HAVE_NUMBA, reason='numba not installed')

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = 0x70001000


def _node(addr, kind, size, enc, bits=0, shift=0):
    return Node('n%x' % addr, 'n', addr, CType(kind, 't', size, enc), bits, shift)


def _signals():
    s = [Signal('raw:%x:%d:%d' % (BASE + o, n, sg), 'r', BASE + o, n, signed=bool(sg))
         for o, n, sg in [(0, 4, 0), (4, 2, 1), (6, 1, 0), (7, 1, 1), (8, 8, 0), (16, 4, 1),
                          (17, 2, 0), (24, 3, 0), (40, 8, 1)]]
    nodes = [(48, _node(BASE + 48, 'base', 4, 'float')), (52, _node(BASE + 52, 'base', 8, 'float')),
             (60, _node(BASE + 60, 'base', 4, 'signed', 5, 3)),
             (60, _node(BASE + 60, 'base', 4, 'unsigned', 7, 9)),
             (64, _node(BASE + 64, 'enum', 2, 'signed')),
             (66, _node(BASE + 66, 'base', 2, 'float')),              # no decoding: None
             (68, _node(BASE + 68, 'base', 4, 'bool'))]
    for o, nd in nodes:
        s.append(Signal('node:%d:%d' % (o, nd.bit_shift), 'n', BASE + o, max(nd.size, 1), node=nd))
    return s


def _rows(rng, n, addrs_only_signals=False):
    rows, cyc = [], 1000
    for _ in range(n):
        cyc += rng.randrange(0, 50)
        r = rng.random()
        c = cyc if rng.random() > 0.02 else -1
        if r < 0.70:
            size = rng.choice((1, 2, 4, 4, 0))
            off = rng.randrange(-4, 80)
            val = rng.getrandbits(8 * (size or 4)) if rng.random() > 0.05 else -1
            addr = BASE + off if rng.random() > 0.05 else -1
            rows.append((c, rng.choice((K_WRITE, K_READ)), addr, val, size, -1, -1, 'DTU0', -1))
        elif r < 0.78:
            rows.append((c, K_WPS, -1, -1, 0, rng.randrange(-1, 9), -1, 'WTU', -1))
        elif r < 0.84:
            rows.append((c, K_WPM, -1, -1, 0, rng.randrange(-1, 256), -1, 'WTU', -1))
        elif r < 0.87:
            rows.append((c, K_EVC, -1, -1, 0, 3, rng.randrange(100), 'WTU', -1))
        elif r < 0.90:
            rows.append((c, K_LOST, -1, -1, 0, -1, rng.randrange(-1, 100), 'DTU0', -1))
        elif r < 0.92:
            rows.append((-1, K_GAP, -1, -1, 0, -1, -1, '', -1))
        else:
            rows.append((c, K_WRITE, BASE + 200, rng.getrandbits(32), 4, -1, -1, 'DTU0', -1))
    return rows


def _snapshot():
    return {'snapshot': [{'addr': '0x%08X' % (BASE + 8), 'hex': bytes(range(1, 21)).hex()}]}


def _compare(signals, rows, config, hz=100e6, batch=97):
    a = Extractor(signals, SignalStore(), hz, dict(config))
    b = Extractor(signals, SignalStore(), hz, dict(config))
    for i in range(0, len(rows), batch):
        a.feed_rows(rows[i:i + batch])
        b.feed_columns(Columns.from_rows(rows[i:i + batch]))
    sa, sb = a.store, b.store
    assert sorted(k for k, s in sa.series.items() if s.n) == \
        sorted(k for k, s in sb.series.items() if s.n)
    for sid, s in sa.series.items():
        ta, va = s.view()
        tb, vb = sb.get(sid).view()
        assert len(ta) == len(tb), sid
        assert np.array_equal(ta, tb), sid
        assert np.array_equal(va, vb, equal_nan=True), sid
    assert sa.gaps == sb.gaps
    assert sa.t_end == sb.t_end and sa.samples == sb.samples
    assert vars(a.stats) == vars(b.stats)
    assert a.t0 == b.t0
    return a


@pytest.mark.parametrize('seed', range(8))
def test_random_rows_match(seed):
    rng = random.Random(seed)
    sigs = _signals() + [hits_signal(0, 'h0'), hits_signal(1, 'h1')]
    rows = _rows(rng, 3000)
    ex = _compare(sigs, rows, _snapshot() if seed % 2 else {}, hz=0 if seed == 3 else 100e6,
                  batch=rng.randrange(1, 400))
    assert ex.stats.samples > 100


def test_data_only_single_signal_matches():
    rng = random.Random(9)
    rows = []
    for i in range(500):
        size = rng.choice((0, 1, 2))
        rows.append((1000 + i, K_WRITE, -1, rng.getrandbits(8 * max(size, 1)), size, -1, -1,
                     'DTU0', -1))
    for sig in (Signal('raw', 'r', BASE, 2, signed=True),
                Signal('f', 'f', BASE, 2, node=_node(BASE, 'base', 2, 'float'))):
        _compare([sig], rows, {})


def test_real_capture_matches():
    path = os.path.join(HERE, '..', 'cycles.mcds')
    if not os.path.exists(path):
        pytest.skip('no cycles.mcds')
    config, stream = capfile.read_file(path)
    recs = list(capfile.records(stream))
    dec = FastDecoder(tick_mode=config.get('timestamps') == 'ticks')
    cols = dec.decode([r.payload for r in recs], [r.gap for r in recs], [r.final for r in recs])
    rows = cols.rows()
    addrs = sorted({r[2] for r in rows if r[1] <= K_READ and r[2] >= 0})[:40]
    sigs = [Signal('raw:%x' % a, 'r', a, 4) for a in addrs]
    sigs += [Signal('b:%x' % a, 'b', a + 1, 1) for a in addrs[:5]]
    _compare(sigs, rows, config, hz=config.get('emu_hz') or 0, batch=5000)
