"""Signal extraction: shadow memory, partial writes, snapshots, slot planning."""

import math
import struct

import numpy as np

from mcds_trace import elfsyms
from mcds_trace.decode import Event
from mcds_trace.plotrender import minmax_decimate, value_at
from mcds_trace.signals import (Extractor, Shadow, SignalStore, hits_signal, leaf_signal,
                                plan_slots, raw_signal)

HZ = 100_000_000


def ev(cyc, addr, value, size, kind='write'):
    return Event(cyc, kind, 'DTU0', addr=addr, value=value, size=size)


def test_shadow_merge_and_known_mask():
    sh = Shadow()
    sh.cover(0x100, 0x104)
    sh.cover(0x108, 0x10C)
    sh.write(0x100, b'\x01\x02\x03\x04')
    sh.cover(0x102, 0x10A)                      # bridges both, keeps the bytes
    assert len(sh.ranges) == 1
    assert sh.read(0x100, 4) == b'\x01\x02\x03\x04'
    assert sh.read(0x104, 4) is None             # never written
    sh.write(0x104, b'\xaa\xbb\xcc\xdd')
    assert sh.read(0x102, 4) == b'\x03\x04\xaa\xbb'


def test_partial_writes_compose_a_word():
    s = raw_signal(0x1000, 4)
    store = SignalStore()
    ex = Extractor([s], store, HZ)
    ex.feed([ev(1000, 0x1000, 0x3412, 2)])      # low half: value not complete yet
    assert store.series.get(s.id) is None or store.series[s.id].n == 0
    ex.feed([ev(1100, 0x1002, 0x7856, 2)])      # high half completes it
    t, v = store.snapshot(s.id)
    assert list(v) == [0x78563412]
    assert math.isclose(t[0], 100 / HZ)


def test_wide_store_updates_every_member():
    a = raw_signal(0x2000, 2, 'a')
    b = raw_signal(0x2002, 2, 'b', signed=True)
    store = SignalStore()
    ex = Extractor([a, b], store, HZ)
    ex.feed([ev(0, 0x2000, 0xFFFE0005, 4)])
    assert store.snapshot(a.id)[1].tolist() == [5]
    assert store.snapshot(b.id)[1].tolist() == [-2]
    assert ex.stats.samples == 2


def test_snapshot_seeds_initial_values_and_partials():
    s = raw_signal(0x3000, 4)
    const = raw_signal(0x3004, 1, 'const')
    cfg = {'snapshot': [{'addr': '0x3000', 'hex': '11223344' + '07000000'}]}
    store = SignalStore()
    ex = Extractor([s, const], store, HZ, cfg)
    ex.feed([ev(500, 0x3000, 0xAA, 1)])          # byte store: rest from the snapshot
    t, v = store.snapshot(s.id)
    assert v.tolist() == [0x44332211, 0x443322AA]  # start value at t=0, then the store
    assert t[0] == 0.0
    assert store.snapshot(const.id)[1].tolist() == [7]   # never written, still shown


def test_float_leaf_and_unmatched():
    f32 = elfsyms.CType('base', 'float', 4, 'float')
    node = elfsyms.Node('x.f', 'f', 0x4000, f32)
    s = leaf_signal(node)
    store = SignalStore()
    ex = Extractor([s], store, HZ)
    bits = struct.unpack('<I', struct.pack('<f', -2.25))[0]
    ex.feed([ev(0, 0x4000, bits, 4), ev(10, 0x5000, 1, 4)])
    assert store.snapshot(s.id)[1].tolist() == [-2.25]
    assert ex.stats.unmatched == 1


def test_address_only_and_data_only_payloads():
    s = raw_signal(0x6000, 2)
    store = SignalStore()
    ex = Extractor([s], store, HZ)
    ex.feed([Event(0, 'write', addr=0x6000, value=None, size=2),
             Event(5, 'write', addr=None, value=0x1234, size=2)])
    v = store.snapshot(s.id)[1]
    assert math.isnan(v[0]) and v[1] == 0x1234


def test_hits_and_gaps():
    h = hits_signal(1, 'slot1')
    store = SignalStore()
    ex = Extractor([h], store, HZ)
    ex.feed([Event(0, 'wps', wp=1), Event(10, 'wps', wp=0), Event(-1, 'gap'),
             Event(20, 'wpm', wp=0b10)])
    assert store.series[h.id].n == 2
    assert len(store.gaps) == 1


def test_plan_two_ranges_minimise_bytes():
    sigs = [raw_signal(0x100, 4), raw_signal(0x104, 4), raw_signal(0x2000, 2),
            raw_signal(0x2004, 2), raw_signal(0x9000, 1)]
    plan = plan_slots(sigs)
    assert len(plan.ranges) == 2
    # The largest hole (0x2006..0x9000) is where the ranges split.
    assert plan.ranges[0] == (0x100, 0x2006 - 0x100)
    assert plan.ranges[1] == (0x9000, 1)
    slots = plan.slots('w', ['x', 'y'])
    assert slots[0]['addr'] == '0x00000100' and slots[1]['size'] == 1


def test_plan_single_and_overlap():
    plan = plan_slots([raw_signal(0x10, 4), raw_signal(0x12, 2)])
    assert plan.ranges == [(0x10, 4)] and plan.extra_bytes == 0
    assert plan.slots()[1] == {'enabled': False}


def test_series_growth_and_trim():
    store = SignalStore()
    s = store.get('x')
    for i in range(10000):
        s.append(i * 0.001, float(i))
    store.t_end = 9.999
    store.trim(2.0)
    t, v = store.snapshot('x')
    assert t[0] <= 7.999 + 1e-9 and t[-1] == 9.999 and len(t) < 2100


def test_decimation_keeps_extremes():
    t = np.linspace(0, 1, 100_000)
    v = np.sin(t * 200)
    v[54321] = 50.0                              # a single-sample spike
    dt, dv = minmax_decimate(t, v, 0, 1, 500)
    assert len(dt) <= 1002 and 50.0 in dv.tolist()
    assert np.all(np.diff(dt) >= 0)
    assert value_at(t, v, t[54321]) == 50.0
    assert math.isnan(value_at(t, v, -1.0))


def test_labels_and_workspace_colors():
    from mcds_trace.signals import _short
    from mcds_trace.ui.model import Workspace
    assert _short('(anonymous)::getApp::app.mModulatorMeans._M_elems[3]') == \
        'app.mModulatorMeans[3]'
    d = {'signals': [{'id': 'raw:0x70000000:4', 'addr': '0x70000000', 'size': 4},
                     {'id': 'raw:0x70000004:4', 'addr': '0x70000004', 'size': 4},
                     {'id': 'raw:0x70000008:4', 'addr': '0x70000008', 'size': 4,
                      'color': '#123456'}]}
    ws, missing = Workspace.from_json(d, None)
    colors = [s.color for s in ws.signals.values()]
    assert not missing and len(set(colors)) == 3 and colors[2] == '#123456'


def test_exact_stores_then_a_partial_one():
    # Whole-signal stores skip the shadow until something needs it: a byte
    # store into the word must still see the latest whole value.
    a, b = raw_signal(0x100, 4), raw_signal(0x104, 2, signed=True)
    store = SignalStore()
    ex = Extractor([a, b], store, HZ)
    ex.feed([ev(0, 0x100, 0x11223344, 4), ev(10, 0x104, 0xFFFE, 2),
             ev(20, 0x100, 0x55667788, 4), ev(30, 0x101, 0xAA, 1),
             ev(40, 0x100, 0x01020304, 4)])
    ex.feed([ev(50, 0x103, 0xBB, 1)])
    assert list(store.series[a.id].view()[1]) == [0x11223344, 0x55667788, 0x5566AA88,
                                                  0x01020304, 0xBB020304]
    assert list(store.series[b.id].view()[1]) == [-2]
