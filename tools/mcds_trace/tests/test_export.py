"""Parquet export: time order, window, scaling, metadata - and fast at capture scale."""

import json
import time

import numpy as np
import pyarrow.parquet as pq

from mcds_trace.export import parquet_bytes, samples_table, write_parquet
from mcds_trace.signals import SignalStore, raw_signal


def store_with(series: dict) -> SignalStore:
    st = SignalStore()
    for sid, (t, v) in series.items():
        st.get(sid).extend(np.asarray(t, float), np.asarray(v, float))
    return st


def test_rows_in_time_order_inside_the_window(tmp_path):
    a, b = raw_signal(0x1000, 4, 'a'), raw_signal(0x1004, 2, 'b')
    b.gain, b.unit = 0.5, 'V'
    st = store_with({a.id: ([0.0, 1.0, 2.0, 3.0], [1, 2, 3, 4]),
                     b.id: ([0.5, 1.5, 2.5], [10, 20, 30])})
    table = samples_table(st, [a, b], lambda s: s.id, 1.0, 2.5, {'capture': 'x.mcds'})
    path = tmp_path / 's.parquet'
    write_parquet(table, str(path))
    back = pq.read_table(str(path))
    d = back.to_pydict()
    assert d['time_s'] == [1.0, 1.5, 2.0, 2.5]           # [x0, x1], neighbours dropped
    assert d['signal'] == ['a', 'b', 'a', 'b']
    assert d['value'] == [2.0, 10.0, 3.0, 15.0]           # b scaled as shown
    assert d['raw'] == [2.0, 20.0, 3.0, 30.0]
    assert d['unit'] == ['', 'V', '', 'V']
    info = json.loads(back.schema.metadata[b'mcds_trace'])
    assert info['capture'] == 'x.mcds' and info['signals'][1]['addr'] == '0x00001004'
    assert info['signals'][1]['gain'] == 0.5


def test_hits_are_not_scaled_and_empty_is_fine():
    a = raw_signal(0x1000, 4, 'a')
    a.gain = 3.0
    st = store_with({a.id: ([0.0, 1.0], [1, 1])})
    assert samples_table(st, [a], lambda s: s.id, 0, 1, hits=True).column('value').to_pylist() \
        == [1.0, 1.0]
    empty = samples_table(SignalStore(), [a], lambda s: s.id, 0, 1)
    assert empty.num_rows == 0 and len(parquet_bytes(empty)) > 0


def test_six_million_samples_in_seconds():
    # The case that froze the CSV export: 10 signals, 30 s at 20 kHz.
    n, sigs, series = 600_000, [], {}
    for k in range(10):
        s = raw_signal(0x1000 + 4 * k, 4, 's%d' % k)
        sigs.append(s)
        series[s.id] = (np.arange(n) / 20_000 + k * 1e-6, np.arange(n) % 1000)
    st = store_with(series)
    t0 = time.monotonic()
    data = parquet_bytes(samples_table(st, sigs, lambda s: s.id, 0, 30))
    dt = time.monotonic() - t0
    assert dt < 10, dt
    print('6M samples: %.2f s, %.1f MB' % (dt, len(data) / 1e6))
