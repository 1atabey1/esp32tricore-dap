"""Samples to Parquet: one row per sample, in time order.

Built column-wise with numpy (no per-row Python), so tens of millions of
samples take seconds, not minutes; call it off the UI thread anyway.
"""

from __future__ import annotations

import json
from typing import Callable

import numpy as np

from .signals import Signal, SignalStore


def samples_table(store: SignalStore, signals: list[Signal], trace_id: Callable[[Signal], str],
                  x0: float, x1: float, meta: dict | None = None, hits: bool = False):
    """A pyarrow Table of every sample of `signals` in [x0, x1]:
    time_s, signal (label), id, value (scaled, as shown), raw (decoded), unit.
    hits: the series are watch-point hits (compact mode), never scaled.
    The schema metadata `mcds_trace` holds the signals' addresses, types and
    scaling, the window and `meta`."""
    import pyarrow as pa

    ts, codes, values, raws = [], [], [], []
    for k, s in enumerate(signals):
        t, v = store.window(trace_id(s), x0, x1)
        m = (t >= x0) & (t <= x1)
        t, v = t[m], v[m]
        ts.append(t)
        raws.append(v)
        scaled = s.kind == 'value' and not hits
        values.append(np.asarray(s.apply(v) if scaled else v, dtype=np.float64))
        codes.append(np.full(len(t), k, dtype=np.int32))
    t = np.concatenate(ts) if ts else np.empty(0)
    order = np.argsort(t, kind='stable')          # per signal already sorted: a merge
    code = np.concatenate(codes)[order] if codes else np.empty(0, np.int32)

    def dictionary(names: list[str]):
        return pa.DictionaryArray.from_arrays(pa.array(code, pa.int32()),
                                              pa.array(names, pa.string()))

    table = pa.table({
        'time_s': pa.array(t[order], pa.float64()),
        'signal': dictionary([s.label for s in signals]),
        'id': dictionary([s.id for s in signals]),
        'value': pa.array(np.concatenate(values)[order] if values else np.empty(0), pa.float64()),
        'raw': pa.array(np.concatenate(raws)[order] if raws else np.empty(0), pa.float64()),
        'unit': dictionary([s.unit for s in signals]),
    })
    info = {
        'format': 'mcds-trace-samples', 'version': 1, 'window_s': [float(x0), float(x1)],
        'signals': [{'id': s.id, 'label': s.label, 'kind': s.kind,
                     'addr': '0x%08X' % s.addr, 'size': s.size,
                     'type': s.node.type_name() if s.node is not None else
                     ('signed' if s.signed else 'unsigned'),
                     'gain': s.gain, 'offset': s.offset, 'unit': s.unit} for s in signals],
    }
    info.update(meta or {})
    return table.replace_schema_metadata({'mcds_trace': json.dumps(info)})


def write_parquet(table, path_or_buffer) -> None:
    import pyarrow.parquet as pq

    pq.write_table(table, path_or_buffer, compression='zstd')


def parquet_bytes(table) -> bytes:
    import pyarrow as pa

    sink = pa.BufferOutputStream()
    write_parquet(table, sink)
    return sink.getvalue().to_pybytes()
