"""The frame renderer: tick labels, geometry and awkward data."""

import math

import numpy as np
import pytest

from mcds_trace.plotrender import (FrameSpec, SubplotRenderer, Trace, eng_label, minmax_decimate,
                                   scalar_labels)


def test_eng_labels_match_matplotlib():
    assert [eng_label(x, 's') for x in (0, 0.6, 1.2, 3e-6, 0.0006)] == \
        ['0s', '600ms', '1.2s', '3µs', '600µs']
    assert eng_label(1.2e6) == '1.2M' and eng_label(-2.5e6) == '−2.5M'
    assert eng_label(999999.9) == '1M'


def test_scalar_labels_share_decimals():
    assert scalar_labels([0.0, 0.25, 0.5, 0.75, 1.0]) == ['0.00', '0.25', '0.50', '0.75', '1.00']
    assert scalar_labels([-30000.0, -15000.0, 0.0]) == ['−30000', '−15000', '0']
    assert scalar_labels([0.3, 0.6, 0.9]) == ['0.3', '0.6', '0.9']


def _frame(**kw):
    spec = FrameSpec(**{'width': 400, 'height': 150, 'x0': 0.0, 'x1': 1.0, **kw})
    r = SubplotRenderer()
    buf, w, h = r.render(spec)
    assert len(buf) == w * h * 4 and (w, h) == (int(spec.width * spec.scale),
                                                int(spec.height * spec.scale))
    return r.info, np.frombuffer(buf, np.uint8).reshape(h, w, 4)


@pytest.mark.parametrize('style', ['step', 'line', 'points'])
def test_frames_for_awkward_data(style):
    t = np.linspace(0, 1, 5000)
    cases = [
        [],
        [Trace(t, np.full_like(t, np.nan), '#ff0000')],
        [Trace(t, np.where(t > 0.5, np.inf, 1.0), '#00ff00')],
        [Trace(t, np.sin(t * 40), '#0000ff'), Trace(t[:1], np.array([3.0]), '#123456')],
        [Trace(np.array([-5.0, 5.0]), np.array([1.0, 2.0]), '#aa00aa')],     # both outside
        [Trace(t, t, '#000000', 'hits')],
    ]
    for traces in cases:
        info, _ = _frame(traces=traces, style=style)
        assert math.isfinite(info.y0) and math.isfinite(info.y1) and info.y1 > info.y0
    for ylim in [(math.inf, 1.0), (math.nan, math.nan), (2.0, 2.0)]:
        info, _ = _frame(traces=cases[3], style=style, ylim=ylim)       # ignored, not fatal
        assert math.isfinite(info.y0) and info.y1 > info.y0
    info, _ = _frame(traces=cases[3], style=style, ylog=True, scale=2.0, theme='dark',
                     gaps=[0.5], cursor=0.3, marker=0.7, xlabel=False)
    assert info.bottom == 150 - 6


def test_step_autoscale_uses_the_held_value():
    # 0 from t=1, 1000 from t=2; a view inside [1, 2] shows the held 0.
    info, _ = _frame(traces=[Trace(np.array([1.0, 2.0]), np.array([0.0, 1000.0]), '#ff0000')],
                     x0=1.2, x1=1.8)
    assert info.y0 < 0 < info.y1 < 10


def test_line_is_drawn_where_the_data_is():
    t = np.linspace(0, 1, 1000)
    info, img = _frame(traces=[Trace(t, np.zeros_like(t), '#ff0000')], ylim=(-1.0, 1.0))
    y = int(round((info.top + info.bottom) / 2))
    row = img[y - 1:y + 2, int(info.left) + 5:int(info.right) - 5, :3]
    assert (row == [255, 0, 0]).all(axis=-1).any(axis=0).all()      # red across the whole width


def test_decimation_keeps_extremes_in_order():
    t = np.linspace(0, 1, 100000)
    v = np.zeros_like(t)
    v[1234], v[5678] = 5.0, -7.0
    dt, dv = minmax_decimate(t, v, 0, 1, 100)
    assert dv.max() == 5.0 and dv.min() == -7.0 and (np.diff(dt) >= 0).all()


def test_decimation_bins_the_view_not_far_neighbours():
    t = np.concatenate(([0.0], np.linspace(100, 101, 20000), [1000.0]))
    v = np.sin(t * 50)
    dt, dv = minmax_decimate(t, v, 100, 101, 800)
    inside = (dt >= 100) & (dt <= 101)
    assert inside.sum() > 800 and dt[0] == 0.0 and dt[-1] == 1000.0


def test_log_scale_and_held_step_value_autoscale():
    t = np.linspace(0, 1, 100)
    info, _ = _frame(traces=[Trace(t, 1 + 999 * t, '#ff0000')], ylog=True)
    assert 0 < info.y0 < 1 and info.y1 > 1000              # a real log range
    # 100 held from before the view, then 0..1 inside: 100 must be in range
    tt = np.array([-1.0, 0.5, 0.6])
    info, _ = _frame(traces=[Trace(tt, np.array([100.0, 0.0, 1.0]), '#00ff00')])
    assert info.y1 >= 100 and info.y0 <= 0
