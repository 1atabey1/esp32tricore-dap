"""UI helpers that feed control properties: they must hand Flet plain Python
values (its msgpack serializer refuses numpy scalars)."""

import msgpack
import numpy as np

from mcds_trace.plotrender import FrameInfo
from mcds_trace.ui.app import View, link_text, overlay_box, probe_rates

# GET /api/mcds/config mid-trace on the board (2026-10-03), the fields the
# status panel reads: wide and fast came up, the drain runs on two lines and
# outruns the WiFi, so the probe's buffer is full and dropping.
LIVE = {
    'dap_div': 0, 'wide_active': True, 'fast_active': True,
    'stats': {'read_us': 4914991, 'read_paragraphs': 26478, 'queue_size': 4194304,
              'queue_free': 0, 'queue_dropped': 5138022, 'dual': True,
              'ws': {'framed': True, 'frames': 528, 'packed': 33, 'comp_pct': 54,
                     'comp_kbps': 4800, 'wire_kbps': 2900}},
}


def test_link_text_says_what_the_session_got():
    assert link_text(LIVE) == 'wide, fast 48 MHz, two-line drain'
    # An older probe: no fast_active, no dual flag.
    assert link_text({'dap_div': 0, 'wide_active': True}) == 'wide, 24 MHz, one-line drain'
    assert link_text({'dap_div': 5}) == 'narrow, 4 MHz, one-line drain'


def test_probe_rates_from_the_drain_counters():
    drain, fill, dropped = probe_rates(LIVE['stats'])
    assert 5.2e6 < drain < 5.7e6          # 26478 kB in 4.91 s
    assert fill == 1.0 and dropped == 5138022
    assert probe_rates({}) == (0.0, 0.0, 0)


def test_overlay_from_numpy_times_serializes():
    # A view fitted to sample times holds numpy floats; comparisons of those
    # give numpy.bool, which Flet cannot send ("can not serialize 'numpy.bool'").
    info = FrameInfo(64.0, 988.0, 6.0, 198.0, np.float64(-0.4), np.float64(20.2), 0.0, 1.0)
    box = overlay_box(info, np.float64(9.5))
    assert box is not None and all(type(v) is float for v in box)
    msgpack.packb({'visible': box is not None, 'left': box[0], 'top': box[1], 'height': box[2]})
    assert overlay_box(info, np.float64(30.0)) is None and overlay_box(info, None) is None
    assert overlay_box(None, 1.0) is None


def test_view_keeps_plain_floats():
    v = View()
    v.set(np.float64(1.0), np.float64(2.0))
    assert type(v.x0) is float and type(v.x1) is float
