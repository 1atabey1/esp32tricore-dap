"""UI helpers that feed control properties: they must hand Flet plain Python
values (its msgpack serializer refuses numpy scalars)."""

import msgpack
import numpy as np

from mcds_trace.plotrender import FrameInfo
from mcds_trace.ui.app import View, overlay_box


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
