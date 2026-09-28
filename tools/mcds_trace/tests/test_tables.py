"""Value tables: the dec / hex / ascii formats, and how tables live in the workspace."""

import json

from mcds_trace import elfsyms
from mcds_trace.signals import leaf_signal, raw_signal
from mcds_trace.ui.model import Workspace


def test_hex_and_ascii_of_integers():
    u32 = raw_signal(0x1000, 4)
    assert u32.format_as(0x41424344, 'hex') == '0x41424344'
    assert u32.format_as(0x41424344, 'ascii') == "'DCBA'"          # little-endian bytes
    assert u32.format_as(0x41424344, 'dec') == str(0x41424344)
    s16 = raw_signal(0x1000, 2, signed=True)
    assert s16.format_as(-2, 'hex') == '0xFFFE'                      # two's complement, own width
    assert s16.format_as(-2, 'dec') == '-2'
    u8 = raw_signal(0x1000, 1)
    assert u8.format_as(0x0A, 'ascii') == "'.'"                     # not printable
    assert u8.format_as(None, 'hex') == '-'


def test_float_hex_is_its_bit_pattern():
    f32 = elfsyms.CType('base', 'float', 4, 'float')
    s = leaf_signal(elfsyms.Node('x.f', 'f', 0x4000, f32))
    assert s.format_as(1.0, 'hex') == '0x3F800000'
    assert s.format_as(float('nan'), 'hex') == '-'


def test_bitfield_hex_uses_the_field_width():
    u8 = elfsyms.CType('base', 'unsigned char', 1, 'unsigned')
    s = leaf_signal(elfsyms.Node('x.b', 'b', 0x4000, u8, bit_size=3, bit_shift=2))
    assert s.format_as(5, 'hex') == '0x5'


def test_dec_is_scaled_and_hex_is_raw():
    s = raw_signal(0x1000, 2)
    s.gain, s.unit = 0.5, 'V'
    assert s.format_as(10, 'dec') == '5 V'
    assert s.format_as(10, 'hex') == '0x000A'


def test_tables_persist_and_do_not_merge_into_plots():
    ws = Workspace()
    a, b = raw_signal(0x1000, 4, 'a'), raw_signal(0x1004, 4, 'b')
    ws.add_signal(a, -1)                                  # plot 1
    table = ws.new_plot('table')
    ws.add_signal(b, table.id)
    table.fmt = 'hex'
    table.row_fmt[b.id] = 'ascii'
    ws.merge_up(table.id)                                 # a table does not merge into a plot
    assert [p.kind for p in ws.subplots] == ['plot', 'table']

    back, missing = Workspace.from_json(json.loads(json.dumps(ws.to_json())), None)
    assert not missing
    t = back.subplots[1]
    assert (t.kind, t.fmt, t.signals, t.row_fmt) == ('table', 'hex', [b.id], {b.id: 'ascii'})

    back.remove_signal(b.id)
    assert back.subplots[1].row_fmt == {}
    # A workspace from before tables loads as plots.
    old = {'signals': [a.to_json()], 'subplots': [{'id': 1, 'signals': [a.id]}]}
    ws2, _ = Workspace.from_json(old, None)
    assert ws2.subplots[0].kind == 'plot' and ws2.subplots[0].fmt == 'dec'
