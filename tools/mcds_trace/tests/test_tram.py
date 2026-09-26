"""Generic trace memory parsing, checked against the TC3xx TS worked example."""

from mcds_trace.tram import CORE_PTU, parse_paragraph, split_tt


def test_table_342_program_trace_example():
    # Table 342: "C7 00 20 04 00 -8 | 2- 2C -2 | 2- 2C -6" (nibble-packed).
    # Nibbles in stream order: 7 C 0 0 0 2 4 0 0 0 | 8 2 | C 2 2 | 2 C 2 6 ...
    raw = bytes([0xC7, 0x00, 0x20, 0x04, 0x00, 0x28, 0x2C, 0x22, 0x2C, 0x06])
    msgs = [m for m in parse_paragraph(raw) if m.kind == 'msg']
    assert len(msgs) == 3
    first, second, third = msgs
    assert (first.core, first.tt, first.dbits) == (CORE_PTU, 0, 32)
    assert first.data == 0x80004200
    assert (second.core, second.tt) == (CORE_PTU, 1)
    assert split_tt(second)['data1'] == 0x2
    # Nexus compression: XOR with the previous reconstructed value.
    assert first.data ^ split_tt(second)['data1'] == 0x80004202
    assert (0x80004202 ^ split_tt(third)['data1']) == 0x80004204


def test_tick_multick_and_end():
    # <tick>, <multick 5>, <endoftrace>
    stream = [0x0, 0xE, 0x5, 0x0, 0xF, 0xF, 0xF]
    nib = stream + [0]
    raw = bytes(nib[i] | (nib[i + 1] << 4) for i in range(0, len(nib), 2))
    kinds = [(m.kind, m.ticks) for m in parse_paragraph(raw)]
    assert kinds == [('tick', 1), ('multick', 5), ('end', 0)]
