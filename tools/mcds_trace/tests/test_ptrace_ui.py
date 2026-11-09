"""Program trace window: the parts without Tk."""

import pytest

from mcds_trace import tdm
from mcds_trace.ptrace_ui import build_config, fmt_range, parse_range, span_of

from test_ptrace import _model


def test_span_of_functions():
    code = _model()                      # main 1000..100a, f 2000..2008, irq 3000..3004
    sp = span_of(code, [1])
    assert (sp.lo, sp.hi, sp.selected, sp.covered) == (0x2000, 0x2007, 1, [])
    sp = span_of(code, [0, 2])           # main and irq: f lies between
    assert (sp.lo, sp.hi) == (0x1000, 0x3003)
    assert sp.covered == [('f', 0x2000)]
    assert span_of(code, []) is None


def test_ranges():
    assert parse_range(' 0x80051d06:0x80051d07 ') == (0x80051d06, 0x80051d07)
    assert parse_range('100-397') == (100, 397)
    assert parse_range('') is None
    with pytest.raises(ValueError):
        parse_range('0x20:0x10')
    assert fmt_range(0x10, 0x20) == '0x00000010:0x00000020'


def test_config_packs_for_the_probe():
    cfg = build_config(2, 'flow', None, (0x80051d06, 0x80051d07), (100, 397), None)
    raw = tdm.pack_config(cfg)
    assert len(raw) == tdm.CFG_SIZE
    back = tdm.unpack_config(raw)
    assert back['cpu'] == 2 and back['fast'] is True
    assert back['program'] == {'mode': 'flow', 'cft_vshort': 0, 'cft_short': 0, 'window': None,
                               'exclude': ['0x80051D06', '0x80051D07'], 'duty_on_us': 100,
                               'duty_period_us': 397}
    assert not any(s['enabled'] for s in back['slots'])
