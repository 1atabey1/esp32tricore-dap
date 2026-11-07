"""The compiled decoder (fastdecode) against the reference (decode.Decoder):
identical rows and statistics, paragraph by paragraph and in batches."""

import os
import random

import pytest

from mcds_trace import capfile
from mcds_trace.decode import Decoder
from mcds_trace.fastdecode import HAVE_NUMBA, FastDecoder
from mcds_trace.tram import (CODE_ABSIZE, CODE_ERROR, CODE_MULTICK, CORE_DTU0, CORE_DTU1,
                             CORE_TSU, CORE_WTU, SYMLEN_BITS)

from test_decode import Packer, _tsr_par

pytestmark = pytest.mark.skipif(not HAVE_NUMBA, reason='numba not installed')

HERE = os.path.dirname(os.path.abspath(__file__))


def _same(paragraphs, tick_mode=False, batch=7):
    """paragraphs: [(bytes, gap, final)]; both decoders, the fast one in batches."""
    ref = Decoder(tick_mode=tick_mode)
    want = []
    for par, gap, final in paragraphs:
        want += ref.rows(par, 0, gap, final)
    fast = FastDecoder(tick_mode=tick_mode)
    got = []
    for i in range(0, len(paragraphs), batch):
        chunk = paragraphs[i:i + batch]
        got += fast.decode([p for p, _, _ in chunk], [g for _, g, _ in chunk],
                           [f for _, _, f in chunk]).rows()
    assert len(got) == len(want)
    for k, (a, b) in enumerate(zip(got, want)):
        assert a == b, 'row %d: fast %r, reference %r' % (k, a, b)
    sr, sf = ref.stats, fast.stats
    for field in ('paragraphs', 'messages', 'unknown', 'errors', 'lost_messages', 'gaps',
                  'backsteps', 'unsynced', 'torn', 'laps', 'by_type', 'malformed'):
        assert getattr(sf, field) == getattr(sr, field), field
    return got


def test_spec_examples_match():
    p = Packer()
    p.msg(CORE_TSU, 0, [(1000, 32)])
    p.msg(CORE_DTU0, 4, [(0x2, 4), (0, 2), (0x5A, 8), (0x70001234, 32)])
    p.msg(CORE_TSU, 1, [(1000 ^ 1010, 12)])
    p.msg(CORE_DTU0, 5, [(0x1, 4), (0, 2), (0x1, 4)])
    p.msg(CORE_DTU0, 0, [(2, 2), (0xDEADBEEF, 32)])
    p.msg(CORE_DTU0, 2, [(0, 2), (0x7000A000, 32)])
    p.msg(CORE_DTU0, 1, [(2, 2), (0xDEADBEEF ^ 0x12345678, 32)])
    p.msg(CORE_DTU1, 6, [(0x4, 4), (1, 2), (0xBEEF, 16), (0x1_7000_1000, 37)])   # master bits
    p.msg(CORE_WTU, 0, [(5, 3)])
    p.msg(CORE_WTU, 1, [(3, 3)])
    p.msg(CORE_WTU, 2, [(0x81, 8)])
    p.msg(CORE_WTU, 4, [(0x2, 4), (0x12, 8), (77, 16)])
    rows = _same([(p.paragraph(), False, True)])
    assert rows[0][:4] == (1000, 0, 0x70001234, 0x5A)


def test_torn_lapped_and_endoftrace_match():
    # (paragraph, gap, final); Packer paragraphs end in <endoftrace>, which
    # only the final paragraph may (the one marked False is torn for it)
    pars = [(_tsr_par(1000, 2000), False, True), (_tsr_par(3000, 2500), False, True),
            (_tsr_par(4000), False, True), (_tsr_par(5000), False, False)]
    pars += [(_tsr_par(6000 + 1000 * k, 6500 + 1000 * k), False, True) for k in range(6)]
    p = Packer()                                     # a lap later (test_decode's case)
    p.msg(CORE_DTU0, 5, [(0x1, 4), (0, 2), (0x7, 4)])
    p.msg(CORE_TSU, 0, [(19000, 32)])
    p.msg(CORE_DTU0, 4, [(0x2, 4), (0, 2), (0x42, 8), (0x70001234, 32)])
    pars += [(p.paragraph(), False, True), (_tsr_par(19600), False, True)]
    pars += [(_tsr_par(90000), True, True), (_tsr_par(92000), False, True)]
    ref = Decoder()
    for par, g, f in pars:
        ref.rows(par, 0, g, f)
    assert ref.stats.laps == 1 and ref.stats.torn >= 1        # the cases are exercised
    _same(pars)
    _same(pars, batch=1)


def _random_paragraph(rng: random.Random, t: list) -> bytes:
    p = Packer()
    while len(p.bits) < 8000:
        r = rng.random()
        if r < 0.25:
            t[0] = (t[0] + rng.randrange(1, 3000)) & 0xFFFFFFFF
            if rng.random() < 0.5:
                p.msg(CORE_TSU, 0, [(t[0], 32)])
            else:
                p.msg(CORE_TSU, 1, [(rng.getrandbits(12), 12)])
        elif r < 0.55:
            core = rng.choice((CORE_DTU0, CORE_DTU1))
            tt = rng.randrange(8)
            if tt <= 1:
                p.msg(core, tt, [(rng.randrange(4), 2), (rng.getrandbits(32), rng.choice((8, 16, 32)))])
            elif tt <= 3:
                sub = rng.choice((0, 1, 2, 3))
                if sub == 2:
                    p.msg(core, tt, [(2, 2), (rng.randrange(4), 2), (rng.getrandbits(16), 16)])
                else:
                    p.msg(core, tt, [(sub, 2), (rng.getrandbits(37), rng.choice((4, 16, 37)))])
            else:
                l1 = rng.choice((0, 1, 2, 4, 8))
                n1 = {0: 0, 1: 4, 2: 8, 4: 16, 8: 32}[l1]
                p.msg(core, tt, [(l1, 4), (rng.randrange(4), 2), (rng.getrandbits(n1), n1),
                                 (rng.getrandbits(37), rng.choice((0, 8, 32, 37)))])
        elif r < 0.65:
            tt = rng.randrange(8)
            p.msg(CORE_WTU, tt, [(rng.randrange(16), 4), (rng.getrandbits(16), 16)])
        elif r < 0.72:
            p.put(0, 4)                                    # tick
        elif r < 0.77:
            p.put(CODE_MULTICK, 4)
            p.put(rng.randrange(256), 8)
        elif r < 0.80:
            p.put(CODE_ERROR, 4)
            p.put(rng.choice((CORE_DTU0, CORE_TSU, 3)), 5)
            p.put(rng.randrange(128), 7)
        elif r < 0.83:                                     # an <absize> message
            n = rng.randrange(8, 120)
            p.put(CODE_ABSIZE, 4)
            p.put(n, 8)
            p.put(rng.choice((CORE_DTU0, CORE_TSU, CORE_WTU, 2, 0x1F)), 5)
            p.put(rng.randrange(8), 3)
            p.put(rng.getrandbits(n - 8), n - 8)
        elif r < 0.86:                                     # odd units / short messages
            code = rng.choice(list(SYMLEN_BITS))
            p.put(code, 4)
            p.put(rng.getrandbits(SYMLEN_BITS[code]), SYMLEN_BITS[code])
        elif r < 0.87:
            p.bits += [rng.randrange(2) for _ in range(rng.randrange(1, 40))]   # damage
        else:
            continue
    data = p.paragraph()
    return data[:1024] if rng.random() < 0.9 else data[:rng.randrange(1, 1024)]


@pytest.mark.parametrize('seed', range(6))
@pytest.mark.parametrize('tick_mode', [False, True])
def test_random_streams_match(seed, tick_mode):
    rng = random.Random(seed * 7919 + tick_mode)
    t = [rng.getrandbits(32)]
    pars = []
    for i in range(60):
        if rng.random() < 0.05:
            par = bytes(rng.getrandbits(8) for _ in range(1024))      # garbage
        else:
            par = _random_paragraph(rng, t)
        pars.append((par, rng.random() < 0.05, rng.random() < 0.1))
    _same(pars, tick_mode=tick_mode, batch=rng.randrange(1, 9))


def test_real_capture_matches():
    path = os.path.join(HERE, '..', 'cycles.mcds')
    if not os.path.exists(path):
        pytest.skip('no cycles.mcds')
    config, stream = capfile.read_file(path)
    recs = list(capfile.records(stream))
    prev, pars = None, []
    for r in recs:
        pars.append((r.payload, r.gap or (prev is not None and r.seq != prev + 1), r.final))
        prev = r.seq
    _same(pars, tick_mode=config.get('timestamps') == 'ticks', batch=64)
