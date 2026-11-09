"""Program trace: PTU message decoding, flow reconstruction, coverage, exports."""

import json

import numpy as np

from mcds_trace import codemodel as cm
from mcds_trace import ptrace_export as px
from mcds_trace.ptrace import EV_BEGIN, EV_END, FlowDecoder
from mcds_trace.tram import CORE_PTU, CORE_TSU

from test_decode import Packer

# main: 1000 mov, 1004 call f, 1008 j main;  f: 2000 add, 2002 jne f, 2006 ret
# irq:  3000 nop, 3002 rfe
OBJDUMP = """
/x/app.elf:     file format elf32-tricore

Disassembly of section .text:

00001000 <main>:
main():
/src/main.c:10
    1000:\t3b 00 00 20 \tmov %d2,0
/src/main.c:11
    1004:\t6d 00 fe 07 \tcall 2000 <f>
/src/main.c:12
    1008:\t3c fc       \tj 1000 <main>

00002000 <f>:
f():
/src/f.c:3
    2000:\t42 f2       \tadd %d2,%d15
/src/f.c:4
    2002:\t5f f2 ff 7f \tjne %d2,%d15,2000 <f>
/src/f.c:5
    2006:\t00 90       \tret

00003000 <irq>:
irq():
/src/irq.c:7
    3000:\t00 00       \tnop
    3002:\t00 80       \trfe
""".replace('    1', '1').replace('    2', '2').replace('    3', '3')


def _model():
    addr, size, kind, target, line, lines, files, _ = cm.parse_objdump(OBJDUMP)
    return cm.CodeModel('x', np.array(addr, np.uint32), np.array(size, np.uint8),
                        np.array(kind, np.uint8), np.array(target, np.uint32),
                        np.array(line, np.int32), lines, files,
                        np.array([0x1000, 0x2000, 0x3000], np.uint32),
                        np.array([0x100A, 0x2008, 0x3004], np.uint32), ['main', 'f', 'irq'])


def test_objdump_kinds():
    m = _model()
    assert [cm.KIND_NAMES[k] for k in m.kind] == ['seq', 'call', 'jump', 'seq', 'cond', 'ret',
                                                 'seq', 'rfe']
    assert m.size.tolist() == [4, 4, 2, 2, 4, 2, 2, 2]
    assert m.target[1] == 0x2000 and m.target[4] == 0x2000
    assert m.source_of(4) == ('/src/f.c', 4)


def _stream():
    p = Packer()
    p.msg(CORE_TSU, 0, [(1000, 32)])                 # TSR: time 1000 cycles
    p.msg(CORE_PTU, 0, [(0x1000, 32)])               # IPA main (start of trace)
    p.msg(CORE_PTU, 2, [(8, 8)])                     # IPI 8: mov, call f
    p.msg(CORE_PTU, 3, [(8 ^ 6, 8)])                 # IPI 6 (compressed): add, jne taken
    p.msg(CORE_PTU, 3, [(6 ^ 2, 8)])                 # IPI 2: add, then an interrupt
    p.msg(CORE_PTU, 1, [(0x1000 ^ 0x3000, 16)])      # IPA irq (compressed vs 0x1000)
    p.msg(CORE_PTU, 3, [(2 ^ 4, 8)])                 # IPI 4: nop, rfe
    p.msg(CORE_PTU, 1, [(0x3000 ^ 0x2002, 16)])      # IPA back at 2002
    p.msg(CORE_PTU, 3, [(4 ^ 6, 8)])                 # IPI 6: jne not taken, ret
    p.msg(CORE_PTU, 1, [(0x2002 ^ 0x1008, 16)])      # IPA 1008
    p.msg(CORE_TSU, 0, [(1100, 32)])                 # TSR: 1100
    p.msg(CORE_PTU, 3, [(6 ^ 2, 8)])                 # IPI 2: j main
    p.msg(CORE_PTU, 4, [(0, 4), (4, 8)])             # IPE: data 1 empty, 4 bytes (mov)
    return p.paragraph()


def test_flow_coverage_and_calls():
    dec = FlowDecoder(_model(), emu_hz=100e6)
    dec.paragraph(_stream())
    dec.finish()
    st = dec.stats
    assert (st.ipa, st.ipi, st.ipe, st.desync, st.unsynced) == (4, 6, 1, 0, 0)
    assert dec.counts().tolist() == [2, 1, 1, 2, 2, 1, 1, 1]
    seq = [(k, dec.names[n]) for _, k, n in dec.events]
    assert seq == [(EV_BEGIN, 'main'), (EV_BEGIN, 'f'), (EV_BEGIN, 'irq irq'), (EV_END, 'irq irq'),
                   (EV_END, 'f'), (EV_END, 'main')]
    assert st.calls == 1 and st.returns == 2 and st.interrupts == 1
    # time: 1000 cycles at byte 0, 1100 at byte 26 (8+6+2+4+6); 10 ns a cycle
    t = dec.times_ns([0, 26])
    assert abs(t[1] - t[0] - 1000.0) < 1e-6


def test_gap_and_err_resync():
    p = Packer()
    p.msg(CORE_PTU, 0, [(0x1000, 32)])
    p.msg(CORE_PTU, 2, [(8, 8)])                     # into f
    p.put(0x1, 4); p.put(CORE_PTU, 5); p.put(3, 7)   # ERR: 3 IPI lost
    p.msg(CORE_PTU, 3, [(8 ^ 6, 8)])                 # ignored: flow unknown
    p.msg(CORE_PTU, 0, [(0x2000, 32)])               # first IPA after ERR: uncompressed
    p.msg(CORE_PTU, 2, [(6, 8)])
    dec = FlowDecoder(_model())
    dec.paragraph(p.paragraph())
    dec.finish()
    assert dec.stats.err == 1 and dec.stats.lost_ipi == 3 and dec.stats.unsynced == 1
    assert dec.counts().tolist()[3:5] == [1, 1]


def test_nesting_mode_function_coverage(tmp_path):
    p = Packer()
    # IPC: subtype 0, L1 code 8 (32 bits): take-off 1008, landing 2000
    p.msg(CORE_PTU, 6, [(0, 2), (0x8, 4), (0x1008, 32), (0x2000, 32)])
    p.msg(CORE_PTU, 6, [(1, 2), (0x8, 4), (0x2008, 32), (0x1008, 32)])   # IPR
    dec = FlowDecoder(_model())
    dec.paragraph(p.paragraph())
    dec.finish()
    cov = px.coverage(dec)
    assert cov.function_level
    assert {f.name: f.hit > 0 for f in cov.funcs} == {'main': False, 'f': True, 'irq': False}


def test_exports(tmp_path):
    dec = FlowDecoder(_model(), emu_hz=100e6)
    dec.paragraph(_stream())
    dec.finish()
    cov = px.coverage(dec)
    assert cov.insns == 8 and cov.hit == 8
    px.write_lcov(cov, str(tmp_path / 'c.info'))
    info = (tmp_path / 'c.info').read_text()
    assert 'SF:/src/f.c' in info and 'DA:4,2' in info and 'FNDA:1,f' in info
    px.write_html(cov, str(tmp_path / 'c.html'), 'test')
    assert '<table' in (tmp_path / 'c.html').read_text()
    n = px.write_chrome_json(dec, str(tmp_path / 't.json'), cpu=2)
    j = json.loads((tmp_path / 't.json').read_text())
    assert n == 3 and sum(e['ph'] == 'B' for e in j['traceEvents']) == 3
    assert px.write_perfetto(dec, str(tmp_path / 't.pftrace'), cpu=2) == 3
    assert (tmp_path / 't.pftrace').stat().st_size > 50


def test_ipi0_waits_for_the_ipa():
    p = Packer()
    p.msg(CORE_PTU, 0, [(0x1008, 32)])               # sync at the j
    p.msg(CORE_PTU, 2, [(2, 8)])                     # IPI 2: j main -> ip 1000
    p.msg(CORE_PTU, 3, [(2 ^ 0, 8)])                 # IPI 0: exception before mov ran
    p.msg(CORE_PTU, 1, [(0x1008 ^ 0x3000, 16)])      # IPA: the handler
    p.msg(CORE_PTU, 3, [(0 ^ 4, 8)])                 # IPI 4: nop, rfe
    p.msg(CORE_PTU, 1, [(0x3000 ^ 0x1000, 16)])      # IPA: back to mov
    p.msg(CORE_PTU, 3, [(4 ^ 8, 8)])                 # IPI 8: mov, call f
    p.msg(CORE_PTU, 3, [(8 ^ 0, 8)])                 # IPI 0 at f's add (not a branch):
    p.msg(CORE_PTU, 1, [(0x1000 ^ 0x3000, 16)])      # an exception before it, IPA irq
    dec = FlowDecoder(_model())
    dec.paragraph(p.paragraph())
    dec.finish()
    c = dec.counts().tolist()
    assert c[:3] == [1, 1, 1] and c[3] == 0 and c[6:] == [1, 1]
    assert dec.stats.desync == 0 and dec.stats.interrupts == 2
    names = [dec.names[n] for _, k, n in dec.events if k == EV_BEGIN]
    assert names == ['main', 'irq irq', 'f', 'irq irq']


def test_bursts_keep_time_per_window():
    """Two trace windows (bursts): a TSR at both ends of each, the gap between
    them is real time, not spread over the instructions."""
    p = Packer()
    for t0, t1 in ((1000, 1080), (9000, 9080)):
        p.msg(CORE_TSU, 0, [(t0, 32)])               # burst start
        p.msg(CORE_PTU, 0, [(0x1000, 32)])           # IPA main
        p.msg(CORE_PTU, 2, [(8, 8)])                 # mov, call f
        p.msg(CORE_PTU, 4, [(0, 4), (2, 8)])         # IPE: add, window closed
        p.msg(CORE_TSU, 0, [(t1, 32)])               # burst end
    dec = FlowDecoder(_model(), emu_hz=100e6)
    dec.paragraph(p.paragraph())
    dec.finish()
    begins = [i for i, (_, k, _) in enumerate(dec.events) if k == EV_BEGIN]
    t = dec.times_ns([dec.events[i][0] for i in begins], [dec.event_segs[i] for i in begins])
    # main of burst 1 at 0 ns, f at 8/10 of 80 cycles; burst 2 starts 8000 cycles later
    assert abs(t[0]) < 1e-6 and abs(t[1] - 640.0) < 1e-6
    assert abs(t[2] - 80000.0) < 1e-6 and abs(t[3] - 80640.0) < 1e-6


def test_calls_mode_ipa_is_a_direct_call():
    from mcds_trace.ptrace import decode_records
    from mcds_trace.capfile import Record
    p = Packer()
    p.msg(CORE_TSU, 0, [(1000, 32)])
    p.msg(CORE_PTU, 0, [(0x1000, 32)])                                     # start: in main
    p.msg(CORE_PTU, 1, [(0x1000 ^ 0x2000, 16)])                            # IPA: call f
    p.msg(CORE_PTU, 6, [(0, 2), (0x8, 4), (0x2004, 32), (0x3000, 32)])     # IPC: f calls irq()
    p.msg(CORE_PTU, 7, [(1, 2), (0x0, 4), (0x3000 ^ 0x2008, 16)])          # IPR to 2008
    p.msg(CORE_PTU, 6, [(1, 2), (0x8, 4), (0x2008, 32), (0x1008, 32)])     # IPR to main
    p.msg(CORE_TSU, 0, [(1100, 32)])
    dec = decode_records([Record(0, 0, 2, 0, p.paragraph())], _model(), 100e6)
    assert dec.calls_mode
    names = [(k, dec.names[n]) for _, k, n in dec.events]
    assert names == [(EV_BEGIN, 'main'), (EV_BEGIN, 'f'), (EV_BEGIN, 'irq'), (EV_END, 'irq'),
                     (EV_END, 'f'), (EV_END, 'main')]
    assert dec.stats.bytes_executed == 0 and dec.stats.calls == 2 and dec.stats.returns == 2
    cov = px.coverage(dec)
    assert cov.function_level
    assert {f.name for f in cov.funcs if f.hit} == {'f', 'irq'}


def test_return_unwinds_past_calls_without_returns():
    """Compact function trace: a short leaf function's call comes, its return
    does not; the caller's return unwinds through it."""
    p = Packer()
    p.msg(CORE_PTU, 0, [(0x1000, 32)])                                     # in main
    p.msg(CORE_PTU, 1, [(0x1000 ^ 0x2000, 16)])                            # call f
    p.msg(CORE_PTU, 6, [(0, 2), (0x8, 4), (0x2004, 32), (0x3000, 32)])     # f -> irq (short leaf)
    p.msg(CORE_PTU, 6, [(1, 2), (0x8, 4), (0x2008, 32), (0x1008, 32)])     # f returns to main
    dec = FlowDecoder(_model(), mode='calls')
    dec.paragraph(p.paragraph())
    seq = [(k, dec.names[n]) for _, k, n in dec.events]
    assert seq == [(EV_BEGIN, 'main'), (EV_BEGIN, 'f'), (EV_BEGIN, 'irq'), (EV_END, 'irq'),
                   (EV_END, 'f')]


def test_ipa_zero_is_no_ip():
    p = Packer()
    p.msg(CORE_PTU, 0, [(0x1000, 32)])               # in main
    p.msg(CORE_PTU, 2, [(8, 8)])                     # mov, call f
    p.msg(CORE_PTU, 4, [(0, 4), (2, 8)])             # IPE: window closed after add
    p.msg(CORE_PTU, 0, [(0, 32)])                    # IPA 0: no valid IP
    p.msg(CORE_PTU, 2, [(6, 8)])                     # unsynced
    p.msg(CORE_PTU, 0, [(0x2000, 32)])               # a real one
    p.msg(CORE_PTU, 2, [(6, 8)])
    dec = FlowDecoder(_model())
    dec.paragraph(p.paragraph())
    st = dec.stats
    assert (st.invalid_ipa, st.desync, st.unsynced) == (1, 0, 1)
    assert dec.counts().tolist()[3:5] == [2, 1]


def test_demangled_targets_parse():
    text = ("80049b9a:\t3c 13       \tj 80049bc0 <basic_software::isotp::IsoTpEngine::parse("
            "me::Span<unsigned char const, 4294967295ul>, bool)+0x26>\n"
            "80049d04:\t6d 00 10 00 \tcall 80049d24 <isoTp_rxCallback>\n"
            "80049d08:\t5f f2 ff 7f \tjne %d2,%d15,80049d06 <f(std::array<int, 3ul>&)+0x2>\n")
    addr, size, kind, target, *_ = cm.parse_objdump(text)
    assert [hex(t) for t in target] == ['0x80049bc0', '0x80049d24', '0x80049d06']
    assert [cm.KIND_NAMES[k] for k in kind] == ['jump', 'call', 'cond']
