"""The tdm (tricore-debug-machine, USB) trace source, without hardware."""

import struct
import threading
import time

import pytest

from mcds_trace import capfile, session, tdm
from mcds_trace.signals import Signal

from test_decode import _tsr_par


def test_config_layout_and_round_trip():
    cfg = {'source': 'memslave', 'cpu': 2, 'mode': 'full', 'payload': 'data',
           'timestamps': 'ticks', 'masters': True, 'dap_div': 3, 'wide': True, 'fast': False,
           'slots': [{'enabled': True, 'name': 'app', 'addr': '0x50002208', 'size': 0x140,
                      'access': 'w', 'value': {'enabled': True, 'lo': '0x10', 'hi': 20,
                                               'mask': '0xFF', 'signed': True}},
                     {'enabled': False}]}
    raw = tdm.pack_config(cfg)
    assert len(raw) == tdm.CFG_SIZE == 152
    # as tricore-debug-machine's tdm_dap.py addresses it
    assert raw[0] == 1 and raw[1] == 2 and raw[6] == 3 and raw[7] == 1 and raw[8] == 0
    assert raw[12] == 1 and struct.unpack_from('<II', raw, 12 + 28) == (0x50002208, 0x140)
    assert (raw[12 + 36], raw[12 + 37]) == (0, 1)
    back = tdm.unpack_config(raw)
    for k in ('source', 'cpu', 'mode', 'payload', 'timestamps', 'masters', 'dap_div', 'wide',
              'fast'):
        assert back[k] == cfg[k], k
    s = back['slots'][0]
    assert (s['name'], s['addr'], s['size'], s['access']) == ('app', '0x50002208', 0x140, 'w')
    assert s['value'] == {'enabled': True, 'lo': '0x00000010', 'hi': '0x00000014',
                          'mask': '0x000000FF', 'signed': True}
    assert back['slots'][1]['enabled'] is False


def test_config_checks():
    with pytest.raises(tdm.TdmError, match='no enabled slot'):
        tdm.pack_config({'slots': [{'enabled': False}]})
    with pytest.raises(tdm.TdmError, match='size 0'):
        tdm.pack_config({'slots': [{'addr': 1, 'size': 0}]})


def test_info_and_stats():
    snap = bytes(range(16))
    raw = bytearray(tdm.INFO_SIZE)
    struct.pack_into('<IIBB', raw, 0, 150_000_000, 1234, 1, 1)
    struct.pack_into('<IH', raw, 12, 0x50002208, len(snap))
    raw[20:20 + len(snap)] = snap
    info = tdm.unpack_info(bytes(raw))
    assert info['emu_hz'] == 150_000_000 and info['wide_active'] and info['fast_active']
    assert info['snapshot'] == [{'slot': 0, 'addr': '0x50002208', 'hex': snap.hex()}]
    st = tdm.unpack_stats(struct.pack('<B3x14I', 1, *range(1, 15)))
    assert st['running'] and st['paragraphs'] == 1 and st['queue_size'] == 14


def test_addresses():
    assert tdm.is_tdm('tdm') and tdm.is_tdm(' USB ') and tdm.is_tdm('tdm:5100')
    assert not tdm.is_tdm('192.168.178.99') and not tdm.is_tdm('tdmhost')
    assert tdm.serial_of('tdm:5100') == '5100' and tdm.serial_of('tdm') is None
    assert isinstance(session.open_probe('tdm'), tdm.TdmSource)
    assert isinstance(session.open_probe('10.0.0.1'), session.Probe)


def _record(seq, payload, flags=0):
    return struct.pack('<IIIHHI', capfile.REC_MAGIC, seq, 0, len(payload), flags, 0) + payload


class FakeTdm:
    """TdmProbe's interface over a scripted trace."""
    instances = []
    lock = threading.Lock()

    def __init__(self, serial=None):
        with FakeTdm.lock:
            if any(not p.closed for p in FakeTdm.instances):
                raise tdm.TdmError('busy')                 # one owner at a time, as WinUSB
            FakeTdm.instances.append(self)
        self.closed = False
        self.cfg = None
        self.started = self.stopped = False
        self.queue = [_record(7, _tsr_par(1))]            # left by an earlier trace
        self.info = {}

    def drain(self):
        self.queue.clear()

    def read(self, timeout_ms=200):
        if self.queue:
            return self.queue.pop(0)
        time.sleep(0.01)
        return None

    def post_config(self, cfg):
        self.cfg = tdm.unpack_config(tdm.pack_config(cfg))
        return dict(self.cfg)

    def config(self):
        out = dict(self.cfg or {})
        out.update({'emu_hz': 100_000_000, 'wide_active': True, 'fast_active': True,
                    'snapshot': [], 'stats': {'paragraphs': 3}, 'probe': 'tdm',
                    'running': self.started and not self.stopped})
        return out

    def status(self):
        return 'tdm FAKE attached cores=4 bin=48MHz 24V=on'

    def start(self):
        self.started = True
        for k in range(3):
            # (FINAL on each: Packer paragraphs end in <endoftrace>)
            self.queue.append(_record(k, _tsr_par(1000 * k + 10, 1000 * k + 500), flags=2))
        return 'started emu_hz=100000000 wide=1 fast=1'

    def stop(self):
        self.stopped = True
        self.queue.append(_record(3, _tsr_par(3010), flags=2))
        return 'stopped paragraphs=4 lost=0 queue_dropped=0'

    def close(self):
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def test_live_session_over_usb(monkeypatch, tmp_path):
    FakeTdm.instances = []
    monkeypatch.setattr(tdm, 'TdmProbe', FakeTdm)
    out = tmp_path / 'usb.mcds'
    sig = Signal('raw:0x70001234:1', 'v', 0x70001234, 1)
    cfg = {'slots': [{'enabled': True, 'addr': '0x70001234', 'size': 1, 'access': 'w'}]}
    probe = session.open_probe('tdm')
    ls = session.LiveSession(probe, cfg, [sig], str(out), in_process=True)
    ls.start()
    assert len(FakeTdm.instances) == 1                    # the worker owns the probe
    assert ls.config['probe'] == 'tdm' and ls.config['emu_hz'] == 100_000_000
    assert probe.status().startswith('tdm FAKE')         # through the worker
    time.sleep(0.3)
    ls.stop()
    assert ls.error is None, ls.error
    assert FakeTdm.instances[0].stopped and FakeTdm.instances[0].closed
    assert probe.status().startswith('tdm FAKE')         # the UI opens it itself now
    _, v = ls.store.snapshot(sig.id)
    assert list(v) == [10, 244, 242, 220, 218, 196, 194]  # (1000k+10, 1000k+500) & 0xFF, 3010
    config, stream = capfile.read_file(str(out))
    recs = list(capfile.records(stream))
    assert [r.seq for r in recs] == [0, 1, 2, 3] and recs[-1].final
    assert config['probe'] == 'tdm'
    ls.close()
