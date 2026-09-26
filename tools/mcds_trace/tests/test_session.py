"""Record framing, file sessions, live sessions against a fake probe."""

import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from mcds_trace import capfile, session
from mcds_trace.capfile import REC_HEADER, REC_MAGIC
from mcds_trace.signals import raw_signal


def record(seq: int, payload: bytes, lost: int = 0) -> bytes:
    return REC_HEADER.pack(REC_MAGIC, seq, 0, len(payload), 0, lost) + payload


def test_record_stream_across_chunks():
    data = b''.join(record(i, bytes([i]) * 1024) for i in range(5))
    garbage = b'\x00\x01junk'
    rs = session.RecordStream()
    got = []
    blob = garbage + data
    for k in range(0, len(blob), 777):          # arbitrary chunking
        got += rs.feed(blob[k:k + 777])
    assert [r.seq for r in got] == [0, 1, 2, 3, 4]
    assert all(r.payload == bytes([r.seq]) * 1024 for r in got)
    assert rs.resyncs >= 1


def test_event_log_roundtrip_and_limit():
    from mcds_trace.decode import Event
    log = session.EventLog(limit=70000)
    for i in range(140000):
        log.add(Event(i, 'write', addr=0x100 + (i % 4), value=i & 0xFFFF, size=2))
    rows = list(log.rows())
    assert log.dropped > 0 and len(rows) == log.n - log.dropped
    assert rows[-1].cycles == 139999 and rows[-1].value == 139999 & 0xFFFF
    assert rows[0].kind == 'write' and rows[0].addr is not None


def _find_capture():
    for p in ('/tmp/tput.mcds',):
        if os.path.exists(p):
            return p
    return None


@pytest.mark.skipif(_find_capture() is None, reason='no real capture file available')
def test_file_session_real_capture():
    path = _find_capture()
    cfg, _ = capfile.read_file(path)
    slot = [s for s in cfg['slots'] if s.get('enabled')][0]
    addr = int(slot['addr'], 0)
    sig = raw_signal(addr, min(int(slot['size']), 4))
    fs = session.FileSession(path)
    fs.load([sig])
    t, v = fs.store.snapshot(sig.id)
    assert len(t) > 100 and (t[1:] >= t[:-1]).all()
    # Re-selection from the event log gives the same series.
    before = len(t)
    fs.pipe.reselect([sig])
    assert len(fs.store.snapshot(sig.id)[0]) == before
    assert fs.pipe.log.addresses()
    fs.close()


class FakeProbe(BaseHTTPRequestHandler):
    """/api/mcds/* over HTTP; the WebSocket part is replaced in the test."""
    state = {}

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype='application/json'):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        n = int(self.headers['Content-Length'])
        cfg = json.loads(self.rfile.read(n))
        self.state['cfg'] = cfg
        self._send(200, json.dumps(cfg))

    def do_GET(self):
        if self.path == '/api/mcds/start':
            self.state['started'] = True
            self._send(200, 'started emu_hz=100000000 wide=1\n', 'text/plain')
        elif self.path == '/api/mcds/stop':
            self.state['stopped'] = True
            self._send(200, 'stopped\n', 'text/plain')
        elif self.path == '/api/mcds/config':
            c = dict(self.state.get('cfg', {}))
            c.update({'emu_hz': 100000000, 'running': True, 'stats': {'paragraphs': 3}})
            self._send(200, json.dumps(c))
        else:
            self._send(404, 'no')


def test_live_session_with_fake_probe(tmp_path, monkeypatch):
    srv = ThreadingHTTPServer(('127.0.0.1', 0), FakeProbe)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    host = '127.0.0.1:%d' % srv.server_address[1]
    frames = [record(i, b'\xff' * 1024) for i in range(3)]

    class FakeWS:
        def __init__(self):
            self.q = list(frames)

        def settimeout(self, t):
            pass

        def recv(self):
            import websocket
            if self.q and FakeProbe.state.get('started'):    # data only once tracing
                return self.q.pop(0)
            time.sleep(0.05)
            raise websocket.WebSocketTimeoutException()

        def close(self):
            pass

    import websocket
    monkeypatch.setattr(websocket, 'create_connection', lambda *a, **k: FakeWS())
    out = tmp_path / 'live.mcds'
    sig = raw_signal(0x5000220C, 2)
    cfg = {'mode': 'full', 'slots': [{'enabled': True, 'addr': '0x5000220C', 'size': 2,
                                      'access': 'w'}, {'enabled': False}]}
    ls = session.LiveSession(session.Probe(host), cfg, [sig], str(out), in_process=True)
    ls.start()
    time.sleep(0.6)
    ls.stop(timeout=10)
    assert ls.pipe.log.addresses() == {}      # still answers after the stop
    ls.close()
    srv.shutdown()
    assert FakeProbe.state.get('started') and FakeProbe.state.get('stopped')
    assert ls.stats.frames == 3 and ls.stats.records == 3 and ls.error is None
    config, stream = capfile.read_file(str(out))
    assert config['emu_hz'] == 100000000
    assert len(list(capfile.records(stream))) == 3


def test_file_session_in_a_worker_process(tmp_path):
    # The real (spawned) worker: a small synthetic capture.
    from test_decode import _tsr_par
    path = tmp_path / 'syn.mcds'
    with open(path, 'wb') as f:
        capfile.write_header(f, {'emu_hz': 1000, 'slots': []})
        for i in range(5):
            par = _tsr_par(1000 * i + 10, 1000 * i + 500).ljust(1024, bytes(1))
            f.write(REC_HEADER.pack(REC_MAGIC, i, 0, len(par), capfile.FLAG_FINAL, 0) + par)
    sig = raw_signal(0x70001234, 1)
    fs = session.FileSession(str(path))
    fs.load([sig])
    t, v = fs.store.snapshot(sig.id)
    assert len(t) == 10 and fs.stats.records == 5
    assert fs.pipe.log.addresses() == {(0x70001234, 1): 10}
    fs.close()


def test_probe_errors_are_readable():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    p = session.Probe('127.0.0.1:%d' % port, timeout=1)
    with pytest.raises(session.ProbeError):
        p.config()
    with pytest.raises(session.ProbeError):
        session.Probe('1.2.3.4:abc').config()
