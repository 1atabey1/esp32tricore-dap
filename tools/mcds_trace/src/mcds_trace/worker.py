"""The trace data path, run apart from the UI.

A worker owns everything between the probe and the samples: the stream (the
esp32jtag's WebSocket, or the tdm probe's USB endpoint), the capture file,
the decoder, the event log and the extractor.  It sends the samples it
produces to the UI in batches over a multiprocessing Connection and answers
requests (a new signal selection, the accessed addresses, and the commands
for a tdm probe, whose USB interface it holds).  In a separate process it
has its own interpreter lock, so neither a busy UI nor rendering can slow
the stream down - the file stays complete and the probe's buffer drains.
Tests run the same code in a thread.

Messages to the UI: ('ready',) ('error', text) ('config', dict)
('batch', series, gaps, t_end, samples, stats) ('store', series, gaps, t_end,
samples) ('addresses', dict) ('progress', fraction) ('loaded', stats)
('done', stats, error) ('ctl', result).  Requests: ('select', signals)
('addresses',) ('ctl', method, args) ('stop',) ('abort',) ('close',).
"""

from __future__ import annotations

import base64
import queue
import threading
import time
from dataclasses import asdict

import struct

from . import capfile, fastextract
from .capfile import REC_HEADER, REC_MAGIC, write_header
from .session import Probe, RecordStream, SessionStats, _Pipeline
from .zframe import QUERY, unframe

BATCH_S = 0.05                # how often samples go to the UI
QUIET_S = 1.5                 # after stop: the stream has ended once it is quiet this long


_MAGIC = struct.pack('<I', REC_MAGIC)


def _record_starts(data: bytes, seq: int) -> list[int]:
    """Offsets of record headers with this sequence number."""
    out, pos = [], data.find(_MAGIC)
    while pos >= 0:
        if len(data) - pos >= REC_HEADER.size:
            m, sq, _, length, _, _ = REC_HEADER.unpack_from(data, pos)
            if sq == seq and 0 < length <= 65535:
                out.append(pos)
        pos = data.find(_MAGIC, pos + 1)
    return out


class _StartGate:
    """Passes the stream from the new trace's first record (sequence 0) on:
    bytes an earlier trace left in the probe come before it."""

    def __init__(self):
        self.open = False
        self.buf = b''

    def start(self, early: bytes) -> bytes:
        # The last first record received so far is the new trace's.
        starts = _record_starts(early, 0)
        if starts:
            self.open = True
            return early[starts[-1]:]
        self.buf = early[-(REC_HEADER.size - 1):]      # a header may straddle frames
        return b''

    def feed(self, data: bytes) -> bytes:
        if self.open:
            return data
        buf = self.buf + data
        starts = _record_starts(buf, 0)
        if starts:
            self.open = True
            self.buf = b''
            return buf[starts[0]:]
        self.buf = buf[-(REC_HEADER.size - 1):]
        return b''


def _take_samples(pipe: _Pipeline) -> tuple[dict, list, float, int]:
    """What the extractor produced since the last call (and forget it here:
    the UI keeps the samples)."""
    store = pipe.store
    with store.lock:
        series = {}
        for sid, s in store.series.items():
            if s.n:
                t, v = s.view()
                series[sid] = (t.copy(), v.copy())
                s.n = 0
        gaps, store.gaps = store.gaps, []
        samples, store.samples = store.samples, 0
        return series, gaps, store.t_end, samples


def _stats(st: SessionStats, pipe: _Pipeline) -> dict:
    return {'bytes': st.bytes, 'frames': st.frames, 'records': pipe.records,
            'probe_lost': st.probe_lost, 'decode_skipped': st.decode_skipped,
            'backlog': st.backlog, 'rate': st.rate,
            'decode': asdict(pipe.decoder.stats), 'extract': vars(pipe.extractor.stats).copy(),
            'log_n': pipe.log.n, 'log_dropped': pipe.log.dropped}


class _Serve:
    """Requests common to live and file workers."""

    def __init__(self, conn, pipe: _Pipeline):
        self.conn = conn
        self.pipe = pipe
        self.gap_history: list[float] = []

    def send_batch(self, st: SessionStats) -> None:
        series, gaps, t_end, samples = _take_samples(self.pipe)
        self.gap_history += gaps
        if series or gaps or samples:
            self.conn.send(('batch', series, gaps, t_end, samples, _stats(st, self.pipe)))
        else:
            self.conn.send(('batch', {}, [], t_end, 0, _stats(st, self.pipe)))

    def handle(self, cmd, st: SessionStats) -> str | None:
        """Answer a request; returns the kinds the caller acts on itself.  A
        request that fails is answered with ('failed', text)."""
        try:
            return self._handle(cmd, st)
        except (EOFError, OSError, BrokenPipeError):
            raise
        except Exception as e:
            self.conn.send(('failed', '%s: %s' % (cmd[0], e)))
            return None

    def _handle(self, cmd, st: SessionStats) -> str | None:
        kind = cmd[0]
        if kind == 'select':
            self.send_batch(st)                    # the old selection's last samples
            self.pipe.store.gaps = list(self.gap_history)
            self.pipe.reselect(cmd[1])
            series, gaps, t_end, samples = _take_samples(self.pipe)
            self.conn.send(('store', series, gaps, t_end, samples))
            return None
        if kind == 'addresses':
            self.conn.send(('addresses', self.pipe.log.addresses()))
            return None
        return kind


class _StreamClosed(Exception):
    pass


class _WsStream:
    """The esp32jtag: /ws/trace (LZ4-framed); commands are HTTP, sent by the UI."""

    def __init__(self, host: str, auth: str):
        import websocket
        self._ws_mod = websocket
        self.host, self.auth = host, auth
        hdr = {'Authorization': 'Basic ' + base64.b64encode(auth.encode()).decode()}
        # Connect and handshake get seconds: over Wi-Fi, with the probe's one
        # HTTP task serving other requests, 0.2 s was often not enough ("trace
        # stream: timed out").  Only the drain of leftovers below is short.
        # LZ4-compressed frames (zframe): Wi-Fi limits a fast trace, not the DAP.
        ws = websocket.create_connection('ws://%s/ws/trace%s' % (host, QUERY), header=hdr,
                                         timeout=5)
        ws.settimeout(0.2)
        quiet_since, t0 = time.monotonic(), time.monotonic()
        while time.monotonic() - quiet_since < 0.3 and time.monotonic() - t0 < 5:
            try:
                if ws.recv():
                    quiet_since = time.monotonic()
            except websocket.WebSocketTimeoutException:
                pass
        ws.settimeout(1)
        self.ws = ws

    def recv(self) -> bytes | None:
        websocket = self._ws_mod
        try:
            frame = self.ws.recv()
        except websocket.WebSocketTimeoutException:
            return None
        except (websocket.WebSocketConnectionClosedException, OSError):
            raise _StreamClosed('the probe closed the trace stream') from None
        if not frame or isinstance(frame, str):
            return None
        return unframe(frame)

    def control(self, method: str, args: tuple):
        raise RuntimeError('the esp32jtag is controlled over HTTP, not through the worker')

    def stop_trace(self) -> None:
        Probe(self.host, self.auth, timeout=10).stop()

    def close(self) -> None:
        try:
            self.ws.close()
        except Exception:
            pass


class _TdmStream:
    """The tdm probe over USB: the trace endpoint, and the commands (the
    worker owns the device for the session; see tdm.TdmSource)."""

    def __init__(self, serial: str | None):
        from .tdm import TdmProbe
        self.probe = TdmProbe(serial)
        self._closed = False
        self.probe.drain()

    def recv(self) -> bytes | None:
        from .tdm import TdmError
        if self._closed:
            raise _StreamClosed('closed')
        try:
            return self.probe.read(200)
        except TdmError as e:
            raise _StreamClosed(str(e)) from None

    def control(self, method: str, args: tuple):
        if self._closed:
            raise RuntimeError('the trace worker has released the probe')
        if method not in ('status', 'config', 'post_config', 'start', 'stop'):
            raise RuntimeError('unknown probe command %r' % method)
        return getattr(self.probe, method)(*args)

    def stop_trace(self) -> None:
        self.probe.stop()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self.probe.close()


def open_stream(spec: tuple):
    """('ws', host, auth) or ('tdm', serial)."""
    if spec[0] == 'tdm':
        return _TdmStream(spec[1])
    return _WsStream(spec[1], spec[2])


def _answer_control(conn, stream, cmd) -> None:
    """('ctl', method, args) from the UI: a probe command (tdm)."""
    try:
        result = stream.control(cmd[1], tuple(cmd[2]))
    except Exception as e:
        conn.send(('failed', '%s: %s' % (cmd[1], e)))
        return
    conn.send(('ctl', result))


def live_main(conn, spec: tuple, out_path: str, signals: list,
              log_limit: int | None, max_backlog: int) -> None:
    """The stream is connected before the trace starts, so its first
    paragraphs cannot overflow the probe's buffer while this process comes
    up: connect, drop what an earlier session left, report 'ready', then wait
    for ('begin', config) - the trace has started - to write the file.
    spec: see open_stream; a tdm probe's commands ('ctl') are answered here."""
    st = SessionStats()
    try:
        f = open(out_path, 'wb')
    except OSError as e:
        conn.send(('error', 'capture file: %s' % e))
        return
    try:
        fastextract.warm()              # before the stream: a cold JIT takes seconds
    except Exception as e:
        conn.send(('error', 'decoder: %s' % e))
        f.close()
        return
    try:
        stream = open_stream(spec)
    except Exception as e:
        f.close()
        conn.send(('error', 'trace stream: %s' % e))
        return
    conn.send(('ready',))

    q: queue.Queue = queue.Queue()
    stop = threading.Event()
    lock = threading.Lock()            # the file start and SessionStats, shared with rx
    error: list[str] = []
    early: list[bytes] = []            # frames before the header is written
    begun = [False]
    gate = _StartGate()

    def take(frame: bytes) -> None:
        frame = gate.feed(frame)
        if not frame:
            return
        f.write(frame)
        st.bytes += len(frame)
        st.frames += 1
        st.backlog += len(frame)
        q.put(frame)

    def rx() -> None:
        idle_since = None
        last = time.monotonic()
        window = 0
        try:
            while True:
                if stop.is_set():
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since > QUIET_S:
                        break               # the stream has gone quiet after stop
                try:
                    frame = stream.recv()
                except _StreamClosed as e:
                    if not stop.is_set():
                        error.append(str(e))
                    break
                now = time.monotonic()
                if frame:
                    with lock:
                        if begun[0]:
                            take(frame)
                        else:
                            early.append(frame)
                    window += len(frame)
                    idle_since = None if not stop.is_set() else now
                if now - last >= 0.5:
                    st.rate = 0.6 * st.rate + 0.4 * window / (now - last)
                    window = 0
                    last = now
        except Exception as e:                           # keep the file consistent
            error.append('receiver: %s' % e)
        finally:
            try:
                f.flush()
            except (OSError, ValueError):
                pass
            q.put(None)

    rx_thread = threading.Thread(target=rx, name='trace-rx', daemon=True)
    rx_thread.start()
    # The trace starts now; its configuration (clock, snapshot) comes with 'begin'.
    try:
        while True:
            cmd = conn.recv()
            if cmd[0] == 'begin':
                config = cmd[1]
                break
            if cmd[0] == 'ctl':
                _answer_control(conn, stream, cmd)
                continue
            if cmd[0] in ('abort', 'close', 'stop'):
                raise EOFError
    except (EOFError, OSError):
        stop.set()
        rx_thread.join(timeout=QUIET_S + 2)
        stream.close()
        f.close()
        return
    pipe = _Pipeline(config, signals, log_limit=log_limit)
    with lock:
        write_header(f, config)
        first = gate.start(b''.join(early))
        early.clear()
        if first:
            take(first)
        begun[0] = True
    st.started = time.monotonic()
    serve = _Serve(conn, pipe)
    rs = RecordStream()
    skip_gap = False
    decoding = True
    last_batch = time.monotonic()
    try:
        while True:
            try:
                while conn.poll():
                    cmd = conn.recv()
                    if cmd[0] == 'ctl':
                        _answer_control(conn, stream, cmd)
                        continue
                    kind = serve.handle(cmd, st)
                    if kind == 'stop':
                        stop.set()
                    elif kind in ('abort', 'close'):
                        stop.set()
                        decoding = False
                        if spec[0] == 'tdm':
                            try:
                                stream.stop_trace()      # nobody records it now
                            except Exception:
                                pass
                        rx_thread.join(timeout=2)
                        stream.close()
            except (EOFError, OSError):                  # the UI is gone
                if not stop.is_set():
                    try:
                        stream.stop_trace()              # nobody records it now
                    except Exception:
                        pass
                stop.set()
                decoding = False
                rx_thread.join(timeout=2)
                stream.close()
            try:
                item = q.get(timeout=BATCH_S)
            except queue.Empty:
                item = b''
            if item is None:
                break
            if item and decoding:
                with lock:
                    st.backlog -= len(item)
                    behind = st.backlog > max_backlog
                if behind:
                    # Behind: drop what is queued from the live view (the file
                    # has it) and restart the decoder at the next record.
                    dropped = 0
                    while True:
                        try:
                            nxt = q.get_nowait()
                        except queue.Empty:
                            break
                        if nxt is None:
                            q.put(None)
                            break
                        dropped += len(nxt)
                    with lock:
                        st.backlog -= dropped
                    st.decode_skipped += dropped // (REC_HEADER.size + 1024)
                    rs = RecordStream()
                    skip_gap = True
                else:
                    try:
                        recs = rs.feed(item)
                        if recs:
                            for r in recs:
                                st.probe_lost += r.lost
                            pipe.records_in(recs, forced_gap=skip_gap)
                            skip_gap = False
                    except Exception as e:
                        # The live view ends here; the file is still written.
                        decoding = False
                        error.append('decoder: %s' % e)
                        conn.send(('error', 'the live view stopped (decoder: %s); the capture '
                                            'file is still being written' % e))
            elif item:
                with lock:
                    st.backlog -= len(item)
            now = time.monotonic()
            if now - last_batch >= BATCH_S:
                last_batch = now
                try:
                    serve.send_batch(st)
                except (EOFError, OSError, BrokenPipeError):
                    decoding = False
                    stop.set()
    except Exception as e:
        error.append('decoder: %s' % e)
        stop.set()
    finally:
        rx_thread.join(timeout=QUIET_S + 5)
        stream.close()                  # a tdm probe is free for the UI again
        try:
            f.close()
        except (OSError, ValueError):
            pass
        try:
            serve.send_batch(st)
            conn.send(('done', _stats(st, pipe), error[0] if error else None))
            # Stay for re-selections of the recorded trace until the UI
            # closes the session.
            while decoding:
                cmd = conn.recv()
                if cmd[0] == 'ctl':
                    _answer_control(conn, stream, cmd)     # fails: the probe is released
                elif serve.handle(cmd, st) in ('abort', 'close'):
                    break
        except (EOFError, OSError, BrokenPipeError):
            pass
        conn.close()


def file_main(conn, path: str, signals: list) -> None:
    st = SessionStats()
    try:
        config, stream = capfile.read_file(path)
        recs = list(capfile.records(stream))
    except Exception as e:
        conn.send(('error', str(e)))
        return
    conn.send(('config', config))
    fastextract.warm()
    pipe = _Pipeline(config, signals)
    serve = _Serve(conn, pipe)
    n = len(recs)
    legacy = not any(r.final for r in recs)   # files from before the FINAL flag
    last = time.monotonic()
    try:
        for i in range(0, n, 64):
            batch = recs[i:i + 64]
            for r in batch:
                st.probe_lost += r.lost
            pipe.records_in(batch, last_is_final=legacy and i + 64 >= n)
            now = time.monotonic()
            if now - last >= 0.2:
                last = now
                serve.send_batch(st)
                conn.send(('progress', min(1.0, (i + 64) / max(1, n))))
                while conn.poll():
                    if serve.handle(conn.recv(), st) in ('abort', 'close'):
                        return
        st.bytes = len(stream)
        del recs, stream
        serve.send_batch(st)
        conn.send(('loaded', _stats(st, pipe)))
        # Stay for re-selections until the UI closes the session.
        while True:
            if serve.handle(conn.recv(), st) in ('abort', 'close', 'stop'):
                return
    except (EOFError, OSError, BrokenPipeError):
        return
    except Exception as e:
        try:
            conn.send(('error', 'decoder: %s' % e))
        except (EOFError, OSError, BrokenPipeError):
            pass
    finally:
        conn.close()
