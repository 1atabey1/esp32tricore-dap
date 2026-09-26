"""Trace sessions for the UI: a live capture from the probe, or a capture file.

Both decode the DTRP record stream and feed an Extractor.  Decoded events are
kept in compact numpy columns (EventLog), so the selection of signals can be
changed afterwards without decoding again.

LiveSession runs three threads: the receiver writes every WebSocket frame to
the capture file first (the file is always complete) and queues it; the
decoder drains the queue - when it falls too far behind it skips ahead and
marks a gap in the live view only; a poller reads the probe's drain counters.
"""

from __future__ import annotations

import base64
import json
import queue
import struct
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field

import numpy as np

from . import capfile
from .capfile import REC_HEADER, REC_MAGIC, Record, write_header
from .decode import Decoder
from .signals import Extractor, Signal, SignalStore

KINDS = ('write', 'read', 'wps', 'wpm', 'evc', 'lost', 'gap')
_KIND_CODE = {k: i for i, k in enumerate(KINDS)}


class Row:
    """An event row, as the Extractor reads it (attribute access)."""
    __slots__ = ('cycles', 'kind', 'addr', 'value', 'size', 'wp', 'count')

    def __init__(self, cycles, kind, addr, value, size, wp, count):
        self.cycles = cycles
        self.kind = kind
        self.addr = addr
        self.value = value
        self.size = size
        self.wp = wp
        self.count = count


class EventLog:
    """Decoded events as numpy columns; appended in chunks."""

    def __init__(self, limit: int | None = None):
        self._chunks: list[dict] = []
        self._pending: list[tuple] = []
        self.n = 0
        self.limit = limit            # keep at most this many (oldest chunks go)
        self.dropped = 0

    def add(self, ev) -> None:
        self._pending.append((ev.cycles, _KIND_CODE.get(ev.kind, 255),
                              -1 if ev.addr is None else ev.addr,
                              -1 if ev.value is None else ev.value,
                              ev.size or 0, -1 if ev.wp is None else ev.wp,
                              -1 if ev.count is None else ev.count))
        self.n += 1
        if len(self._pending) >= 65536:
            self._flush()

    def _flush(self) -> None:
        if not self._pending:
            return
        cols = list(zip(*self._pending))
        self._chunks.append({
            'cycles': np.array(cols[0], dtype=np.int64), 'kind': np.array(cols[1], dtype=np.uint8),
            'addr': np.array(cols[2], dtype=np.int64), 'value': np.array(cols[3], dtype=np.int64),
            'size': np.array(cols[4], dtype=np.uint8), 'wp': np.array(cols[5], dtype=np.int32),
            'count': np.array(cols[6], dtype=np.int32)})
        self._pending = []
        if self.limit:
            while len(self._chunks) > 1 and self.n - self.dropped > self.limit:
                self.dropped += len(self._chunks.pop(0)['kind'])

    def rows(self):
        """Every event again, as Rows, in order."""
        self._flush()
        for ch in self._chunks:
            for c, k, a, v, s, w, n in zip(ch['cycles'].tolist(), ch['kind'].tolist(),
                                           ch['addr'].tolist(), ch['value'].tolist(),
                                           ch['size'].tolist(), ch['wp'].tolist(),
                                           ch['count'].tolist()):
                yield Row(c, KINDS[k] if k < len(KINDS) else '?', None if a < 0 else a,
                          None if v < 0 else v, s, None if w < 0 else w, None if n < 0 else n)

    def addresses(self) -> dict[tuple[int, int], int]:
        """Access count per (address, size) - writes and reads - for 'what did
        the trace see'."""
        self._flush()
        out: dict[tuple[int, int], int] = {}
        for ch in self._chunks:
            m = (ch['kind'] <= 1) & (ch['addr'] >= 0)
            key = ch['addr'][m] * 16 + np.maximum(ch['size'][m].astype(np.int64), 1)
            a, c = np.unique(key, return_counts=True)
            for x, y in zip(a.tolist(), c.tolist()):
                k = (x >> 4, x & 15)
                out[k] = out.get(k, 0) + y
        return out


class RecordStream:
    """Incremental DTRP record framing across arbitrary chunk boundaries."""

    _MAGIC = struct.pack('<I', REC_MAGIC)

    def __init__(self):
        self.buf = bytearray()
        self.resyncs = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        pos = 0
        buf = self.buf
        hs = REC_HEADER.size
        while len(buf) - pos >= hs:
            m, seq, off, length, flags, lost = REC_HEADER.unpack_from(buf, pos)
            if m != REC_MAGIC or length > 65535:
                nxt = buf.find(self._MAGIC, pos + 1)
                self.resyncs += 1
                if nxt < 0:
                    pos = max(pos, len(buf) - 3)
                    break
                pos = nxt
                continue
            if len(buf) - pos - hs < length:
                break
            start = pos + hs
            out.append(Record(seq, off, flags, lost, bytes(buf[start:start + length])))
            pos = start + length
        del self.buf[:pos]
        return out


@dataclass
class SessionStats:
    started: float = 0.0
    bytes: int = 0
    frames: int = 0
    records: int = 0
    probe_lost: int = 0           # paragraphs the probe reported lost
    decode_skipped: int = 0       # paragraphs skipped by the live decoder (file has them)
    backlog: int = 0              # bytes queued for decoding
    probe: dict = field(default_factory=dict)     # /api/mcds/config "stats"
    rate: float = 0.0             # received bytes/s, smoothed


class _Pipeline:
    """Records -> decoder -> event log + extractor (shared by both sessions)."""

    def __init__(self, config: dict, signals: list[Signal], log_limit: int | None = None):
        self.config = config
        self.tick_mode = config.get('timestamps') == 'ticks'
        self.decoder = Decoder(tick_mode=self.tick_mode)
        self.log = EventLog(log_limit)
        self.store = SignalStore()
        self.extractor = Extractor(signals, self.store, config.get('emu_hz') or 0, config)
        self.prev_seq: int | None = None
        self.records = 0
        self.lock = threading.Lock()     # records_in vs reselect

    def records_in(self, recs, forced_gap: bool = False) -> None:
        with self.lock:
            self._records_in(recs, forced_gap)

    def _records_in(self, recs, forced_gap: bool) -> None:
        rows = []
        for rec in recs:
            gap = forced_gap or rec.gap or (self.prev_seq is not None and rec.seq != self.prev_seq + 1)
            forced_gap = False
            self.prev_seq = rec.seq
            self.records += 1
            for ev in self.decoder.paragraph(rec.payload, rec.seq, gap=gap):
                self.log.add(ev)
                rows.append(ev)
        if rows:
            self.extractor.feed(rows)

    def reselect(self, signals: list[Signal]) -> SignalStore:
        """Rebuild every series for a new selection from the event log."""
        with self.lock:
            store = SignalStore()
            # Same time zero even when the log has dropped its oldest events.
            ex = Extractor(signals, store, self.config.get('emu_hz') or 0, self.config,
                           t0=self.extractor.t0)
            batch = []
            for row in self.log.rows():
                batch.append(row)
                if len(batch) >= 8192:
                    ex.feed(batch)
                    batch = []
            ex.feed(batch)
            store.gaps = list(self.store.gaps)
            if self.store.t_end > store.t_end:
                store.t_end = self.store.t_end
            self.store, self.extractor = store, ex
            return store


# -- probe HTTP ---------------------------------------------------------------

class ProbeError(RuntimeError):
    pass


class Probe:
    def __init__(self, host: str, auth: str = 'admin:admin', timeout: float = 10.0):
        self.host = host.strip()
        self.auth = auth
        self.timeout = timeout

    def _req(self, path: str, data: bytes | None = None, timeout: float | None = None) -> bytes:
        req = urllib.request.Request('http://%s%s' % (self.host, path), data=data,
                                     method='POST' if data is not None else 'GET')
        req.add_header('Authorization', 'Basic ' + base64.b64encode(self.auth.encode()).decode())
        if data is not None:
            req.add_header('Content-Type', 'application/json')
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors='replace').strip()
            raise ProbeError('%s: HTTP %d %s' % (path, e.code, body or e.reason)) from None
        except (urllib.error.URLError, OSError) as e:
            raise ProbeError('%s: %s' % (self.host, getattr(e, 'reason', e))) from None

    def status(self) -> str:
        return self._req('/api/dap_gdb/status', timeout=4).decode(errors='replace').strip()

    def config(self) -> dict:
        return json.loads(self._req('/api/mcds/config'))

    def post_config(self, cfg: dict) -> dict:
        return json.loads(self._req('/api/mcds/config', json.dumps(cfg).encode()))

    def start(self) -> str:
        return self._req('/api/mcds/start', timeout=60).decode(errors='replace').strip()

    def stop(self) -> str:
        return self._req('/api/mcds/stop', timeout=60).decode(errors='replace').strip()


# -- live ---------------------------------------------------------------------

class LiveSession:
    MAX_BACKLOG = 8 << 20          # bytes queued before the live view skips ahead

    def __init__(self, probe: Probe, config: dict, signals: list[Signal], out_path: str):
        self.probe = probe
        self.request = config
        self.signals = signals
        self.out_path = out_path
        self.stats = SessionStats()
        self.error: str | None = None
        self.config: dict = {}
        self.pipe: _Pipeline | None = None
        self._q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._rx_done = threading.Event()
        self._threads: list[threading.Thread] = []
        self._ws = None
        self._file = None
        self.running = False
        self.lock = threading.Lock()

    @property
    def store(self) -> SignalStore | None:
        return self.pipe.store if self.pipe else None

    def start(self) -> None:
        import websocket
        applied = self.probe.post_config(self.request)
        reply = self.probe.start()
        if not reply.startswith('started'):
            raise ProbeError('the probe did not start the trace: %s' % reply)
        self.config = self.probe.config()
        self.config.setdefault('slots', applied.get('slots', []))
        self._file = open(self.out_path, 'wb')
        write_header(self._file, self.config)
        # ~35 bytes per event: 6 M events keep a re-selection cheap to hold.
        self.pipe = _Pipeline(self.config, self.signals, log_limit=6_000_000)
        hdr = {'Authorization': 'Basic ' + base64.b64encode(self.probe.auth.encode()).decode()}
        try:
            self._ws = websocket.create_connection('ws://%s/ws/trace' % self.probe.host,
                                                   header=hdr, timeout=1)
        except Exception as e:
            self._file.close()
            try:
                self.probe.stop()
            except ProbeError:
                pass
            raise ProbeError('trace stream: %s' % e) from None
        self.stats.started = time.monotonic()
        self.running = True
        for fn, name in ((self._rx, 'trace-rx'), (self._decode, 'trace-decode'),
                         (self._poll, 'trace-poll')):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def _rx(self) -> None:
        import websocket
        st = self.stats
        idle_since = None
        last = time.monotonic()
        window = 0
        try:
            while True:
                if self._stop.is_set():
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since > 1.5:
                        break               # the stream has gone quiet after stop
                try:
                    frame = self._ws.recv()
                except websocket.WebSocketTimeoutException:
                    frame = None
                except (websocket.WebSocketConnectionClosedException, OSError):
                    if not self._stop.is_set():
                        self.error = 'the probe closed the trace stream'
                    break
                now = time.monotonic()
                if frame and not isinstance(frame, str):
                    self._file.write(frame)
                    st.bytes += len(frame)
                    st.frames += 1
                    window += len(frame)
                    st.backlog += len(frame)
                    self._q.put(frame)
                    idle_since = None if not self._stop.is_set() else now
                if now - last >= 0.5:
                    st.rate = 0.6 * st.rate + 0.4 * window / (now - last)
                    window = 0
                    last = now
        except Exception as e:                           # keep the file consistent
            self.error = 'receiver: %s' % e
        finally:
            try:
                self._file.flush()
            except (OSError, ValueError):
                pass
            self._rx_done.set()
            self._q.put(None)

    def _decode(self) -> None:
        rs = RecordStream()
        st = self.stats
        pipe = self.pipe
        skip_gap = False
        try:
            while True:
                item = self._q.get()
                if item is None:
                    break
                st.backlog -= len(item)
                if st.backlog > self.MAX_BACKLOG:
                    # Behind: drop what is queued from the live view (the file
                    # has it) and restart the decoder at the next record.
                    dropped = 0
                    while True:
                        try:
                            nxt = self._q.get_nowait()
                        except queue.Empty:
                            break
                        if nxt is None:
                            self._q.put(None)
                            break
                        st.backlog -= len(nxt)
                        dropped += len(nxt)
                    st.decode_skipped += dropped // 1048
                    rs = RecordStream()
                    skip_gap = True
                    continue
                recs = rs.feed(item)
                if recs:
                    for r in recs:
                        st.probe_lost += r.lost
                    pipe.records_in(recs, forced_gap=skip_gap)
                    skip_gap = False
                    st.records = pipe.records
        except Exception as e:
            self.error = 'decoder: %s' % e

    def _poll(self) -> None:
        while not self._rx_done.wait(1.0):
            try:
                cfg = self.probe.config()
                self.stats.probe = cfg.get('stats', {})
            except (ProbeError, ValueError):
                pass

    def stop(self, timeout: float = 90.0) -> None:
        """Stop the trace on the probe, collect the tail, close everything."""
        if not self.running:
            return
        self._stop.set()
        try:
            self.probe.stop()
        except ProbeError as e:
            self.error = self.error or str(e)
        self._rx_done.wait(timeout)
        for t in self._threads:
            t.join(timeout=10)
        try:
            self._ws.close()
        except Exception:
            pass
        try:
            self._file.close()
        except (OSError, ValueError):
            pass
        self.running = False

    def abort(self) -> None:
        """Close without collecting the tail (the app is exiting)."""
        self._stop.set()
        try:
            self._ws.close()
        except Exception:
            pass


# -- offline ------------------------------------------------------------------

class FileSession:
    def __init__(self, path: str):
        self.path = path
        self.config: dict = {}
        self.pipe: _Pipeline | None = None
        self.stats = SessionStats()

    @property
    def store(self) -> SignalStore | None:
        return self.pipe.store if self.pipe else None

    def load(self, signals: list[Signal], progress=None) -> None:
        config, stream = capfile.read_file(self.path)
        self.config = config
        self.pipe = _Pipeline(config, signals)
        recs = list(capfile.records(stream))
        n = len(recs)
        for i in range(0, n, 64):
            batch = recs[i:i + 64]
            for r in batch:
                self.stats.probe_lost += r.lost
            self.pipe.records_in(batch)
            if progress:
                progress(min(1.0, (i + 64) / max(1, n)))
        self.stats.records = n
        self.stats.bytes = len(stream)
