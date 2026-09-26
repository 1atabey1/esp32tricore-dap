"""Trace sessions for the UI: a live capture from the probe, or a capture file.

Both decode the DTRP record stream and feed an Extractor.  Decoded events are
kept in compact numpy columns (EventLog), so the selection of signals can be
changed afterwards without decoding again.

Both run their data path in a worker process (worker.py): it receives the
stream and writes every frame to the capture file first (the file is always
complete), decodes - when it falls too far behind it skips ahead and marks a
gap in the live view only - and sends the samples here in batches.  The UI
process keeps the samples and polls the probe's drain counters.
"""

from __future__ import annotations

import base64
import http.client
import json
import queue
import struct
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from .capfile import REC_HEADER, REC_MAGIC, Record
from .decode import KIND_CODE, KINDS, DecodeStats, Decoder
from .signals import ExtractStats, Extractor, Signal, SignalStore

_KIND_CODE = KIND_CODE


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
        self.add_rows([(ev.cycles, _KIND_CODE.get(ev.kind, 255),
                        -1 if ev.addr is None else ev.addr,
                        -1 if ev.value is None else ev.value,
                        ev.size or 0, -1 if ev.wp is None else ev.wp,
                        -1 if ev.count is None else ev.count)])

    def add_rows(self, rows: list[tuple]) -> None:
        """Decoder rows (decode.ROW_FIELDS); fields after the seventh are not kept."""
        self._pending.extend(rows)
        self.n += len(rows)
        if len(self._pending) >= 65536:
            self._flush()

    def _flush(self) -> None:
        if not self._pending:
            return
        cols = list(zip(*self._pending))[:7]
        self._chunks.append({
            'cycles': np.array(cols[0], dtype=np.int64), 'kind': np.array(cols[1], dtype=np.uint8),
            'addr': np.array(cols[2], dtype=np.int64), 'value': np.array(cols[3], dtype=np.int64),
            'size': np.array(cols[4], dtype=np.uint8), 'wp': np.array(cols[5], dtype=np.int32),
            'count': np.array(cols[6], dtype=np.int32)})
        self._pending = []
        if self.limit:
            while len(self._chunks) > 1 and self.n - self.dropped > self.limit:
                self.dropped += len(self._chunks.pop(0)['kind'])

    def raw_rows(self, batch: int = 16384):
        """Every event again, as lists of (cycles, kind, addr, value, size, wp,
        count) tuples in order - what Extractor.feed_rows takes."""
        self._flush()
        for ch in self._chunks:
            cols = [ch[k].tolist() for k in ('cycles', 'kind', 'addr', 'value', 'size', 'wp',
                                             'count')]
            for i in range(0, len(cols[0]), batch):
                yield list(zip(*(c[i:i + batch] for c in cols)))

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

    def records_in(self, recs, forced_gap: bool = False, last_is_final: bool = False) -> None:
        with self.lock:
            self._records_in(recs, forced_gap, last_is_final)

    def _records_in(self, recs, forced_gap: bool, last_is_final: bool = False) -> None:
        rows: list[tuple] = []
        decode = self.decoder.rows
        for rec in recs:
            gap = forced_gap or rec.gap or (self.prev_seq is not None and rec.seq != self.prev_seq + 1)
            forced_gap = False
            self.prev_seq = rec.seq
            self.records += 1
            final = rec.final or (last_is_final and rec is recs[-1])
            rows += decode(rec.payload, rec.seq, gap, final)
        if rows:
            self.log.add_rows(rows)
            self.extractor.feed_rows(rows)

    def reselect(self, signals: list[Signal]) -> SignalStore:
        """Rebuild every series for a new selection from the event log."""
        with self.lock:
            store = SignalStore()
            # Same time zero even when the log has dropped its oldest events.
            ex = Extractor(signals, store, self.config.get('emu_hz') or 0, self.config,
                           t0=self.extractor.t0)
            for batch in self.log.raw_rows():
                ex.feed_rows(batch)
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
        except (ValueError, http.client.InvalidURL) as e:     # e.g. 'host:abc'
            raise ProbeError('%r is not a valid probe address (%s)' % (self.host, e)) from None

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


# -- workers ------------------------------------------------------------------

class _RemoteLog:
    """The worker's event log, as far as the UI asks about it."""

    def __init__(self, remote: '_Remote'):
        self._remote = remote
        self.n = 0
        self.dropped = 0

    def addresses(self) -> dict[tuple[int, int], int]:
        return self._remote.request(('addresses',), 'addresses')[1]


class _Remote:
    """The UI side of a worker (see worker.py): samples land in a local
    SignalStore; the decoder and extractor statistics mirror the worker's.
    The attribute names follow _Pipeline, which the worker runs."""

    def __init__(self, target, args: tuple, in_process: bool = False):
        import multiprocessing as mp
        ctx = mp.get_context('spawn')
        here, there = ctx.Pipe()
        if in_process:
            self.proc = threading.Thread(target=target, args=(there,) + args, daemon=True,
                                         name='trace-worker')
        else:
            self.proc = ctx.Process(target=target, args=(there,) + args, daemon=True,
                                    name='mcds-trace-worker')
        self.proc.start()
        if not in_process:
            there.close()                   # the child has its own copy
        self.conn = here
        self.store = SignalStore()
        self.decoder = SimpleNamespace(stats=DecodeStats())
        self.extractor = SimpleNamespace(stats=ExtractStats())
        self.log = _RemoteLog(self)
        self.records = 0
        self.error: str | None = None
        self.on_progress = None
        self.on_stats = None
        self.done = threading.Event()
        self._send_lock = threading.Lock()
        self._req_lock = threading.Lock()
        self._control: queue.Queue = queue.Queue()
        self._replies: queue.Queue = queue.Queue()
        self._rx = threading.Thread(target=self._recv, name='trace-results', daemon=True)
        self._rx.start()

    # -- messages ------------------------------------------------------------

    def _recv(self) -> None:
        try:
            while True:
                try:
                    msg = self.conn.recv()
                except (EOFError, OSError):
                    break
                kind = msg[0]
                if kind == 'batch':
                    self._apply(self.store, *msg[1:5])
                    self._stats(msg[5])
                elif kind == 'store':
                    store = SignalStore()
                    self._apply(store, *msg[1:5])
                    store.trimmed = self.store.trimmed
                    self.store = store
                    self._replies.put(('store', store))
                elif kind == 'addresses':
                    self._replies.put(msg)
                elif kind == 'progress':
                    if self.on_progress:
                        self.on_progress(msg[1])
                elif kind == 'done':            # the stream ended; requests still work
                    self._stats(msg[1])
                    self.error = self.error or msg[2]
                    self.done.set()
                else:                       # ready, config, loaded, error
                    if kind == 'error':
                        self.error = msg[1]
                    elif kind == 'loaded':
                        self._stats(msg[1])
                    self._control.put(msg)
        finally:
            self.done.set()
            self._control.put(('closed',))
            self._replies.put(('closed',))

    @staticmethod
    def _apply(store: SignalStore, series: dict, gaps: list, t_end: float, samples: int) -> None:
        with store.lock:
            for sid, (t, v) in series.items():
                store.get(sid).extend(t, v)
            store.gaps.extend(gaps)
            if t_end > store.t_end:
                store.t_end = t_end
            store.samples += samples

    def _stats(self, d: dict) -> None:
        self.decoder.stats = DecodeStats(**d['decode'])
        self.extractor.stats = ExtractStats(**d['extract'])
        self.records = d['records']
        self.log.n, self.log.dropped = d['log_n'], d['log_dropped']
        if self.on_stats:
            self.on_stats(d)

    def send(self, cmd: tuple) -> bool:
        with self._send_lock:
            try:
                self.conn.send(cmd)
                return True
            except (OSError, ValueError, BrokenPipeError):
                return False

    def wait_control(self, kinds: tuple, timeout: float | None = None) -> tuple:
        """The next control message of one of these kinds (or 'error'/'closed')."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            left = None if deadline is None else max(0.0, deadline - time.monotonic())
            try:
                msg = self._control.get(timeout=left)
            except queue.Empty:
                return ('error', 'the worker did not answer in %.0f s' % timeout)
            if msg[0] in kinds or msg[0] in ('error', 'closed'):
                return msg

    def request(self, cmd: tuple, reply: str, timeout: float = 600.0) -> tuple:
        with self._req_lock:
            if not self.send(cmd):
                raise ProbeError('the trace worker has ended')
            while True:
                try:
                    msg = self._replies.get(timeout=timeout)
                except queue.Empty:
                    raise ProbeError('the trace worker did not answer') from None
                if msg[0] == 'closed':
                    raise ProbeError(self.error or 'the trace worker has ended')
                if msg[0] == reply:
                    return msg

    def reselect(self, signals: list[Signal]) -> SignalStore:
        """Rebuild every series for a new selection from the worker's event log."""
        return self.request(('select', signals), 'store')[1]

    def close(self, timeout: float = 5.0) -> None:
        self.send(('close',))
        self.proc.join(timeout)
        if self.proc.is_alive() and hasattr(self.proc, 'terminate'):
            self.proc.terminate()
            self.proc.join(2)
        try:
            self.conn.close()
        except OSError:
            pass


# -- live ---------------------------------------------------------------------

class LiveSession:
    MAX_BACKLOG = 8 << 20          # bytes queued before the live view skips ahead
    LOG_LIMIT = 6_000_000          # events kept for re-selection (~35 bytes each)

    def __init__(self, probe: Probe, config: dict, signals: list[Signal], out_path: str,
                 in_process: bool = False):
        self.probe = probe
        self.request = config
        self.signals = signals
        self.out_path = out_path
        self.in_process = in_process
        self.stats = SessionStats()
        self._error: str | None = None
        self.config: dict = {}
        self.pipe: _Remote | None = None
        self.running = False
        self._poller: threading.Thread | None = None

    @property
    def error(self) -> str | None:
        return self._error or (self.pipe.error if self.pipe else None)

    @error.setter
    def error(self, value: str | None) -> None:
        self._error = value

    @property
    def store(self) -> SignalStore | None:
        return self.pipe.store if self.pipe else None

    def start(self) -> None:
        from . import worker
        applied = self.probe.post_config(self.request)
        reply = self.probe.start()
        if not reply.startswith('started'):
            raise ProbeError('the probe did not start the trace: %s' % reply)
        try:
            self.config = self.probe.config()
            self.config.setdefault('slots', applied.get('slots', []))
            self.pipe = _Remote(worker.live_main,
                                (self.probe.host, self.probe.auth, self.out_path, self.config,
                                 self.signals, self.LOG_LIMIT, self.MAX_BACKLOG),
                                self.in_process)
            self.pipe.on_stats = self._on_stats
            msg = self.pipe.wait_control(('ready',), timeout=30)
            if msg[0] != 'ready':
                raise ProbeError(msg[1] if len(msg) > 1 else 'the trace worker ended')
        except Exception:
            # Nothing records this trace: do not leave the probe running it.
            try:
                self.probe.stop()
            except ProbeError:
                pass
            if self.pipe is not None:
                self.pipe.close(timeout=2)
            raise
        self.stats.started = time.monotonic()
        self.running = True
        self._poller = threading.Thread(target=self._poll, name='trace-poll', daemon=True)
        self._poller.start()

    def _on_stats(self, d: dict) -> None:
        st = self.stats
        st.bytes, st.frames, st.records = d['bytes'], d['frames'], d['records']
        st.probe_lost, st.decode_skipped = d['probe_lost'], d['decode_skipped']
        st.backlog, st.rate = d['backlog'], d['rate']

    def _poll(self) -> None:
        while self.pipe is not None and not self.pipe.done.wait(1.0):
            try:
                cfg = self.probe.config()
                self.stats.probe = cfg.get('stats', {})
            except (ProbeError, ValueError):
                pass

    def stop(self, timeout: float = 90.0) -> None:
        """Stop the trace on the probe, collect the tail, close everything."""
        if not self.running:
            return
        try:
            self.probe.stop()
        except ProbeError as e:
            self._error = self._error or str(e)
        self.pipe.send(('stop',))
        if not self.pipe.done.wait(timeout):
            self._error = self._error or 'the trace worker did not finish'
        self.running = False
        # The worker stays for re-selections until close().

    def abort(self) -> None:
        """Close without collecting the tail (the app is exiting)."""
        if self.pipe is not None:
            self.pipe.send(('abort',))
            self.pipe.close(timeout=3)
        self.running = False

    def close(self, timeout: float = 90.0) -> None:
        """End the session and its worker (stopping the trace if it runs)."""
        if self.running:
            self.stop(timeout)
        if self.pipe is not None:
            self.pipe.close()


# -- offline ------------------------------------------------------------------

class FileSession:
    def __init__(self, path: str, in_process: bool = False):
        self.path = path
        self.in_process = in_process
        self.config: dict = {}
        self.pipe: _Remote | None = None
        self.stats = SessionStats()

    @property
    def store(self) -> SignalStore | None:
        return self.pipe.store if self.pipe else None

    def load(self, signals: list[Signal], progress=None) -> None:
        from . import worker
        self.pipe = _Remote(worker.file_main, (self.path, signals), self.in_process)
        self.pipe.on_progress = progress
        msg = self.pipe.wait_control(('config',), timeout=300)
        if msg[0] != 'config':
            self.close()
            raise RuntimeError(msg[1] if len(msg) > 1 else 'the trace worker ended')
        self.config = msg[1]
        msg = self.pipe.wait_control(('loaded',))
        if msg[0] != 'loaded':
            self.close()
            raise RuntimeError(msg[1] if len(msg) > 1 else 'the trace worker ended')
        st = msg[1]
        self.stats.records = st['records']
        self.stats.bytes = st['bytes']
        self.stats.probe_lost = st['probe_lost']

    def close(self) -> None:
        if self.pipe is not None:
            self.pipe.close()
