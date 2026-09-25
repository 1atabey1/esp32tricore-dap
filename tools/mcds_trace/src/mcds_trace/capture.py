"""Capture a running trace from the probe's /ws/trace into a capture file."""

from __future__ import annotations

import base64
import json
import signal
import time
import urllib.request

import websocket

from .capfile import write_header


def _get(host: str, path: str, auth: str, timeout: float = 30) -> str:
    req = urllib.request.Request('http://%s%s' % (host, path))
    req.add_header('Authorization', 'Basic ' + base64.b64encode(auth.encode()).decode())
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read().decode(errors='replace')


def trace_config(host: str, auth: str) -> dict:
    """The configuration the probe applied, as JSON (GET /api/mcds/config)."""
    try:
        return json.loads(_get(host, '/api/mcds/config', auth))
    except Exception:
        return {}


def capture(host: str, out_path: str, auth: str = 'admin:admin',
            seconds: float | None = None, start: bool = False) -> dict:
    """Stream /ws/trace into out_path until Ctrl-C or `seconds` elapse.

    With start=True the trace is started first (GET /api/mcds/start) and
    stopped afterwards.  Returns simple counters.
    """
    if start:
        _get(host, '/api/mcds/start', auth)
    config = trace_config(host, auth)
    hdr = {'Authorization': 'Basic ' + base64.b64encode(auth.encode()).decode()}
    ws = websocket.create_connection('ws://%s/ws/trace' % host, header=hdr, timeout=1)

    stop = {'flag': False}
    old = signal.signal(signal.SIGINT, lambda *a: stop.update(flag=True))
    t0 = time.monotonic()
    count = {'bytes': 0, 'frames': 0}

    def pump(f, idle_limit: float | None) -> None:
        """Receive until stopped, or until idle for idle_limit seconds."""
        last = time.monotonic()
        while True:
            if idle_limit is None:
                if stop['flag'] or (seconds is not None and time.monotonic() - t0 >= seconds):
                    return
            elif time.monotonic() - last >= idle_limit:
                return
            try:
                frame = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except websocket.WebSocketConnectionClosedException:
                return
            if isinstance(frame, str):
                continue
            f.write(frame)
            count['bytes'] += len(frame)
            count['frames'] += 1
            last = time.monotonic()

    try:
        with open(out_path, 'wb') as f:
            write_header(f, config)
            pump(f, None)
            if start:
                # Stop first: the probe flushes the TRAM and the ring (up to
                # 1 MB) is still in flight; read until the stream goes quiet.
                _get(host, '/api/mcds/stop', auth)
                pump(f, 1.5)
    finally:
        signal.signal(signal.SIGINT, old)
        ws.close()
    return {'bytes': count['bytes'], 'frames': count['frames'],
            'seconds': time.monotonic() - t0}
