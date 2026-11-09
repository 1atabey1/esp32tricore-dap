"""The tricore-debug-machine probe ("tdm") as a trace source, over USB.

The tdm probe runs the same trace firmware (dap_mcds.c, dap_trace.c) as the
esp32jtag, behind its vendor interface (USB interface 3, WinUSB on Windows
without a driver install) instead of HTTP and a WebSocket:

    EP 0x04 / 0x84   commands: u16 op | u16 len | u32 tag | payload, answered
                     with the same header and i32 status (esp_err_t) | data
    EP 0x85          the trace: the DTRP record stream, as /ws/trace sends it
                     (uncompressed)

`TdmProbe` speaks that protocol and offers what session.Probe offers for the
esp32jtag (status, config, post_config, start, stop), with the configuration
in the web page's JSON form, so sessions, capture files and the UI do not
care which probe they use.  The tricore-debug-machine repository's
Appli/usb/vendor_dap.h is the protocol's reference, sw/tools/tdm_dap.py its
command line.

A USB interface belongs to one process at a time: the trace worker owns the
probe for a live session (stream and commands); `TdmSource` is the UI side,
which passes its commands to the worker while it holds the probe and opens
the probe itself otherwise.

Fast mode (a DAP bit every FPGA fabric clock) uses the probe's clock
handshake: TRACE_START with the auto flag tries fast mode on each FPGA bin
from the fastest down and keeps the first whose drain holds up (steps down
later if it starts failing); config() reports the bin (`fast_mhz`).
Without fast mode the trace runs on the 48 MHz bin, where the DAP divider
means what it means on the esp32jtag (0: 24 MHz).  Firmware without the
handshake ignores the flag and traces on the bin `prepare` loads (48 MHz).
"""

from __future__ import annotations

import struct
import threading
import time

VID, PID = 0x1D50, 0x606F
IFACE = 3
EP_OUT, EP_IN, EP_TRACE = 0x04, 0x84, 0x85
MAX_PAYLOAD = 8192
TRACE_READ = 64 * 1024             # a multiple of 512; the probe ends transfers short
TRACE_BIN_MHZ = 48                 # see the module docstring

OP = dict(INFO=0x0001, LOG=0x0002, POWER24=0x0010, FPGA_LOAD=0x0013, ATTACH=0x0020,
          DETACH=0x0021, SESSION=0x0025, TRACE_CFG=0x0040, TRACE_START=0x0041,
          TRACE_STOP=0x0042, TRACE_STATS=0x0043)

ESP_ERR = {-1: 'FAIL', 0x101: 'NO_MEM', 0x102: 'INVALID_ARG', 0x103: 'INVALID_STATE',
           0x104: 'INVALID_SIZE', 0x105: 'NOT_FOUND', 0x106: 'NOT_SUPPORTED', 0x107: 'TIMEOUT',
           0x108: 'INVALID_RESPONSE', 0x109: 'INVALID_CRC', 0x10A: 'INVALID_VERSION'}

# dap_mcds_config_t / dap_mcds_info_t / dap_trace_stats_t as arm-none-eabi
# lays them out (short enums); the probe reports the config's size.
_CFG_HEAD = struct.Struct('<9B3x')            # source cpu mode payload timestamps masters div wide fast
_CFG_SLOT = struct.Struct('<B24s3xIIBBBxIIIB3x')
_CFG_PROG = struct.Struct('<BBHHBxIIIIHH')    # mode window cft_vshort cft_short exclude lo hi
                                              # xlo xhi duty_on_us duty_period_us
CFG_SIZE = _CFG_HEAD.size + 2 * _CFG_SLOT.size + _CFG_PROG.size     # 152
PROG_MODES = ('off', 'flow', 'calls')
_INFO_HEAD = struct.Struct('<IIBB2x')
_SNAP_HEAD = struct.Struct('<IH2x')
SNAPSHOT_BYTES = 2048
INFO_SIZE = _INFO_HEAD.size + 2 * (_SNAP_HEAD.size + SNAPSHOT_BYTES)      # 4124
_STATS = struct.Struct('<B3x14I')
STATS_FIELDS = ('paragraphs', 'lost', 'laps', 'overruns', 'read_errors', 'bytes', 'fifonow',
                'queue_free', 'queue_dropped', 'poll_us_max', 'read_us', 'read_paragraphs',
                'passes', 'queue_size')
_SESSION = struct.Struct('<BBBBHBBII')       # attached wide fast cores bin_mhz div - khz asked

SOURCES = ('cpu', 'memslave', 'lmu0')
MODES = ('full', 'compact')
PAYLOADS = ('addr_data', 'data', 'addr')
TIMESTAMPS = ('hit', 'ticks', 'none')


class TdmError(RuntimeError):
    pass


def is_tdm(host: str) -> bool:
    """'tdm', 'usb' or 'tdm:SERIAL' in the probe field select this probe."""
    h = (host or '').strip().lower()
    return h in ('tdm', 'usb') or h.startswith('tdm:')


def serial_of(host: str) -> str | None:
    h = (host or '').strip()
    return h[4:] or None if h.lower().startswith('tdm:') else None


# -- configuration: the web page's JSON <-> dap_mcds_config_t ------------------

def _num(v, default: int) -> int:
    if isinstance(v, bool) or v is None:
        return default
    if isinstance(v, (int, float)):
        return int(v) & 0xFFFFFFFF
    try:
        return int(str(v), 0) & 0xFFFFFFFF if str(v) else default
    except ValueError:
        return default


def _pick(v, names: tuple, default: int = 0) -> int:
    return names.index(v) if v in names else default


def pack_config(cfg: dict) -> bytes:
    """The JSON configuration as dap_mcds_config_t (mcds_web.c config_from_json:
    the same defaults and checks)."""
    head = _CFG_HEAD.pack(_pick(cfg.get('source'), SOURCES), _num(cfg.get('cpu'), 0) & 0xFF,
                          _pick(cfg.get('mode'), MODES), _pick(cfg.get('payload'), PAYLOADS),
                          _pick(cfg.get('timestamps'), TIMESTAMPS), bool(cfg.get('masters', False)),
                          _num(cfg.get('dap_div'), 0) & 0xFF, bool(cfg.get('wide', True)),
                          bool(cfg.get('fast', True)))
    slots = list(cfg.get('slots') or [])
    out, enabled = [head], 0
    for j in range(2):
        s = slots[j] if j < len(slots) and isinstance(slots[j], dict) else None
        if s is None:
            out.append(bytes(_CFG_SLOT.size))
            continue
        on = bool(s.get('enabled', True))
        acc = s.get('access') if isinstance(s.get('access'), str) else 'w'
        size = _num(s.get('size'), 4)
        v = s.get('value') if isinstance(s.get('value'), dict) else {}
        if on:
            if size == 0 or not ('r' in acc or 'w' in acc):
                raise TdmError('slot %d: size 0 or no access selected' % j)
            enabled += 1
        name = str(s.get('name') or '').encode()[:23]
        out.append(_CFG_SLOT.pack(on, name, _num(s.get('addr'), 0), size, 'r' in acc, 'w' in acc,
                                  bool(v.get('enabled', False)), _num(v.get('lo'), 0),
                                  _num(v.get('hi'), 0xFFFFFFFF), _num(v.get('mask'), 0xFFFFFFFF),
                                  bool(v.get('signed', False))))
    pg = cfg.get('program') if isinstance(cfg.get('program'), dict) else {}
    pmode = _pick(pg.get('mode'), PROG_MODES)
    win = pg.get('window') if isinstance(pg.get('window'), (list, tuple)) else None
    exc = pg.get('exclude') if isinstance(pg.get('exclude'), (list, tuple)) else None
    out.append(_CFG_PROG.pack(pmode, win is not None, _num(pg.get('cft_vshort'), 0) & 0x3FF,
                              _num(pg.get('cft_short'), 0) & 0x3FF, exc is not None,
                              _num(win[0], 0) if win else 0, _num(win[1], 0) if win else 0,
                              _num(exc[0], 0) if exc else 0, _num(exc[1], 0) if exc else 0,
                              _num(pg.get('duty_on_us'), 0) & 0xFFFF,
                              _num(pg.get('duty_period_us'), 0) & 0xFFFF))
    if not enabled and not pmode:
        raise TdmError('no enabled slot and no program trace')
    return b''.join(out)


def unpack_config(raw: bytes) -> dict:
    """dap_mcds_config_t as the JSON the esp32jtag returns (config_to_json)."""
    src, cpu, mode, payload, ts, masters, div, wide, fast = _CFG_HEAD.unpack_from(raw)
    cfg = {'source': SOURCES[src % 3], 'cpu': cpu, 'mode': MODES[mode % 2],
           'payload': PAYLOADS[payload % 3], 'timestamps': TIMESTAMPS[ts % 3],
           'masters': bool(masters), 'dap_div': div, 'wide': bool(wide), 'fast': bool(fast),
           'slots': []}
    for j in range(2):
        (on, name, addr, size, rd, wr, ven, lo, hi, mask,
         sgn) = _CFG_SLOT.unpack_from(raw, _CFG_HEAD.size + j * _CFG_SLOT.size)
        cfg['slots'].append({
            'enabled': bool(on), 'name': name.split(b'\0')[0].decode(errors='replace'),
            'addr': '0x%08X' % addr, 'size': size,
            'access': 'rw' if rd and wr else 'r' if rd else 'w',
            'value': {'enabled': bool(ven), 'lo': '0x%08X' % lo, 'hi': '0x%08X' % hi,
                      'mask': '0x%08X' % mask, 'signed': bool(sgn)}})
    if len(raw) >= CFG_SIZE:
        (pmode, win, vs, sh, exc, lo, hi, xlo, xhi, don,
         dper) = _CFG_PROG.unpack_from(raw, CFG_SIZE - _CFG_PROG.size)
        cfg['program'] = {'mode': PROG_MODES[pmode % 3], 'cft_vshort': vs, 'cft_short': sh,
                          'window': ['0x%08X' % lo, '0x%08X' % hi] if win else None,
                          'exclude': ['0x%08X' % xlo, '0x%08X' % xhi] if exc else None,
                          'duty_on_us': don, 'duty_period_us': dper}
    return cfg


def unpack_info(raw: bytes) -> dict:
    """dap_mcds_info_t: the clock, link and snapshot the start produced."""
    emu_hz, tsu, wide, fast = _INFO_HEAD.unpack_from(raw)
    snaps, off = [], _INFO_HEAD.size
    for j in range(2):
        addr, n = _SNAP_HEAD.unpack_from(raw, off)
        data = raw[off + _SNAP_HEAD.size:off + _SNAP_HEAD.size + min(n, SNAPSHOT_BYTES)]
        if n and len(data) == n:
            snaps.append({'slot': j, 'addr': '0x%08X' % addr, 'hex': data.hex()})
        off += _SNAP_HEAD.size + SNAPSHOT_BYTES
    return {'emu_hz': emu_hz, 'tsu_start': tsu, 'wide_active': bool(wide),
            'fast_active': bool(fast), 'snapshot': snaps}


def unpack_stats(raw: bytes) -> dict:
    v = _STATS.unpack_from(raw)
    d = dict(zip(STATS_FIELDS, v[1:]))
    d['running'] = bool(v[0])
    d['dual'] = False
    return d


# -- the probe -----------------------------------------------------------------

def _backend():
    """libusb-1.0 from the system, else from the libusb-package wheel."""
    import usb.backend.libusb1
    b = usb.backend.libusb1.get_backend()
    if b is not None:
        return b
    try:
        import libusb_package
    except ImportError:
        return None
    return libusb_package.get_libusb1_backend()


class TdmProbe:
    """One open tdm probe (USB interface 3).  Commands may come from any
    thread while another one reads the trace endpoint."""

    def __init__(self, serial: str | None = None):
        try:
            import usb.core
            import usb.util
        except ImportError:
            raise TdmError('pyusb is not installed (uv sync)') from None
        self._usb = usb
        try:
            devs = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID,
                                      backend=_backend()))
        except usb.core.NoBackendError:
            raise TdmError('no libusb-1.0 (install libusb-package)') from None
        if serial:
            devs = [d for d in devs if _serial(d) == serial]
        if not devs:
            raise TdmError('no tdm probe%s on USB' % (' ' + serial if serial else ''))
        self.dev = devs[0]
        self.serial = _serial(self.dev) or '?'
        try:
            if self.dev.is_kernel_driver_active(IFACE):
                self.dev.detach_kernel_driver(IFACE)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            usb.util.claim_interface(self.dev, IFACE)
        except usb.core.USBError as e:
            raise TdmError('tdm probe %s is busy (%s): another program or trace holds it'
                           % (self.serial, e)) from None
        self._tag = 1
        self._cmd_lock = threading.Lock()
        self._buf = usb.util.create_buffer(TRACE_READ)
        self.cfg: dict | None = None
        self.info: dict = {}

    def close(self) -> None:
        try:
            self._usb.util.release_interface(self.dev, IFACE)
        except Exception:
            pass
        try:
            self._usb.util.dispose_resources(self.dev)
        except Exception:
            pass

    def __enter__(self) -> 'TdmProbe':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- commands -------------------------------------------------------------

    def cmd(self, name: str, payload: bytes = b'', timeout: int = 10000) -> tuple[int, bytes]:
        op = OP[name]
        usb = self._usb
        with self._cmd_lock:
            self._tag = (self._tag + 1) & 0xFFFFFFFF
            tag = self._tag
            try:
                self.dev.write(EP_OUT, struct.pack('<HHI', op, len(payload), tag) + payload,
                               timeout=timeout)
                buf = bytes(self.dev.read(EP_IN, 8 + MAX_PAYLOAD + 512, timeout=timeout))
                rop, ln, rtag = struct.unpack_from('<HHI', buf)
                if ln > MAX_PAYLOAD:
                    raise TdmError('response length %d is more than any op returns' % ln)
                while len(buf) < 8 + ln:
                    buf += bytes(self.dev.read(EP_IN, 8 + MAX_PAYLOAD + 512, timeout=timeout))
            except usb.core.USBError as e:
                raise TdmError('%s: %s' % (name, e)) from None
        if rtag != tag:
            raise TdmError('%s: response tag %d, expected %d' % (name, rtag, tag))
        if rop == op | 0x8000:
            raise TdmError('the probe does not know %s (firmware too old?)' % name)
        body = buf[8:8 + ln]
        if len(body) < 4:
            raise TdmError('%s: response without a status' % name)
        return struct.unpack_from('<i', body)[0], body[4:]

    def ok(self, name: str, payload: bytes = b'', **kw) -> bytes:
        status, data = self.cmd(name, payload, **kw)
        if status != 0:
            raise TdmError('%s: %s' % (name, ESP_ERR.get(status, hex(status))))
        return data

    def session(self) -> dict:
        att, wide, fast, cores, mhz, div, _, khz, _ = _SESSION.unpack_from(self.ok('SESSION'))
        return {'attached': bool(att), 'wide': bool(wide), 'fast': bool(fast), 'cores': cores,
                'bin_mhz': mhz, 'div': div, 'khz': khz}

    def power24(self) -> str:
        status, d = self.cmd('POWER24', bytes([2]))
        if status != 0 or len(d) < 4:
            return '?'
        return ('off', 'starting', 'on', 'fault')[d[0]] if d[0] < 4 else '?'

    # -- what session.Probe offers ------------------------------------------------

    def status(self) -> str:
        s = self.session()
        return 'tdm %s %s cores=%d bin=%dMHz 24V=%s' % (
            self.serial, 'attached' if s['attached'] else 'detached', s['cores'], s['bin_mhz'],
            self.power24())

    def _config_raw(self) -> bytes:
        d = self.ok('TRACE_CFG')
        (size,) = struct.unpack_from('<I', d)
        if size != CFG_SIZE:
            raise TdmError('the probe\'s trace configuration is %d bytes, this tool knows %d'
                           % (size, CFG_SIZE))
        return d[4:4 + size]

    def post_config(self, cfg: dict) -> dict:
        raw = pack_config(cfg)
        self.ok('TRACE_CFG', raw)
        self.cfg = unpack_config(raw)
        return dict(self.cfg)

    def config(self) -> dict:
        """As GET /api/mcds/config: the configuration, the start's clock, link
        and snapshot, and the drain counters."""
        if self.cfg is None:
            self.cfg = unpack_config(self._config_raw())
        out = dict(self.cfg)
        stats = unpack_stats(self.ok('TRACE_STATS'))
        out.update({'running': stats.pop('running'), 'emu_hz': 0, 'tsu_start': 0,
                    'wide_active': False, 'fast_active': False, 'format': 1, 'snapshot': []})
        out.update(self.info)
        out['stats'] = stats
        out['probe'] = 'tdm'
        return out

    def prepare(self, bin_mhz: int = TRACE_BIN_MHZ) -> dict:
        """The trace bin loaded and the target attached (with fast mode the
        probe's handshake picks the bin itself, starting from this one)."""
        s = self.session()
        if bin_mhz and s['bin_mhz'] != bin_mhz:
            if s['attached']:
                self.cmd('DETACH')
            status, _ = self.cmd('FPGA_LOAD', struct.pack('<H', bin_mhz), timeout=20000)
            if status != 0:
                raise TdmError('loading the %d MHz FPGA bin failed (%s)'
                               % (bin_mhz, ESP_ERR.get(status, status)))
            s = self.session()
        if not s['attached']:
            status, _ = self.cmd('ATTACH', timeout=30000)
            if status != 0:
                raise TdmError('attaching the target failed (%s); is it powered (24 V %s)?'
                               % (ESP_ERR.get(status, status), self.power24()))
            s = self.session()
        return s

    def start(self) -> str:
        if self.cfg is None:
            self.cfg = unpack_config(self._config_raw())
        fast = bool(self.cfg.get('fast'))
        try:
            self.prepare()
        except TdmError as e:
            return 'failed: %s' % e
        # auto: the probe picks the bin (it tries them, so this takes up to ~1 s)
        status, d = self.cmd('TRACE_START', b'\x01' if fast else b'', timeout=60000)
        if status != 0:
            return 'failed: TRACE_START %s' % ESP_ERR.get(status, hex(status))
        if len(d) < INFO_SIZE:
            return 'failed: TRACE_START answered %d bytes, expected %d' % (len(d), INFO_SIZE)
        self.info = unpack_info(d)
        if len(d) >= INFO_SIZE + 3:
            bin_mhz, attempts = struct.unpack_from('<HB', d, INFO_SIZE)
            self.info['bin_mhz'] = bin_mhz
            self.info['attempts'] = attempts
            if self.info['fast_active']:
                self.info['fast_mhz'] = bin_mhz
        elif self.info['fast_active']:
            self.info['fast_mhz'] = TRACE_BIN_MHZ
        return 'started emu_hz=%d wide=%d fast=%d' % (self.info['emu_hz'],
                                                      self.info['wide_active'],
                                                      self.info['fast_active'])

    def stop(self) -> str:
        status, _ = self.cmd('TRACE_STOP', timeout=30000)
        st = unpack_stats(self.ok('TRACE_STATS'))
        return '%s paragraphs=%d lost=%d queue_dropped=%d' % (
            'stopped' if status == 0 else 'stop: %s' % ESP_ERR.get(status, hex(status)),
            st['paragraphs'], st['lost'], st['queue_dropped'])

    # -- the trace endpoint -------------------------------------------------------

    def read(self, timeout_ms: int = 200) -> bytes | None:
        """One transfer of the DTRP stream, or None after timeout_ms quiet."""
        usb = self._usb
        try:
            n = self.dev.read(EP_TRACE, self._buf, timeout=timeout_ms)
        except usb.core.USBTimeoutError:
            return None
        except usb.core.USBError as e:
            if getattr(e, 'errno', None) in (110, 10060) or 'timed out' in str(e).lower():
                return None
            raise TdmError('trace endpoint: %s' % e) from None
        return bytes(self._buf[:n])

    def drain(self, quiet_s: float = 0.3, limit_s: float = 5.0) -> int:
        """Drop what an earlier trace left on the endpoint."""
        n, t0, quiet = 0, time.monotonic(), time.monotonic()
        while time.monotonic() - quiet < quiet_s and time.monotonic() - t0 < limit_s:
            b = self.read(50)
            if b:
                n += len(b)
                quiet = time.monotonic()
        return n


def _serial(dev) -> str | None:
    try:
        return dev.serial_number
    except Exception:
        return None


class TdmSource:
    """The UI's handle on a tdm probe (session.Probe's methods).  While a
    trace worker holds the probe (bind), commands go through the worker;
    otherwise each call opens the probe for itself."""

    def __init__(self, host: str = 'tdm'):
        self.host = host.strip() or 'tdm'
        self.serial = serial_of(self.host)
        self.auth = ''
        self._remote = None

    def stream_spec(self) -> tuple:
        return ('tdm', self.serial)

    def bind(self, remote) -> None:
        self._remote = remote

    def _call(self, method: str, *args):
        from .session import ProbeError
        r = self._remote
        if r is not None and not r.done.is_set():
            return r.request(('ctl', method, args), 'ctl', timeout=90)[1]
        try:
            with TdmProbe(self.serial) as p:
                return getattr(p, method)(*args)
        except TdmError as e:
            raise ProbeError(str(e)) from None

    def status(self) -> str:
        return self._call('status')

    def config(self) -> dict:
        return self._call('config')

    def post_config(self, cfg: dict) -> dict:
        return self._call('post_config', cfg)

    def start(self) -> str:
        return self._call('start')

    def stop(self) -> str:
        return self._call('stop')
