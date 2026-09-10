#!/usr/bin/env python3
"""Decode a usbmon capture of an FT2232 probe into the bits it put on the wire.

The miniWiggler V3.1 is an FT2232 with an Infineon VID, so its USB traffic is
FTDI MPSSE rather than anything DAP-shaped: opcodes that say "clock these bytes
out on the falling edge, LSB first", "clock this many bits in", "write these
GPIO levels and directions".  Reconstructing that gives the exact bit sequence
a working probe drives, which is the reference our own framing needs.

Usage:
    decode_mpsse.py capture.usbmon [--dev 004] [--limit N] [--bits]

usbmon's text format, one event per line:

    URB-ID  Timestamp  Event  Bt:Bus:Dev:Ep  Status  Length  =  data words

Event 'S' is a submit and 'C' a completion, so host-to-probe payloads appear on
'S' lines of the OUT endpoint and probe-to-host payloads on 'C' lines of the IN
endpoint.
"""

from __future__ import annotations

import argparse
import re
import sys

#: `ffff...  3722913854 S Bo:1:004:2 -115 6 = 80080b 82000d`
LINE_RE = re.compile(
    r"^(?P<urb>\S+)\s+(?P<ts>\d+)\s+(?P<ev>[SCE])\s+"
    r"(?P<type>[BCIZ])(?P<dir>[io]):(?P<bus>\d+):(?P<dev>\d+):(?P<ep>\d+)\s+"
    r"(?P<rest>.*)$"
)

# MPSSE opcodes that are not data-shifting commands, with their argument counts.
STATIC_OPS = {
    0x80: ("set_bits_low",  2),
    0x82: ("set_bits_high", 2),
    0x81: ("read_bits_low", 0),
    0x83: ("read_bits_high", 0),
    0x84: ("loopback_on",   0),
    0x85: ("loopback_off",  0),
    0x86: ("set_clk_divisor", 2),
    0x87: ("send_immediate", 0),
    0x88: ("wait_io_high",  0),
    0x89: ("wait_io_low",   0),
    0x8A: ("div5_disable",  0),
    0x8B: ("div5_enable",   0),
    0x8C: ("three_phase_on", 0),
    0x8D: ("three_phase_off", 0),
    0x8E: ("clock_bits_no_data", 1),
    0x8F: ("clock_bytes_no_data", 2),
    0x94: ("clk_wait_io_high", 0),
    0x95: ("clk_wait_io_low", 0),
    0x96: ("adaptive_on",   0),
    0x97: ("adaptive_off",  0),
    0x9C: ("clk_bytes_until_high", 2),
    0x9D: ("clk_bytes_until_low", 2),
    0x9E: ("set_io_open_drain", 2),
}


def describe_shift(op: int) -> str:
    """Name a data-shifting opcode from its bit fields."""
    parts = []
    parts.append("write" if op & 0x10 else "")
    parts.append("read" if op & 0x20 else "")
    parts.append("tms" if op & 0x40 else "")
    what = "+".join(p for p in parts if p) or "none"
    return (
        f"{what}, {'bits' if op & 0x02 else 'bytes'}, "
        f"{'LSB' if op & 0x08 else 'MSB'} first, "
        f"write on {'-ve' if op & 0x01 else '+ve'}, "
        f"read on {'-ve' if op & 0x04 else '+ve'}"
    )


def parse_tshark_fields(path: str):
    """Parse `tshark -T fields` output: time, direction, endpoint, hex payload.

    Preferred over the usbmon text API, which truncates payloads at 32 bytes -
    this probe sends URBs up to ~100 bytes, and a truncated stream mis-frames
    every MPSSE command after the first cut.
    """
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 4 or not parts[3]:
                continue
            try:
                ts = int(float(parts[0]) * 1e6)          # us
                direction = "i" if parts[1] == "1" else "o"
                ep = int(parts[2])
                payload = bytes.fromhex(parts[3].replace(":", ""))
            except (ValueError, IndexError):
                continue
            if payload:
                yield ts, direction, ep, payload


def parse_usbmon(path: str, dev: str | None):
    """Yield (timestamp, direction, endpoint, payload bytes)."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            if m.group("type") != "B":            # bulk only
                continue
            if dev and m.group("dev") != dev:
                continue
            rest = m.group("rest")
            if "=" not in rest:
                continue
            head, data = rest.split("=", 1)
            payload = bytes.fromhex("".join(data.split()))
            if not payload:
                continue
            # Host-to-device data rides on the submit; device-to-host on the
            # completion.  The other combination carries no payload.
            ev, direction = m.group("ev"), m.group("dir")
            if (direction == "o" and ev != "S") or (direction == "i" and ev != "C"):
                continue
            yield int(m.group("ts")), direction, int(m.group("ep")), payload


def decode_out(payload: bytes, show_bits: bool, bit_sink: list | None = None) -> list[str]:
    """Decode a host-to-probe MPSSE byte stream into human-readable steps.

    IMPORTANT: pass the *concatenated* OUT stream, not individual USB packets.
    A single MPSSE command may straddle a bulk transfer - a 43-byte shift sent
    in a 32-byte packet continues in the next one - so decoding per packet
    mis-frames every command after the first split.

    When `bit_sink` is given, each write command appends
    (kind, bit_list, byte_offset) so the wire-level bit sequence can be
    reconstructed and searched.
    """
    out: list[str] = []
    i = 0
    n = len(payload)
    while i < n:
        op = payload[i]
        i += 1
        if op in STATIC_OPS:
            name, argc = STATIC_OPS[op]
            args = payload[i:i + argc]
            i += argc
            if name in ("set_bits_low", "set_bits_high") and len(args) == 2:
                out.append(f"{name}: value=0x{args[0]:02X} dir=0x{args[1]:02X}")
            elif name == "set_clk_divisor" and len(args) == 2:
                div = args[0] | (args[1] << 8)
                # 60 MHz master clock on an FT2232H, /5 unless div-by-5 is off.
                out.append(f"{name}: {div} -> {30e6 / (div + 1) / 1e6:.3f} MHz "
                           f"(or /5 of that with div5 enabled)")
            elif name in ("clock_bits_no_data", "clock_bytes_no_data"):
                count = args[0] + 1 if len(args) == 1 else (
                    (args[0] | (args[1] << 8)) + 1) * 8 if len(args) == 2 else 0
                out.append(f"{name}: {count} clocks, no data")
            else:
                out.append(name + (f" {args.hex()}" if args else ""))
            continue

        if op & 0x80:
            out.append(f"unknown opcode 0x{op:02X}")
            continue

        # Data-shifting command.
        desc = describe_shift(op)
        if op & 0x02:                                     # bit mode
            if i >= n:
                out.append(f"0x{op:02X} truncated")
                break
            nbits = payload[i] + 1
            i += 1
            data = b""
            if op & 0x10 or op & 0x40:                    # writes carry a byte
                data = payload[i:i + 1]
                i += 1
            line = f"0x{op:02X} {desc}: {nbits} bit(s)"
            if data:
                bits = [(data[0] >> b) & 1 for b in range(nbits)] if op & 0x08 \
                    else [(data[0] >> (7 - b)) & 1 for b in range(nbits)]
                line += f" data=0x{data[0]:02X}"
                if show_bits:
                    line += " [" + "".join(str(b) for b in bits) + "]"
                if bit_sink is not None:
                    bit_sink.append(("write", bits, i))
            out.append(line)
        else:                                             # byte mode
            if i + 1 >= n:
                out.append(f"0x{op:02X} truncated")
                break
            nbytes = (payload[i] | (payload[i + 1] << 8)) + 1
            i += 2
            data = b""
            if op & 0x10 or op & 0x40:
                data = payload[i:i + nbytes]
                i += nbytes
            line = f"0x{op:02X} {desc}: {nbytes} byte(s)"
            if data:
                line += " data=" + data[:16].hex()
                if len(data) > 16:
                    line += f"...(+{len(data) - 16})"
                if bit_sink is not None:
                    bits = []
                    for byte in data:
                        if op & 0x08:                     # LSB first
                            bits.extend((byte >> b) & 1 for b in range(8))
                        else:
                            bits.extend((byte >> (7 - b)) & 1 for b in range(8))
                    bit_sink.append(("write", bits, i - len(data)))
            elif bit_sink is not None and op & 0x20:      # read-only shift
                bit_sink.append(("read", [None] * (nbytes * 8), i))
            out.append(line)
    return out


def find_pattern(bits: list[int], pattern: list[int]) -> list[int]:
    """Every offset where `pattern` occurs in `bits`, ignoring byte alignment."""
    hits = []
    n, m = len(bits), len(pattern)
    for i in range(n - m + 1):
        if bits[i:i + m] == pattern:
            hits.append(i)
    return hits


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--dev", help="usbmon device number, e.g. 004")
    ap.add_argument("--ep-out", type=int, default=2)
    ap.add_argument("--ep-in", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0, help="stop after N events")
    ap.add_argument("--bits", action="store_true", help="expand bit payloads")
    ap.add_argument("--stream", action="store_true",
                    help="decode the concatenated stream (correct; commands span packets)")
    ap.add_argument("--summary", action="store_true",
                    help="opcode histogram instead of a transcript")
    args = ap.parse_args()

    # tshark field output has tab-separated columns; usbmon text does not.
    with open(args.capture, encoding="utf-8", errors="replace") as fh:
        head = fh.readline()
    if "	" in head:
        events = list(parse_tshark_fields(args.capture))
    else:
        events = list(parse_usbmon(args.capture, args.dev))
    if not events:
        print("no bulk payloads found - wrong device number, or the capture is empty")
        return 1

    if args.stream:
        out_stream = b"".join(p for _, d, ep, p in events
                              if d == "o" and ep == args.ep_out)
        # Every FTDI IN packet is prefixed with two modem-status bytes.
        in_stream = b"".join(p[2:] for _, d, ep, p in events
                             if d == "i" and ep == args.ep_in and len(p) > 2)
        print(f"OUT stream {len(out_stream)} bytes, IN stream {len(in_stream)} bytes\n")

        sink: list = []
        steps = decode_out(out_stream, args.bits, sink)
        if not args.summary:
            for s in steps[:args.limit or len(steps)]:
                print("  " + s)

        wire: list[int] = []
        for kind, bits, _off in sink:
            if kind == "write":
                wire.extend(bits)
        print(f"\nreconstructed {len(wire)} written bits")

        # Our own sync frame, as the frame layer builds it: start bit, CMD 0x10
        # and LEN 63 LSB-first, CRC6 = 9, trailing zero.  19 bits, 0x09FE1.
        sync = [1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0]
        hits = find_pattern(wire, sync)
        print(f"our 19-bit sync frame (0x09FE1) occurs {len(hits)} time(s)"
              + (f" at bit offsets {hits[:10]}" if hits else ""))

        # And the first 64 bits actually put on the wire, which is the opening
        # sequence any probe has to match.
        head = "".join(str(b) for b in wire[:96])
        print(f"first 96 written bits: {head}")
        return 0

    first_ts = events[0][0]
    histogram: dict[str, int] = {}
    shown = 0

    for ts, direction, ep, payload in events:
        if direction == "o" and ep != args.ep_out:
            continue
        if direction == "i" and ep != args.ep_in:
            continue

        us = (ts - first_ts)
        if direction == "o":
            steps = decode_out(payload, args.bits)
            for s in steps:
                key = s.split(":")[0]
                histogram[key] = histogram.get(key, 0) + 1
            if not args.summary:
                print(f"[{us:>10} us] OUT {len(payload):>4}B")
                for s in steps:
                    print(f"                  {s}")
        else:
            # The first two bytes of every FTDI IN packet are modem status.
            body = payload[2:] if len(payload) >= 2 else b""
            if not args.summary and body:
                print(f"[{us:>10} us] IN  {len(body):>4}B  {body[:32].hex()}"
                      + ("..." if len(body) > 32 else ""))
            if body:
                histogram["<reply bytes>"] = histogram.get("<reply bytes>", 0) + len(body)

        shown += 1
        if args.limit and shown >= args.limit:
            break

    if args.summary:
        print(f"{len(events)} bulk events, {shown} on the MPSSE endpoints\n")
        for key, count in sorted(histogram.items(), key=lambda kv: -kv[1]):
            print(f"  {count:>8}  {key}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
