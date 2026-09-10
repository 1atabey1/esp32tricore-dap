#!/usr/bin/env python3
"""Parse a reconstructed DAP bit stream into frames, using CRC6 as the oracle.

Feed it the bits a working probe drove (see decode_mpsse.py --stream) and it
walks them as DAP downstream frames:

    start bit (1) | CMD (5) | LEN (6) | DATA (LEN) | CRC6 (6) | trailing 0

every field LSB first.  Each frame's CRC6 is recomputed with the generator our
firmware uses - a right-shifting Galois LFSR, polynomial 0x30, seed 0x20 - so a
run of frames that all check out is independent confirmation that our framing
matches the silicon.  A frame that does not check out is either a
misunderstanding of the layout or a place where the probe is doing something
other than sending a frame.

`sync` is the one command whose LEN is not a data length: it goes out with LEN
all ones as a JTAG-TAP safety measure and carries no payload.
"""

from __future__ import annotations

import argparse
import sys

CMD_NAMES = {
    0x10: "sync",
    0x11: "dapisc",
    0x12: "poll",
    0x13: "jtag_reset",
    0x18: "get_CRCdown",
    0x19: "get_CRCup",
    0x1A: "client_read",
    0x1B: "client_write",
    0x1C: "client_set",
    0x1D: "client_reset",
}

#: Commands documented as carrying LEN = 0 rather than a payload length.
NO_DATA_CMDS = {0x10, 0x12, 0x13, 0x18, 0x19, 0x1D}


def crc6(bits: list[int]) -> int:
    """Galois LFSR, poly 0x30, seed 0x20 - the generator the target accepts."""
    crc = 0x20
    for b in bits:
        fb = (crc ^ b) & 1
        crc >>= 1
        if fb:
            crc ^= 0x30
    return crc & 0x3F


def take(bits: list[int], pos: int, n: int) -> tuple[int, int]:
    """Read `n` bits LSB-first starting at `pos`; return (value, new_pos)."""
    v = 0
    for k in range(n):
        v |= (bits[pos + k] & 1) << k
    return v, pos + n


def parse(bits: list[int], start: int, limit: int) -> list[dict]:
    """Walk frames from `start`, skipping runs of idle zeros between them."""
    frames: list[dict] = []
    pos = start
    n = len(bits)

    while pos < n and len(frames) < limit:
        # Frames begin with a one; anything else is idle or turnaround.
        skipped = 0
        while pos < n and bits[pos] != 1:
            pos += 1
            skipped += 1
        if pos + 12 >= n:
            break

        frame_start = pos
        pos += 1                                  # start bit
        cmd, pos = take(bits, pos, 5)
        length, pos = take(bits, pos, 6)

        data_bits = 0 if cmd in NO_DATA_CMDS else length
        if pos + data_bits + 6 > n:
            break
        data, pos = take(bits, pos, data_bits) if data_bits else (0, pos)
        crc_seen, pos = take(bits, pos, 6)

        payload = []
        for k in range(5):
            payload.append((cmd >> k) & 1)
        for k in range(6):
            payload.append((length >> k) & 1)
        for k in range(data_bits):
            payload.append((data >> k) & 1)

        ok = crc6(payload) == crc_seen
        trailing = bits[pos] if pos < n else None
        if trailing == 0:
            pos += 1

        frames.append({
            "at": frame_start,
            "idle_before": skipped,
            "cmd": cmd,
            "name": CMD_NAMES.get(cmd, f"0x{cmd:02X}?"),
            "len": length,
            "data_bits": data_bits,
            "data": data,
            "crc": crc_seen,
            "crc_ok": ok,
            "trailing_zero": trailing == 0,
        })

        # A bad CRC means the layout assumption broke; resynchronise rather
        # than emitting a cascade of nonsense from the same wrong offset.
        if not ok:
            pos = frame_start + 1

    return frames


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bitfile", help="file of '0'/'1' characters")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--limit", type=int, default=60)
    args = ap.parse_args()

    text = open(args.bitfile, encoding="utf-8").read()
    bits = [1 if c == "1" else 0 for c in text if c in "01"]
    print(f"{len(bits)} bits, parsing from {args.start}\n")

    frames = parse(bits, args.start, args.limit)
    good = sum(1 for f in frames if f["crc_ok"])
    for f in frames:
        flag = "ok " if f["crc_ok"] else "BAD"
        extra = f" data=0x{f['data']:X}({f['data_bits']}b)" if f["data_bits"] else ""
        print(f"  bit {f['at']:>6}  +{f['idle_before']:>3} idle  {flag}  "
              f"{f['name']:<12} LEN={f['len']:<3} crc={f['crc']:<2}{extra}")
    print(f"\n{good}/{len(frames)} frames pass CRC6")
    return 0


if __name__ == "__main__":
    sys.exit(main())
