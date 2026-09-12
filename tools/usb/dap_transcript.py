#!/usr/bin/env python3
"""Turn a captured FT2232 session into a DAP-level transcript, replies included.

decode_mpsse.py reconstructs what a probe *drove*; this walks the OUT command
stream and the IN data stream together, so each read command is paired with the
bytes that came back.  That is what makes the transcript readable as a
conversation rather than a monologue: command frame, then the target's answer.

Every DAP frame's CRC6 is recomputed with the generator our firmware uses, so a
frame printed here with `ok` is one our implementation would have built
identically.

    dap_transcript.py capture.fields [--from-bit N] [--limit N]
"""

from __future__ import annotations

import argparse
import sys

import decode_mpsse as mp
import parse_dap_frames as pf


def bits_lsb(data: bytes) -> list[int]:
    out: list[int] = []
    for byte in data:
        out.extend((byte >> b) & 1 for b in range(8))
    return out


def walk(out_stream: bytes, in_stream: bytes):
    """Yield (driven_bit_position, kind, detail) across both streams.

    The IN stream is consumed in order: an MPSSE read command of n bytes takes
    the next n bytes the device sent, because the probe issues them
    sequentially and FTDI returns them in the same order.
    """
    i = 0
    in_pos = 0
    driven = 0
    n = len(out_stream)

    while i < n:
        op = out_stream[i]
        i += 1

        if op in mp.STATIC_OPS:
            name, argc = mp.STATIC_OPS[op]
            args = out_stream[i:i + argc]
            i += argc
            if name == "set_clk_divisor" and len(args) == 2:
                div = args[0] | (args[1] << 8)
                yield driven, "clk", f"{30.0 / (div + 1):.2f} MHz"
            elif name == "set_bits_high" and len(args) == 2:
                # Bit 4 of the high byte is this probe's direction control,
                # and it reads like our own RDnWR: set means the *target*
                # drives.  Confirmed by which commands follow each value -
                # 0x57 (bit set) always precedes reads, 0x67 always precedes
                # writes.
                yield driven, "dir", ("target drives (read)" if args[0] & 0x10
                                      else "probe drives (write)") + f" (0x{args[0]:02X})"
            continue

        if op & 0x80:
            continue

        if op & 0x02:                                     # bit mode
            count = out_stream[i] + 1
            i += 1
            if op & 0x10 or op & 0x40:
                data = out_stream[i:i + 1]
                i += 1
                yield driven, "write", bits_lsb(data)[:count]
                driven += count
            else:
                took = in_stream[in_pos:in_pos + 1]
                in_pos += 1
                yield driven, "read", (count, took)
        else:                                             # byte mode
            nbytes = (out_stream[i] | (out_stream[i + 1] << 8)) + 1
            i += 2
            if op & 0x10 or op & 0x40:
                data = out_stream[i:i + nbytes]
                i += nbytes
                yield driven, "write", bits_lsb(data)
                driven += nbytes * 8
            else:
                took = in_stream[in_pos:in_pos + nbytes]
                in_pos += nbytes
                yield driven, "read", (nbytes, took)


def frames_in(bits: list[int]) -> list[dict]:
    """Every non-overlapping CRC-valid frame in one write command's bits."""
    found = []
    i = 0
    while i < len(bits) - 12:
        if bits[i] != 1:
            i += 1
            continue
        fr = pf.parse(bits, i, 1)
        if fr and fr[0]["crc_ok"] and fr[0]["at"] == i:
            f = fr[0]
            found.append(f)
            i = f["at"] + 1 + 5 + 6 + f["data_bits"] + 6
        else:
            i += 1
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--from-bit", type=int, default=0)
    ap.add_argument("--limit", type=int, default=60)
    args = ap.parse_args()

    events = list(mp.parse_tshark_fields(args.capture))
    out_stream = b"".join(p for _, d, ep, p in events if d == "o" and ep == 2)
    in_stream = b"".join(p[2:] for _, d, ep, p in events
                         if d == "i" and ep == 1 and len(p) > 2)

    shown = 0
    for pos, kind, detail in walk(out_stream, in_stream):
        if pos < args.from_bit:
            continue
        if shown >= args.limit:
            break
        shown += 1

        if kind == "clk":
            print(f"[bit {pos:>6}] clock {detail}")
        elif kind == "dir":
            print(f"[bit {pos:>6}] {detail}")
        elif kind == "write":
            frames = frames_in(detail)
            if frames:
                for f in frames:
                    extra = f" data=0x{f['data']:X}" if f["data_bits"] else ""
                    print(f"[bit {pos:>6}] -> {f['name']:<12} LEN={f['len']:<3} "
                          f"crc={f['crc']:<2} ok{extra}")
            else:
                head = "".join(str(b) for b in detail[:48])
                print(f"[bit {pos:>6}] -> {len(detail)} bits, no CRC-valid frame: {head}...")
        else:
            nbytes, data = detail
            bits = bits_lsb(data)
            word = 0
            for k in range(min(32, len(bits))):
                word |= bits[k] << k
            allzero = not any(bits)
            note = "  (all zero - no reply)" if allzero else ""
            print(f"[bit {pos:>6}] <- read {nbytes}B: {data[:12].hex()}"
                  f"{'...' if len(data) > 12 else ''} first32=0x{word:08X}{note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
