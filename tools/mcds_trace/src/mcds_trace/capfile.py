"""Capture files: a JSON config header followed by the probe's DTRP records.

    "MCDSCAP1" | u32 json_len | json (utf-8) | record*

record (dap_trace_record_t, little-endian):
    u32 magic "DTRP" | u32 seq | u32 tram_offset | u16 length | u16 flags | u32 lost
    followed by `length` payload bytes (one TRAM paragraph).

The web page writes the same layout, so both capture paths decode alike.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from typing import BinaryIO, Iterator

FILE_MAGIC = b'MCDSCAP1'
REC_MAGIC = 0x50525444          # "DTRP"
REC_HEADER = struct.Struct('<IIIHHI')
FLAG_GAP = 1 << 0
FLAG_FINAL = 1 << 1               # the session's last paragraph (may end early)


@dataclass
class Record:
    seq: int
    tram_offset: int
    flags: int
    lost: int
    payload: bytes

    @property
    def gap(self) -> bool:
        return bool(self.flags & FLAG_GAP)

    @property
    def final(self) -> bool:
        return bool(self.flags & FLAG_FINAL)


def write_header(f: BinaryIO, config: dict) -> None:
    blob = json.dumps(config, sort_keys=True).encode()
    f.write(FILE_MAGIC + struct.pack('<I', len(blob)) + blob)


def read_file(path: str) -> tuple[dict, bytes]:
    """Return (config, record stream bytes)."""
    with open(path, 'rb') as f:
        data = f.read()
    if data.startswith(FILE_MAGIC):
        (n,) = struct.unpack_from('<I', data, len(FILE_MAGIC))
        start = len(FILE_MAGIC) + 4
        return json.loads(data[start:start + n]), data[start + n:]
    return {}, data                     # bare record stream (old .dtrp)


def records(stream: bytes) -> Iterator[Record]:
    """Parse DTRP records, resynchronising on the magic after damage."""
    pos = 0
    magic = struct.pack('<I', REC_MAGIC)
    while pos + REC_HEADER.size <= len(stream):
        m, seq, off, length, flags, lost = REC_HEADER.unpack_from(stream, pos)
        if m != REC_MAGIC or pos + REC_HEADER.size + length > len(stream):
            nxt = stream.find(magic, pos + 1)
            if nxt < 0:
                return
            pos = nxt
            continue
        start = pos + REC_HEADER.size
        yield Record(seq, off, flags, lost, stream[start:start + length])
        pos = start + length
