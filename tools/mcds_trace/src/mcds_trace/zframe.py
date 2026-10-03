"""The probe's compressed trace stream (/ws/trace?z=1).

Each frame is an 8-byte header - b"DTRZ" and a little-endian u32, the raw
length, its top bit set when the data follows stored - then the frame's bytes
as one LZ4 block.  Frames are independent.  A frame without the header is raw:
a probe that predates compression ignores the query and sends the plain
stream, which this passes through.
"""

from __future__ import annotations

import struct

import lz4.block

MAGIC = b'DTRZ'
STORED = 0x80000000

# Ask for it in the stream URL.
QUERY = '?z=1'


def unframe(frame: bytes) -> bytes:
    """The raw DTRP bytes one frame carries."""
    if frame[:4] != MAGIC or len(frame) < 8:
        return frame
    (n,) = struct.unpack_from('<I', frame, 4)
    if n & STORED:
        return frame[8:8 + (n & ~STORED)]
    return lz4.block.decompress(frame[8:], uncompressed_size=n)
