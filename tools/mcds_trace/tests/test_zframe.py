"""The compressed trace stream's framing (zframe)."""

import os
import struct

import lz4.block

from mcds_trace.zframe import MAGIC, STORED, unframe

CAPTURE = os.path.join(os.path.dirname(__file__), '..', 'cycles.mcds')


def _z(raw: bytes) -> bytes:
    """A frame as the probe sends it compressed."""
    return MAGIC + struct.pack('<I', len(raw)) + lz4.block.compress(raw, store_size=False)


def _stored(raw: bytes) -> bytes:
    return MAGIC + struct.pack('<I', len(raw) | STORED) + raw


def test_compressed_frames_of_a_real_capture():
    data = open(CAPTURE, 'rb').read()
    raw = data[data.find(b'DTRP'):]
    frames = [raw[k:k + 16384] for k in range(0, len(raw), 16384)]
    assert b''.join(unframe(_z(f)) for f in frames) == raw


def test_stored_frame():
    raw = os.urandom(1000)
    assert unframe(_stored(raw)) == raw


def test_raw_frame_passes_through():
    raw = b'DTRP' + bytes(100)
    assert unframe(raw) == raw
    assert unframe(b'') == b''
