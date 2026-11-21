"""How one message travels inside one WebSocket binary message. No ROS, no networking.

    byte 0      channel: 0 = command (app -> gateway), 1 = data (gateway -> app)
    byte 1      type of the protobuf message that follows (proto/vacuum.proto)
    bytes 2-5   its length in bytes, unsigned, big-endian
    bytes 6-    the protobuf message

The length has four bytes: two bytes would cap a message at 64 KB, which a map of a larger
house would pass.
"""

import struct
from typing import NamedTuple

_HEADER = struct.Struct(">BBI")
HEADER_SIZE = _HEADER.size  # 6


class FrameError(ValueError):
    """A message that cannot be framed, or bytes that are not a valid frame."""


class Frame(NamedTuple):
    channel: int
    type: int
    payload: bytes


def encode(channel: int, msg_type: int, payload: bytes, max_payload: int) -> bytes:
    if not (0 <= channel <= 0xFF and 0 <= msg_type <= 0xFF):
        raise FrameError(f"channel {channel} or type {msg_type} does not fit in one byte")
    if len(payload) > max_payload:
        raise FrameError(f"message of {len(payload)} bytes is over the limit of {max_payload}")
    return _HEADER.pack(channel, msg_type, len(payload)) + payload


def decode(data: bytes, max_payload: int) -> Frame:
    if len(data) < HEADER_SIZE:
        raise FrameError(f"{len(data)} bytes is shorter than the {HEADER_SIZE}-byte header")
    channel, msg_type, length = _HEADER.unpack_from(data)
    if length > max_payload:
        raise FrameError(f"message of {length} bytes is over the limit of {max_payload}")
    if len(data) - HEADER_SIZE != length:
        raise FrameError(f"the header says {length} bytes follow, but {len(data) - HEADER_SIZE} do")
    return Frame(channel, msg_type, bytes(data[HEADER_SIZE:]))
