"""Map arithmetic for the gateway. No ROS: plain numbers and numpy arrays in and out."""

import math

import numpy as np


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rle_encode(cells) -> bytes:
    """Run-length encode map cells (signed bytes) as pairs: run length minus 1, then the value.
    The format of Map.cells_rle in proto/vacuum.proto."""
    a = np.asarray(cells, dtype=np.int8).ravel()
    if a.size == 0:
        return b""
    breaks = np.flatnonzero(a[1:] != a[:-1]) + 1
    starts = np.concatenate(([0], breaks))
    lengths = np.diff(np.concatenate((starts, [a.size])))
    values = a[starts]
    # A run longer than 256 cells is written as several runs of at most 256.
    pieces = (lengths + 255) // 256
    run_lengths = np.full(int(pieces.sum()), 256, dtype=np.int64)
    run_lengths[np.cumsum(pieces) - 1] = lengths - 256 * (pieces - 1)
    out = np.empty(2 * run_lengths.size, dtype=np.uint8)
    out[0::2] = run_lengths - 1
    out[1::2] = np.repeat(values, pieces).view(np.uint8)
    return out.tobytes()


def rle_decode(data: bytes) -> np.ndarray:
    """The inverse of rle_encode: the cells as int8."""
    pairs = np.frombuffer(data, dtype=np.uint8)
    if pairs.size % 2:
        raise ValueError("run-length data has an odd number of bytes")
    return np.repeat(pairs[1::2].view(np.int8), pairs[0::2].astype(np.int64) + 1)
