import math

import numpy as np
import pytest

from vacuum_gateway import geometry


def test_rle_layout():
    # three free cells, one occupied, two unknown: (count - 1, value) pairs, values as signed bytes
    assert geometry.rle_encode([0, 0, 0, 100, -1, -1]) == bytes([2, 0, 0, 100, 1, 0xFF])


def test_rle_splits_runs_longer_than_256():
    data = geometry.rle_encode([-1] * 600)
    assert data == bytes([255, 0xFF, 255, 0xFF, 87, 0xFF])  # 256 + 256 + 88


@pytest.mark.parametrize("cells", [
    [],
    [5],
    [-1] * 256,
    [-1] * 257,
    [0, 100] * 300,
    list(np.random.default_rng(1).choice([-1, 0, 100, 37], size=5000, p=[0.5, 0.3, 0.15, 0.05])),
])
def test_rle_round_trip(cells):
    assert geometry.rle_decode(geometry.rle_encode(cells)).tolist() == [int(c) for c in cells]


def test_rle_shrinks_a_map_like_grid():
    grid = np.full((112, 177), -1, dtype=np.int8)
    grid[10:100, 10:160] = 0
    grid[10:100, 10:13] = 100
    data = geometry.rle_encode(grid)
    assert len(data) < grid.size / 10


def test_rle_decode_refuses_odd_length():
    with pytest.raises(ValueError):
        geometry.rle_decode(b"\x01")


def test_yaw_from_quaternion():
    assert math.isclose(geometry.yaw_from_quaternion(0, 0, math.sin(0.6), math.cos(0.6)), 1.2)
