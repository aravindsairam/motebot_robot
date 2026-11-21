import math
import os

import numpy as np
import pytest
import yaml
from PIL import Image

from vacuum_mode_manager import map_files as mf


def test_compose_translates_and_rotates():
    # frame rotated 90 degrees left at (1, 2); a point 1 m ahead of it is at (1, 3)
    x, y, yaw = mf.compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert (round(x, 6), round(y, 6)) == (1.0, 3.0)
    assert abs(yaw - math.pi / 2) < 1e-9


def test_compose_with_identity_and_yaw_wrap():
    assert mf.compose((0.0, 0.0, 0.0), (3.0, -1.0, 0.5)) == (3.0, -1.0, 0.5)
    _, _, yaw = mf.compose((0.0, 0.0, 3.0), (0.0, 0.0, 3.0))
    assert -math.pi <= yaw <= math.pi


def test_yaw_from_quaternion():
    assert abs(mf.yaw_from_quaternion(0.0, 0.0, math.sin(0.4), math.cos(0.4)) - 0.8) < 1e-9


def test_grid_to_image_values_and_row_order():
    # 2 wide, 3 tall; grid row 0 is the BOTTOM of the map
    data = [100, 0,
            -1, 50,
            0, 100]
    img = mf.grid_to_image(data, width=2, height=3)
    assert img.shape == (3, 2)
    assert img.tolist() == [
        [mf.FREE, mf.OCCUPIED],      # top image row = last grid row
        [mf.UNKNOWN, mf.UNKNOWN],    # -1 and the in-between value 50 are both unknown
        [mf.OCCUPIED, mf.FREE],      # bottom image row = grid row 0
    ]


def test_write_and_read_map(tmp_path):
    stem = str(tmp_path / "maps" / "test_map")
    img = np.full((40, 60), mf.FREE, dtype=np.uint8)
    path = mf.write_map(stem, img, 0.05, (-1.0, -2.0))
    assert path == stem + ".yaml"
    meta = yaml.safe_load(open(path))
    assert meta["image"] == "test_map.pgm" and meta["mode"] == "trinary" and meta["origin"] == [-1.0, -2.0, 0]
    # one "key: value" per line with the origin as a bracketed list, as Nav2's map_saver writes it
    text = open(path).read().splitlines()
    assert "origin: [-1, -2, 0]" in text and "resolution: 0.05" in text and "negate: 0" in text
    info = mf.read_map_meta(path)
    assert info == {"resolution": 0.05, "origin": (-1.0, -2.0), "width": 60, "height": 40}


def map_server_reads(yaml_path):
    """The grid Nav2's map_server makes of a saved map in trinary mode (nav2_map_server map_io):
    occupancy = (255 - pixel) / 255; occupied above occupied_thresh, free below free_thresh,
    unknown otherwise. Rows come back bottom first, as in an OccupancyGrid."""
    meta = yaml.safe_load(open(yaml_path))
    pixels = np.asarray(Image.open(os.path.join(os.path.dirname(yaml_path), meta["image"])), dtype=np.float64)
    occ = (255.0 - pixels) / 255.0
    grid = np.full(occ.shape, -1, dtype=np.int16)
    grid[occ > meta["occupied_thresh"]] = 100
    grid[occ < meta["free_thresh"]] = 0
    return np.flipud(grid)


def test_a_saved_map_reads_back_unchanged(tmp_path):
    # free, occupied and unknown cells, as slam_toolbox publishes them
    grid = np.array([[0, 100, -1],
                     [-1, 0, 0]], dtype=np.int16)
    path = mf.write_map(str(tmp_path / "m"), mf.grid_to_image(grid.ravel(), 3, 2), 0.05, (0.0, 0.0))
    assert map_server_reads(path).tolist() == grid.tolist()  # unknown stays unknown, not free


def test_keepout_mask_lines_up_with_the_map(tmp_path):
    stem = str(tmp_path / "house")
    mf.write_map(stem, np.full((40, 60), mf.FREE, dtype=np.uint8), 0.05, (-1.0, -1.0))  # 3 m x 2 m
    zones = [{"name": "corner", "polygon": [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]}]
    mask_yaml = mf.write_keepout_mask(stem + ".yaml", zones, stem + "_keepout")
    mask = np.array(Image.open(stem + "_keepout.pgm"))
    assert mask.shape == (40, 60)
    info = mf.read_map_meta(mask_yaml)
    assert info["origin"] == (-1.0, -1.0) and info["resolution"] == 0.05

    def cell(x, y):  # value of the cell containing map point (x, y)
        col = int((x + 1.0) / 0.05)
        row = 40 - 1 - int((y + 1.0) / 0.05)
        return mask[row, col]

    assert cell(1.5, 0.5) == mf.OCCUPIED      # inside the zone
    assert cell(0.5, 0.5) == mf.FREE          # left of it
    assert cell(1.5, -0.5) == mf.FREE         # below it
    assert cell(-0.9, -0.9) == mf.FREE        # far corner of the map


def test_no_zones_gives_an_all_free_mask(tmp_path):
    stem = str(tmp_path / "house")
    mf.write_map(stem, np.full((10, 10), mf.FREE, dtype=np.uint8), 0.05, (0.0, 0.0))
    mf.write_keepout_mask(stem + ".yaml", [], stem + "_keepout")
    assert (np.array(Image.open(stem + "_keepout.pgm")) == mf.FREE).all()
    assert mf.read_zones(str(tmp_path / "missing.yaml")) == []


def test_read_map_gives_the_grid_map_server_serves(tmp_path):
    grid = np.array([[0, 100, -1],
                     [-1, 0, 0]], dtype=np.int16)
    path = mf.write_map(str(tmp_path / "m"), mf.grid_to_image(grid.ravel(), 3, 2), 0.05, (-1.0, 2.0))
    cells, meta = mf.read_map(path)
    assert cells.dtype == np.int8
    assert cells.tolist() == map_server_reads(path).tolist()  # row 0 is the bottom, as in an OccupancyGrid
    assert meta == {"resolution": 0.05, "origin": (-1.0, 2.0), "width": 3, "height": 2}


def test_keepout_cells_are_the_mask_nav2_gets(tmp_path):
    # The cleaning planner and Nav2 must agree on every no-go cell: both come from one function.
    stem = str(tmp_path / "house")
    mf.write_map(stem, np.full((40, 60), mf.FREE, dtype=np.uint8), 0.05, (-1.0, -1.0))
    zones = [{"name": "z", "polygon": [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]},
             {"name": "tri", "polygon": [[-0.8, -0.8], [0.0, -0.8], [-0.8, 0.3]]}]
    mf.write_keepout_mask(stem + ".yaml", zones, stem + "_keepout")
    nav2 = np.flipud(np.array(Image.open(stem + "_keepout.pgm")) == mf.OCCUPIED)
    cells = mf.keepout_cells(mf.read_map_meta(stem + ".yaml"), zones)
    assert cells.dtype == bool and cells.shape == (40, 60)
    assert (cells == nav2).all()
    assert cells[int((0.5 + 1.0) / 0.05), int((1.5 + 1.0) / 0.05)]  # row from y, column from x
    assert not cells[int((0.5 + 1.0) / 0.05), int((0.5 + 1.0) / 0.05)]


# ---------------------------------------------------------------- a map's own files
def test_a_maps_zones_and_mask_are_named_after_it():
    assert mf.zones_path("/maps", "house") == "/maps/house_zones.yaml"
    assert mf.keepout_stem("/maps", "house") == "/maps/house_keepout"
    assert mf.zones_path_for("/maps/upstairs-2.yaml") == "/maps/upstairs-2_zones.yaml"


@pytest.mark.parametrize("name", ["house", "upstairs-2", "Flat_3", "0"])
def test_good_map_names(name):
    assert mf.map_name_problem(name) == ""


@pytest.mark.parametrize("name", [
    "", "_house", "-house", "my house", "house.yaml", "../house", "été",
    "house_keepout",  # would be the mask of the map 'house'
    "house_zones",    # its .yaml would be the zones of the map 'house'
])
def test_names_that_are_refused(name):
    assert "must not end in '_keepout' or '_zones'" in mf.map_name_problem(name)


def test_no_map_name_can_reach_another_maps_files():
    # The zones file of any allowed name is never the .yaml of another allowed name.
    for name in ("house", "a", "house-2"):
        other = os.path.basename(mf.zones_path("/m", name))[: -len(".yaml")]
        assert mf.map_name_problem(other) != ""


ZONES = [
    {"name": "stairwell", "polygon": [[4.55, 2.0], [5.65, 2.0], [5.65, 3.3], [4.55, 3.3]]},
    {"name": 'rug é "big": #1 \\ ok', "polygon": [[-1.0, -0.5], [0.0, 0.0], [-0.5, 1.0]]},
    {"name": "🐈 bowl", "polygon": [[0.123456, -0.0000001], [1.0, 0.0], [1.0, 1.0]]},
]


def test_zones_written_read_back_the_same(tmp_path):
    path = str(tmp_path / "maps" / "house_zones.yaml")  # the folder is made if missing
    mf.write_zones(path, ZONES)
    back = mf.read_zones(path)
    assert [z["name"] for z in back] == [z["name"] for z in ZONES]
    assert back[0]["polygon"] == ZONES[0]["polygon"]
    assert back[2]["polygon"] == [[0.1235, 0.0], [1.0, 0.0], [1.0, 1.0]]  # to 0.1 mm, and no "-0.0"
    text = open(path, encoding="utf-8").read()
    assert text.startswith("# No-go zones") and "rug é" in text  # readable: accents kept as they are
    assert "polygon: [[4.55, 2.0], [5.65, 2.0], [5.65, 3.3], [4.55, 3.3]]" in text


def test_zones_from_float32_corners_are_written_as_given(tmp_path):
    # geometry_msgs/Polygon holds float32: 4.55 arrives as 4.550000190734863.
    path = str(tmp_path / "z.yaml")
    mf.write_zones(path, [{"name": "a", "polygon": [[float(np.float32(4.55)), float(np.float32(-2.3))], [5, 2], [5, 3]]}])
    assert mf.read_zones(path)[0]["polygon"][0] == [4.55, -2.3]


def test_an_empty_list_writes_a_file_with_no_zones(tmp_path):
    path = str(tmp_path / "z.yaml")
    mf.write_zones(path, ZONES)
    mf.write_zones(path, [])
    assert os.path.isfile(path) and mf.read_zones(path) == []


def test_writing_replaces_the_file_whole_and_leaves_nothing_behind(tmp_path):
    path = str(tmp_path / "z.yaml")
    mf.write_zones(path, ZONES[:1])
    before = os.stat(path).st_ino
    mf.write_zones(path, ZONES)
    assert os.stat(path).st_ino != before  # a new file renamed over the old one, not rewritten in place
    assert os.listdir(tmp_path) == ["z.yaml"]
    assert os.stat(path).st_mode & 0o777 == 0o644


def test_a_failed_write_leaves_the_old_file(tmp_path, monkeypatch):
    path = str(tmp_path / "z.yaml")
    mf.write_zones(path, ZONES[:1])

    def broken_rename(src, dst):
        raise OSError("disk full")

    monkeypatch.setattr(mf.os, "replace", broken_rename)
    with pytest.raises(OSError):
        mf.write_zones(path, ZONES)
    assert [z["name"] for z in mf.read_zones(path)] == ["stairwell"]
    assert os.listdir(tmp_path) == ["z.yaml"]  # the temporary file is gone too


def test_empty_keepout_mask_is_small_all_free_and_no_maps_file(tmp_path):
    """Exploring starts Nav2 with no saved map: the mask it is given keeps the robot out of nothing."""
    mask_yaml = mf.write_empty_keepout_mask(str(tmp_path))
    cells, meta = mf.read_map(mask_yaml)
    assert (cells == 0).all()
    assert cells.size <= 100
    name = os.path.splitext(os.path.basename(mask_yaml))[0]
    assert mf.map_name_problem(name) != ""  # so it can never be, or overwrite, a saved map's file
    assert mf.map_name_problem(name[: -len("_keepout")]) != ""  # nor the mask of a saved map
