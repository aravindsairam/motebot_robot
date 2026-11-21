import os

import pytest

from vacuum_gateway import convert, zones
from vacuum_gateway import vacuum_pb2 as pb


def write(tmp_path, text, name="zones.yaml"):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def test_reads_names_and_corners(tmp_path):
    path = write(tmp_path, "zones:\n  - name: stairwell\n    polygon: [[4.55, 2.0], [5.65, 2.0], [5.65, 3.3]]\n")
    assert zones.read_zones(path) == [("stairwell", [(4.55, 2.0), (5.65, 2.0), (5.65, 3.3)])]


def test_a_missing_file_or_an_empty_list_means_no_zones(tmp_path):
    assert zones.read_zones(str(tmp_path / "nothing.yaml")) == []
    assert zones.read_zones("") == []
    assert zones.read_zones(write(tmp_path, "zones: []\n")) == []
    assert zones.read_zones(write(tmp_path, "")) == []


@pytest.mark.parametrize("text", [
    "zones:\n  - name: a\n    polygon: [[0, 0], [1, 1]]\n",          # two corners
    "zones:\n  - name: a\n    polygon: [[0, 0], [1], [2, 2]]\n",     # a corner with one number
    "zones:\n  - name: a\n    polygon: [[0, x], [1, 1], [2, 2]]\n",  # not a number
    "zones:\n  - name: a\n",                                         # no polygon
    "- just a list\n",
    "zones: [\n",                                                    # not YAML
])
def test_a_broken_file_is_reported(tmp_path, text):
    with pytest.raises(ValueError):
        zones.read_zones(write(tmp_path, text))


def test_the_file_is_read_again_only_when_it_changes(tmp_path):
    path = write(tmp_path, "zones:\n  - name: a\n    polygon: [[0, 0], [1, 0], [1, 1]]\n")
    watch = zones.ZonesFile(path)
    assert [name for name, _ in watch.changed()] == ["a"]
    assert watch.changed() is None
    with open(path, "w") as f:
        f.write("zones:\n  - name: b\n    polygon: [[0, 0], [2, 0], [2, 2]]\n  - name: c\n    polygon: [[0, 0], [3, 0], [3, 3]]\n")
    os.utime(path, ns=(1, 1))  # a different time, whatever the file system's resolution
    assert [name for name, _ in watch.changed()] == ["b", "c"]
    os.remove(path)
    assert watch.changed() == []
    assert watch.changed() is None


def test_a_file_replaced_by_one_of_the_same_size_and_time_is_still_seen(tmp_path):
    # The mode manager writes a new file and renames it over the old one.
    path = write(tmp_path, "zones:\n  - name: a\n    polygon: [[0, 0], [1, 0], [1, 1]]\n")
    watch = zones.ZonesFile(path)
    watch.changed()
    stat = os.stat(path)
    replacement = write(tmp_path, "zones:\n  - name: b\n    polygon: [[0, 0], [1, 0], [1, 1]]\n", "new.yaml")
    os.utime(replacement, ns=(stat.st_atime_ns, stat.st_mtime_ns))
    os.replace(replacement, path)
    assert [name for name, _ in watch.changed()] == ["b"]


def test_a_broken_file_is_tried_again(tmp_path):
    path = write(tmp_path, "zones: [\n")
    watch = zones.ZonesFile(path)
    for _ in range(2):
        with pytest.raises(ValueError):
            watch.changed()  # reported each time, not taken as "unchanged" after the first


def test_zones_are_sent_in_centimetres():
    out = convert.no_go_zones([("stairwell", [(4.554, 2.0), (5.65, -2.006), (0.0, 3.3)])], "house")
    assert isinstance(out, pb.NoGoZones)
    assert [(z.name, list(z.points_cm)) for z in out.zones] == [("stairwell", [455, 200, 565, -201, 0, 330])]
    assert out.map_name == "house"
    assert convert.no_go_zones([]).SerializeToString() == b""  # no saved map in use: nothing at all


# ---------------------------------------------------------------- the map in use
STAIRS = "zones:\n  - name: stairwell\n    polygon: [[4.55, 2.0], [5.65, 2.0], [5.65, 3.3]]\n"
RUG = "zones:\n  - name: rug\n    polygon: [[1, 1], [2, 1], [2, 2]]\n"


def names(found):
    map_name, zone_list = found
    return map_name, [name for name, _ in zone_list]


def test_nothing_is_known_before_the_robot_names_its_map(tmp_path):
    write(tmp_path, STAIRS, "house_zones.yaml")
    assert zones.MapZones(str(tmp_path)).changed() is None


def test_the_zones_follow_the_map_in_use(tmp_path):
    write(tmp_path, STAIRS, "house_zones.yaml")
    write(tmp_path, RUG, "upstairs_zones.yaml")
    watch = zones.MapZones(str(tmp_path))
    assert watch.follow("house")
    assert names(watch.changed()) == ("house", ["stairwell"])
    assert not watch.follow("house")  # the same map again (every robot state names it)
    assert watch.changed() is None
    assert watch.follow("upstairs")
    assert names(watch.changed()) == ("upstairs", ["rug"])
    assert watch.follow("")  # mapping, or no saved map
    assert watch.changed() == ("", [])
    assert watch.changed() is None
    assert watch.follow("house")
    assert names(watch.changed()) == ("house", ["stairwell"])


def test_a_map_without_a_zones_file_has_none(tmp_path):
    write(tmp_path, STAIRS, "house_zones.yaml")
    watch = zones.MapZones(str(tmp_path))
    watch.follow("house")
    watch.changed()
    watch.follow("phone_test")
    assert watch.changed() == ("phone_test", [])
    write(tmp_path, RUG, "phone_test_zones.yaml")  # zones added to it later
    assert names(watch.changed()) == ("phone_test", ["rug"])


def test_force_reads_the_file_again(tmp_path):
    write(tmp_path, STAIRS, "house_zones.yaml")
    watch = zones.MapZones(str(tmp_path))
    watch.follow("house")
    watch.changed()
    assert watch.changed() is None
    assert names(watch.changed(force=True)) == ("house", ["stairwell"])
    watch.follow("")
    watch.changed()
    assert watch.changed(force=True) == ("", [])


def test_the_phones_zones_in_metres():
    message = pb.SetNoGoZones(zones=[pb.NoGoZones.Zone(name="rug", points_cm=[100, -50, 200, -50, 150, 25])])
    assert zones.zones_from_phone(message.zones) == ([("rug", [(1.0, -0.5), (2.0, -0.5), (1.5, 0.25)])], "")
    assert zones.zones_from_phone([]) == ([], "")  # every zone deleted


def test_an_odd_number_of_coordinates_is_a_bad_request():
    message = pb.SetNoGoZones(zones=[pb.NoGoZones.Zone(name="ok", points_cm=[0, 0, 100, 0, 100, 100]),
                                     pb.NoGoZones.Zone(name="rug", points_cm=[100, -50, 200])])
    assert zones.zones_from_phone(message.zones) == (None, "zone 2 (rug) has an odd number of coordinates")
