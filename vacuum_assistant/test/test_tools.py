"""The tools the language model may call, against a stand-in robot."""

import json
from datetime import datetime
from types import SimpleNamespace

from vacuum_assistant import describe, namer, tools
from vacuum_assistant.tools import RobotError, ToolBox

NOW = datetime(2026, 11, 22, 18, 0).timestamp()


class Robot:
    """Records what it was asked; fails on request."""

    def __init__(self, fail=None):
        self.fail = fail
        self.asked = []

    def _do(self, what, out):
        self.asked.append(what)
        if self.fail == what[0]:
            raise RobotError("the mode manager is not running")
        return out

    def status(self):
        return self._do(("status",), {"mode": "docked"})

    def history(self, count):
        return self._do(("history", count), {"cleans": [{"started": NOW - 3 * 3600, "ended": NOW - 2.5 * 3600, "what": "everything"}]})

    def rooms_report(self):
        return self._do(("rooms",), {"rooms": [{"name": "Kitchen", "last_cleaned": NOW - 86400, "last_passed_through": 0.0}]})

    def picture(self):
        return self._do(("picture",), b"jpeg")

    def look_around(self):
        return self._do(("look_around",), [b"one", b"two"])

    def where(self):
        return {"x": 1.0, "y": 2.0, "room": "Kitchen"}

    def find(self, name):
        return self._do(("find", name), ({"what": name, "x": 3.0, "y": 4.0, "distance_m": 1.2}, b"seen", "note"))

    def remembered(self, name):
        return {"what": "sofa", "x": 1.0, "y": 1.0, "seen": NOW - 600} if name == "sofa" else None

    def find_around(self, name):
        return self._do(("find_around", name), (None, b"looked"))

    def go_to_room(self, room):
        return self._do(("go_to_room", room), {"room": room, "outcome": "arrived", "x": 5.0, "y": 6.0})

    def go_near(self, name):
        return self._do(("go_near", name), ({"found": True, "thing": {"x": 7.0, "y": 8.0}, "outcome": "arrived"}, b"after"))

    def start_cleaning(self, target, passes):
        return self._do(("clean", target, passes), {"started": True})

    def pause(self, resume):
        return self._do(("pause", resume), {"done": True})

    def stop(self):
        return self._do(("stop",), {"done": True})

    def dock(self):
        return self._do(("dock",), {"done": True})

    def change_settings(self, changes):
        return self._do(("settings", changes), {"done": True})


def box(robot=None, **kw):
    return ToolBox(robot or Robot(), clock=lambda: NOW, **kw)


def test_every_tool_has_a_closed_schema_and_strict_ones_require_every_field():
    names = [t["name"] for t in tools.TOOLS]
    assert len(names) == len(set(names)) and set(names) == set(tools.TOOL_WORDS)
    for t in tools.TOOLS:
        schema = t["input_schema"]
        assert schema["additionalProperties"] is False
        if t.get("strict"):
            assert set(schema["required"]) == set(schema["properties"])


def test_reading_tools_write_times_as_people_say_them():
    out = box().run("get_status", {})
    assert json.loads(out.text) == {"mode": "docked", "time_now": "Sunday 22 November 2026, 18:00"}
    history = json.loads(box().run("get_cleaning_history", {"count": 500}).text)
    assert history["cleans"][0]["started"] == "Sunday 22 Nov, 15:00 (3 hours ago)"
    rooms = json.loads(box().run("get_rooms", {}).text)["rooms"][0]
    assert rooms["last_cleaned"].endswith("(24 hours ago)") and rooms["last_passed_through"] == "never"


def test_look_and_find_give_the_picture_and_the_place():
    out = box().run("look", {"around": False})
    assert out.pictures == [b"jpeg"] and json.loads(out.text)["robot"]["room"] == "Kitchen"
    assert box().run("look", {"around": True}).pictures == [b"one", b"two"]
    found = box().run("find_object", {"name": "sofa", "look_around": False})
    assert found.pictures == [b"seen"] and found.mark == (3.0, 4.0) and json.loads(found.text)["found"] is True
    assert json.loads(found.text)["seen_before_at"]["seen"].endswith("(10 minutes ago)")
    robot = Robot()
    missed = box(robot).run("find_object", {"name": " red   chair ", "look_around": True})
    assert robot.asked == [("find_around", "red chair")] and json.loads(missed.text)["found"] is False and missed.mark is None


def test_actions_go_to_the_robot_with_their_arguments():
    robot = Robot()
    b = box(robot)
    assert b.run("go_to_room", {"room": "Kitchen"}).mark == (5.0, 6.0)
    near = b.run("go_near", {"name": "sofa"})
    assert near.pictures == [b"after"] and near.mark == (7.0, 8.0)
    b.run("start_cleaning", {"target": "Kitchen", "passes": 2})
    b.run("pause_cleaning", {"resume": True})
    b.run("stop", {})
    b.run("go_to_dock", {})
    b.run("change_settings", {"cleaning_speed": 0.25, "row_direction": "45"})
    assert robot.asked == [("go_to_room", "Kitchen"), ("go_near", "sofa"), ("clean", "Kitchen", 2), ("pause", True), ("stop",),
                           ("dock",), ("settings", {"cleaning_speed": 0.25, "row_direction": "45"})]


def test_what_cannot_be_done_comes_back_as_an_error_with_the_reason():
    assert box(Robot(fail="status")).run("get_status", {}).text == "could not: the mode manager is not running"
    assert box().run("go_to_room", {"room": " "}).is_error
    assert box().run("change_settings", {}).is_error
    assert box().run("change_settings", {"cleaning_speed": "fast"}).is_error
    assert box().run("get_cleaning_history", {"count": "five"}).is_error
    assert box().run("drive_off", {}).is_error
    assert box().run("learn_rooms", {}).is_error  # not offered without the node's job
    assert json.loads(box(learn_rooms=lambda: {"saved": True}).run("learn_rooms", {}).text) == {"saved": True}


def test_when():
    assert tools.when(0) == "never"
    assert tools.when(NOW - 30, NOW).endswith("(just now)")
    assert tools.when(NOW - 20 * 60, NOW).endswith("(20 minutes ago)")
    assert tools.when(NOW - 3 * 86400, NOW).endswith("(3 days ago)")


def test_room_names_from_everything_seen_are_tidied_and_numbered():
    assert namer.clean_name("  living room!! ") == "Living room"
    assert namer.unique(["Bedroom", "Kitchen", "bedroom", "", "Bath"]) == ["Bedroom 1", "Kitchen", "bedroom 2", "", "Bath"]
    square = lambda x0: [(x0, 0.0), (x0 + 2.0, 0.0), (x0 + 2.0, 2.0), (x0, 2.0)]  # noqa: E731
    assert namer.neighbours([square(0.0), square(2.1), square(9.0)]) == [[1], [0], []]

    class Model:
        def __init__(self, text, stop="end_turn"):
            self.messages = self
            self.text, self.stop, self.request = text, stop, None

        def create(self, **request):
            self.request = request
            return SimpleNamespace(stop_reason=self.stop, content=[SimpleNamespace(type="text", text=self.text)])

    rooms = [namer.Evidence(12.0, [b"a", b"b"], {"stove": 0.6}, [2]), namer.Evidence(3.0, [], {}, [1], reached=False)]
    model = Model('{"rooms": [{"room": 1, "name": "kitchen", "why": "a stove"}, {"room": 7, "name": "x", "why": ""}]}')
    assert namer.name_rooms(model, "m", rooms) == [("Kitchen", "a stove"), ("", "")]
    content = model.request["messages"][0]["content"]
    assert [c["type"] for c in content] == ["text", "image", "image", "text"]
    facts = content[-1]["text"]
    assert "Room 1: 12.0 m2 of floor, opens onto room 2; the object finder saw stove (60% sure)" in facts
    assert "could not get in" in facts
    assert model.request["output_config"]["format"]["type"] == "json_schema"
    assert namer.name_rooms(Model("not json"), "m", rooms) == [("", ""), ("", "")]
    assert namer.name_rooms(Model("", stop="refusal"), "m", rooms) == [("", ""), ("", "")]


def _clean(state="finished", **values):
    base = dict(target="Living room", state=state, message="cleaned 21.0 m² (96%) in 14 min", cleaned_area=21.04, planned_area=22.0,
                coverage=95.6, cleaning_time=840.0, distance=88.4, rows_done=31, rows_total=31, rows_skipped=1, bumps=0, time_left=120.0)
    base.update(values)
    return SimpleNamespace(**base)


def test_status_says_what_the_mode_means_and_the_clean_under_way():
    state = SimpleNamespace(state="cleaning", transitioning=False, reason="requested", faults=[], making_map=False, current_map="house")
    settings = SimpleNamespace(cleaning_speed=0.2, manual_speed=0.3, row_spacing=0.25, row_angle_auto=False, row_angle=45.0)
    out = describe.status(state, _clean("cleaning"), settings)
    assert out["mode_meaning"].startswith("cleaning") and out["map"] == "house"
    assert out["clean_under_way"]["minutes_left"] == 2.0 and out["clean_under_way"]["cleaned_m2"] == 21.0
    assert out["settings"]["row_direction"] == "45 degrees from the map's x axis"


def test_history_keeps_the_numbers_and_totals():
    record = SimpleNamespace(started=100.0, ended=940.0, status=_clean())
    totals = SimpleNamespace(cleans=23, area=412.5, time=18600.0, distance=1520.25)
    out = describe.history([record], totals, "house")
    assert out["cleans"][0]["ended_as"] == "finished" and out["cleans"][0]["rows_skipped_blocked"] == 1
    assert out["totals_over_every_clean"] == {"cleans": 23, "area_m2": 412.0, "hours": 5.2, "distance_m": 1520.0}


def test_a_find_is_checked_with_a_yes_or_no():
    class Model:
        def __init__(self, text):
            self.messages = self
            self.text, self.request = text, None

        def create(self, **request):
            self.request = request
            return SimpleNamespace(stop_reason="end_turn", content=[SimpleNamespace(type="text", text=self.text)])

    yes = Model('{"answer": "yes"}')
    assert namer.is_it(yes, "m", b"jpg", "sofa", "the box labelled 'sofa 0.39'")
    assert "sofa 0.39" in yes.request["messages"][0]["content"][1]["text"]
    assert not namer.is_it(Model('{"answer": "unsure"}'), "m", b"jpg", "sofa", "box")
    assert not namer.is_it(Model("nonsense"), "m", b"jpg", "sofa", "box")
