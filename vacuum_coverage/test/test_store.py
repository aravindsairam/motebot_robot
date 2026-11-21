"""Saved plans, records of each clean, history and totals, on disk."""

import os

import pytest
import yaml

from vacuum_coverage.planner import ROW, TRANSIT, Piece
from vacuum_coverage.store import CleaningStore, StoreError

PIECES = [Piece(TRANSIT, (0.0, 0.0), (0.5, 0.5)), Piece(ROW, (0.5, 0.5), (2.0, 0.5), 1, 1, 1)]
JOB = {"what": "area", "area": [[0.5, 0.5], [2.0, 0.5], [2.0, 1.5], [0.5, 1.5]], "direction": "auto", "passes": 1, "edges": False}
ESTIMATE = {"rows": 1, "distance": 2.2, "time": 15.0, "planned_area": 0.6}


def store(tmp_path, size=50):
    return CleaningStore(str(tmp_path), "house", history_size=size)


def record(i, outcome="finished", area=2.0, time_s=60.0, distance=10.0):
    return {"job_id": i, "started": 1_700_000_000.0 + 100 * i, "ended": 1_700_000_060.0 + 100 * i, "target": "everything",
            "outcome": outcome, "reason": "", "status": {"cleaned_area": area, "cleaning_time": time_s, "distance": distance}}


# ---------------------------------------------------------------- plans
def test_a_plan_is_saved_listed_and_loaded_with_the_same_rows(tmp_path):
    s = store(tmp_path)
    s.save_plan("Living room", JOB, PIECES, ESTIMATE, "abc123")
    [listed] = s.list_plans()
    assert listed.name == "Living room" and listed.map_name == "house" and listed.map_checksum == "abc123"
    loaded = s.load_plan("Living room")
    assert loaded.pieces == PIECES
    assert loaded.job == JOB and loaded.estimate == ESTIMATE
    assert loaded.saved > 0


def test_a_taken_name_is_refused_unless_replace(tmp_path):
    s = store(tmp_path)
    s.save_plan("Under the table", JOB, PIECES, ESTIMATE, "a")
    with pytest.raises(StoreError) as err:
        s.save_plan("Under the table", JOB, PIECES[:1], ESTIMATE, "b")
    assert err.value.name_taken
    s.save_plan("Under the table", JOB, PIECES[:1], ESTIMATE, "b", replace=True)
    assert s.load_plan("Under the table").pieces == PIECES[:1]


def test_bad_plan_names_are_refused(tmp_path):
    s = store(tmp_path)
    for name in ("", "   ", "../escape", "a/b", "x" * 41, ".hidden"):
        with pytest.raises(StoreError):
            s.save_plan(name, JOB, PIECES, ESTIMATE, "a")


def test_names_that_differ_only_in_spelling_are_different_plans(tmp_path):
    s = store(tmp_path)
    s.save_plan("a b", JOB, PIECES, ESTIMATE, "a")
    s.save_plan("a_b", JOB, PIECES, ESTIMATE, "a")
    assert sorted(p.name for p in s.list_plans()) == ["a b", "a_b"]


def test_delete_removes_a_plan(tmp_path):
    s = store(tmp_path)
    s.save_plan("Hall", JOB, PIECES, ESTIMATE, "a")
    assert s.delete_plan("Hall")
    assert s.list_plans() == [] and s.load_plan("Hall") is None
    assert not s.delete_plan("Hall")


def test_plans_belong_to_their_map(tmp_path):
    store(tmp_path).save_plan("Hall", JOB, PIECES, ESTIMATE, "a")
    assert CleaningStore(str(tmp_path), "upstairs").list_plans() == []
    assert len(store(tmp_path).list_plans()) == 1  # a new store on the same folder: still there after a restart


def test_a_plan_file_cut_short_is_skipped(tmp_path):
    s = store(tmp_path)
    s.save_plan("Hall", JOB, PIECES, ESTIMATE, "a")
    s.save_plan("Kitchen", JOB, PIECES, ESTIMATE, "a")
    path = os.path.join(s.plans_dir, sorted(os.listdir(s.plans_dir))[0])
    text = open(path).read()
    with open(path, "w") as f:
        f.write(text[: len(text) // 3])
    names = [p.name for p in s.list_plans()]
    assert len(names) == 1
    assert s.problems  # what was skipped, for the log


def test_nothing_half_written_is_left_behind(tmp_path):
    s = store(tmp_path)
    s.save_plan("Hall", JOB, PIECES, ESTIMATE, "a")
    s.add_record(record(1), b"png")
    leftovers = [f for d in (s.plans_dir, s.records_dir) for f in os.listdir(d) if f.startswith(".")]
    assert leftovers == []


# ---------------------------------------------------------------- records, history, totals
def test_a_record_is_kept_with_its_picture(tmp_path):
    s = store(tmp_path)
    rid = s.add_record(record(1), b"\x89PNG picture")
    found, picture = s.get_record(rid)
    assert found["outcome"] == "finished" and found["id"] == rid
    assert picture == b"\x89PNG picture"
    assert s.get_record("no-such-id") is None


def test_history_is_newest_first(tmp_path):
    s = store(tmp_path)
    ids = [s.add_record(record(i), b"p") for i in range(1, 4)]
    assert [r["id"] for r in s.history()] == ids[::-1]


def test_only_the_newest_records_are_kept_but_the_totals_count_all(tmp_path):
    s = store(tmp_path, size=3)
    ids = [s.add_record(record(i, area=1.5, time_s=30.0, distance=4.0), b"p") for i in range(1, 6)]
    assert [r["id"] for r in s.history()] == ids[:1:-1]
    assert len(os.listdir(s.records_dir)) == 6  # three records, three pictures
    totals = s.totals()
    assert totals == {"cleans": 5, "area": pytest.approx(7.5), "time": pytest.approx(150.0), "distance": pytest.approx(20.0)}
    # after a restart too
    assert store(tmp_path, size=3).totals() == totals


def test_a_stopped_clean_is_recorded_as_stopped(tmp_path):
    s = store(tmp_path)
    rid = s.add_record(record(1, outcome="stopped", area=0.8), b"p")
    assert s.get_record(rid)[0]["outcome"] == "stopped"
    assert s.totals()["area"] == pytest.approx(0.8)


def test_a_record_file_cut_short_is_skipped(tmp_path):
    s = store(tmp_path)
    s.add_record(record(1), b"p")
    rid = s.add_record(record(2), b"p")
    with open(os.path.join(s.records_dir, rid + ".yaml"), "w") as f:
        f.write("job_id: 2\nstarted: [")
    assert len(s.history()) == 1


def test_lost_totals_are_rebuilt_from_the_records_that_are_left(tmp_path):
    s = store(tmp_path)
    s.add_record(record(1, area=2.0), b"p")
    s.add_record(record(2, area=3.0), b"p")
    with open(s.totals_path, "w") as f:
        f.write("cleans: [")
    assert s.totals()["cleans"] == 2 and s.totals()["area"] == pytest.approx(5.0)
    assert yaml.safe_load(open(s.totals_path))["cleans"] == 2


def test_a_saved_plan_keeps_its_coordinates_exactly(tmp_path):
    # Map cell centres are not round numbers (origin -0.595887); rounding them moved cells at the edge
    # of the cleaning disc out of it, and a loaded plan cleaned less than the same plan before saving.
    exact = [Piece(ROW, (1.2291130000000001, 0.2349999999999), (3.104113, 0.2349999999999), 1, 1, 1)]
    s = store(tmp_path)
    s.save_plan("Exact", JOB, exact, ESTIMATE, "a")
    assert s.load_plan("Exact").pieces == exact
