"""What a job asks for: checked, turned into planner options, kept as plain data."""

import math

import pytest

from vacuum_coverage.jobspec import AREA, PLAN, SPOT, JobSpec
from vacuum_coverage.planner import Options


def test_everything_with_the_defaults():
    job = JobSpec()
    assert job.check() is None
    assert job.target == "everything"
    assert job.options(edges_by_default=False) == Options(direction="auto", passes=1, edges=False)
    assert job.options(edges_by_default=True).edges is True
    assert job.return_to_dock and job.area_polygon(1.5) is None


def test_an_area_is_its_polygon_and_a_spot_a_square():
    area = JobSpec(what=AREA, area=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    assert area.check() is None and area.target == "area"
    assert area.area_polygon(1.5) == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    spot = JobSpec(what=SPOT, spot=(2.0, 1.0))
    assert spot.target == "spot"
    assert spot.area_polygon(1.0) == [(1.5, 0.5), (2.5, 0.5), (2.5, 1.5), (1.5, 1.5)]


@pytest.mark.parametrize("job,words", [
    (JobSpec(what=AREA, area=[(0.0, 0.0), (1.0, 0.0)]), "corners"),
    (JobSpec(what=AREA, area=[(0.0, 0.0), (math.nan, 0.0), (1.0, 1.0)]), "number"),
    (JobSpec(what=AREA, area=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]), "no floor"),
    (JobSpec(what=SPOT, spot=(math.inf, 0.0)), "number"),
    (JobSpec(what=PLAN, plan_name="  "), "name"),
    (JobSpec(direction="diagonal"), "direction"),
    (JobSpec(passes=3), "passes"),
    (JobSpec(what="wherever"), "what"),
])
def test_bad_jobs_are_refused_with_a_reason(job, words):
    why = job.check()
    assert why is not None and words in why


def test_rows_may_be_asked_for_at_an_angle():
    assert JobSpec(direction="45").check() is None
    assert JobSpec(direction="37.5").options(edges_by_default=False).direction == "37.5"
    assert JobSpec.from_dict(JobSpec(direction="120").to_dict()).direction == "120"


def test_a_job_survives_a_round_trip_through_plain_data():
    job = JobSpec(what=AREA, area=[(0.5, 0.5), (2.0, 0.5), (2.0, 1.5)], direction="y", passes=2, edges=True, return_to_dock=False)
    assert JobSpec.from_dict(job.to_dict()) == job
    plan = JobSpec(what=PLAN, plan_name="Living room")
    assert JobSpec.from_dict(plan.to_dict()) == plan
    assert plan.target == "Living room"


def test_a_saved_plan_runs_with_its_own_area_and_options_and_the_callers_docking():
    saved = JobSpec(what=SPOT, spot=(1.0, 2.0), passes=2, edges=True)
    asked = JobSpec(what=PLAN, plan_name="Hall", return_to_dock=False)
    run = asked.with_saved(saved)
    assert run.what == SPOT and run.spot == (1.0, 2.0) and run.passes == 2 and run.edges is True
    assert run.plan_name == "Hall" and run.return_to_dock is False
    assert run.target == "Hall"


def test_a_room_job_names_the_room_and_takes_its_outline_once_looked_up():
    from vacuum_coverage.jobspec import ROOM
    job = JobSpec(what=ROOM, room="Kitchen")
    assert job.check() is None and job.target == "Kitchen" and job.area_polygon(1.5) is None
    assert "no room name" in JobSpec(what=ROOM, room=" ").check()
    looked_up = JobSpec(what=ROOM, room="Kitchen", area=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    assert looked_up.area_polygon(1.5) == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    assert JobSpec.from_dict(looked_up.to_dict()) == looked_up
