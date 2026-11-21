import itertools

import pytest

from vacuum_mode_manager import state_machine as sm

WORKING = (sm.IDLE, sm.MAPPING, sm.NAVIGATION, sm.MANUAL, sm.CLEANING)


def make(has_map=True, faults=None, job_starting=True, exploring_starting=False, resume_problem="no map was being made"):
    box = {"map": has_map, "faults": list(faults or []), "job": job_starting,
           "exploring": exploring_starting, "resume_problem": resume_problem}
    machine = sm.build_state_machine(
        lambda: box["map"], lambda: box["faults"], lambda: box["job"],
        exploration_job_starting=lambda: box["exploring"],
        resume_mapping_problem=lambda: box["resume_problem"],
    )
    return machine, box


def test_starts_idle():
    machine, _ = make()
    assert machine.state == sm.IDLE
    assert machine.previous == ""


@pytest.mark.parametrize("source,target", [p for p in itertools.permutations(WORKING, 2) if p != (sm.MAPPING, sm.CLEANING)])
def test_every_working_mode_reaches_every_other(source, target):
    machine, _ = make(has_map=True)
    machine.set(source)
    ok, why = machine.check(target)
    assert ok, why


@pytest.mark.parametrize("target", (sm.NAVIGATION, sm.CLEANING))
@pytest.mark.parametrize("source", WORKING)
def test_navigation_and_cleaning_need_a_saved_map(source, target):
    if source == target or (source, target) == (sm.MAPPING, sm.CLEANING):
        return  # mapping -> cleaning is never allowed: test_a_job_cannot_start_from_mapping
    machine, box = make(has_map=False)
    machine.set(source)
    ok, why = machine.check(target)
    assert not ok
    assert "map" in why
    assert target not in machine.allowed()
    box["map"] = True
    assert machine.check(target)[0]


@pytest.mark.parametrize("source", WORKING)
def test_failsafe_is_reachable_from_every_mode(source):
    machine, _ = make()
    machine.set(source)
    assert machine.check(sm.FAILSAFE)[0]


def test_failsafe_only_leads_to_idle():
    machine, _ = make()
    machine.set(sm.FAILSAFE)
    assert machine.allowed() == [sm.IDLE]
    for target in (sm.MAPPING, sm.NAVIGATION, sm.MANUAL, sm.CLEANING, sm.DOCKING, sm.DOCKED, sm.FAILSAFE):
        assert not machine.check(target)[0]


@pytest.mark.parametrize("source", WORKING + (sm.DOCKED,))
def test_docking_can_be_requested_from_every_working_mode_and_needs_a_map(source):
    machine, box = make(has_map=True)
    machine.set(source)
    assert machine.check(sm.DOCKING)[0]
    box["map"] = False
    ok, why = machine.check(sm.DOCKING)
    assert not ok and "map" in why


@pytest.mark.parametrize("source", WORKING + (sm.DOCKING, sm.FAILSAFE))
def test_docked_can_never_be_requested(source):
    machine, _ = make()
    machine.set(source)
    ok, why = machine.check(sm.DOCKED)
    assert not ok
    assert sm.DOCKED not in machine.allowed()


def test_only_docking_becomes_docked_by_itself():
    machine, _ = make()
    for source in WORKING + (sm.FAILSAFE,):
        machine.set(source)
        assert not machine.is_internal(sm.DOCKED)
    machine.set(sm.DOCKING)
    assert machine.is_internal(sm.DOCKED)


@pytest.mark.parametrize("source", (sm.DOCKING, sm.DOCKED))
def test_docking_and_docked_can_be_left_for_any_working_mode_or_failsafe(source):
    machine, _ = make()
    machine.set(source)
    for target in WORKING + (sm.FAILSAFE,):
        assert machine.check(target)[0], target


def test_failsafe_cannot_be_left_while_a_fault_is_present():
    machine, box = make(faults=["lidar data stopped 3.0 s ago"])
    machine.set(sm.FAILSAFE)
    ok, why = machine.check(sm.IDLE)
    assert not ok
    assert "lidar" in why
    assert machine.allowed() == []
    box["faults"].clear()
    assert machine.check(sm.IDLE)[0]


def test_same_state_and_unknown_state_are_refused():
    machine, _ = make()
    assert not machine.check(sm.IDLE)[0]
    ok, why = machine.check("flying")
    assert not ok
    assert "unknown state" in why


def test_set_tracks_previous_state():
    machine, _ = make()
    machine.set(sm.MAPPING)
    machine.set(sm.NAVIGATION)
    assert (machine.previous, machine.state) == (sm.MAPPING, sm.NAVIGATION)
    machine.set(sm.NAVIGATION)  # no change
    assert machine.previous == sm.MAPPING


@pytest.mark.parametrize("state", WORKING + (sm.DOCKING, sm.DOCKED, sm.EXPLORING))
@pytest.mark.parametrize("has_map", [True, False])
def test_never_two_sources_of_the_map_to_odom_transform(state, has_map):
    """slam_toolbox and AMCL both publish map -> odom, so no mode may run both."""
    res = sm.resources_for(state, has_map)
    assert not (res[sm.SLAM] and res[sm.LOCALIZATION])


def test_resources_per_mode():
    def res(state, has_map):
        r = sm.resources_for(state, has_map)
        return r[sm.SLAM], r[sm.LOCALIZATION], r[sm.NAVIGATION_GROUP], r[sm.DOCKING_GROUP]

    assert res(sm.MAPPING, True) == (True, False, False, False)
    assert res(sm.NAVIGATION, True) == (False, True, True, False)
    assert res(sm.IDLE, True) == (False, True, False, False)
    assert res(sm.IDLE, False) == (False, False, False, False)
    assert res(sm.MANUAL, False) == (False, False, False, False)
    assert res(sm.DOCKING, True) == (False, True, True, True)  # Nav2 to get near, then the docking server
    assert res(sm.DOCKED, True) == (False, True, False, False)  # parked: like idle
    assert res(sm.CLEANING, True) == (False, True, True, False)  # the cleaning node drives through Nav2
    # Exploring: slam_toolbox keeps making the map while Nav2 drives; no AMCL, saved map or not.
    assert res(sm.EXPLORING, True) == (True, False, True, False)
    assert res(sm.EXPLORING, False) == (True, False, True, False)



@pytest.mark.parametrize("source", [s for s in WORKING if s not in (sm.CLEANING, sm.MAPPING)] + [sm.DOCKING, sm.DOCKED])
def test_cleaning_only_while_a_cleaning_job_is_starting(source):
    """The robot is in cleaning mode exactly while a job runs: a plain request for the mode is
    refused, with the reason, and the mode is not offered."""
    machine, box = make(has_map=True, job_starting=False)
    machine.set(source)
    ok, why = machine.check(sm.CLEANING)
    assert not ok and "cleaning job" in why
    assert sm.CLEANING not in machine.allowed()
    box["job"] = True
    assert machine.check(sm.CLEANING)[0]


def test_a_job_cannot_start_from_mapping():
    machine, _ = make(has_map=True, job_starting=True)
    machine.set(sm.MAPPING)
    ok, why = machine.check(sm.CLEANING)
    assert not ok and "save or discard" in why


# ---------------------------------------------------------------- exploring
_EXPLORING_HOW = "started by an exploration"


def test_exploring_is_entered_from_mapping_only_while_an_exploration_is_starting():
    """The robot is in exploring mode exactly while an exploration runs: a plain request is refused,
    with the reason, and the mode is not offered."""
    machine, box = make(exploring_starting=False)
    machine.set(sm.MAPPING)
    ok, why = machine.check(sm.EXPLORING)
    assert not ok and _EXPLORING_HOW in why
    assert sm.EXPLORING not in machine.allowed()
    box["exploring"] = True
    assert machine.check(sm.EXPLORING) == (True, "")
    assert sm.EXPLORING in machine.allowed()


def test_exploring_needs_no_saved_map():
    machine, _ = make(has_map=False, exploring_starting=True)
    machine.set(sm.MAPPING)
    assert machine.check(sm.EXPLORING)[0]


@pytest.mark.parametrize("source", (sm.IDLE, sm.NAVIGATION, sm.MANUAL, sm.CLEANING, sm.DOCKING, sm.DOCKED, sm.FAILSAFE))
def test_exploring_is_never_entered_from_another_mode_than_mapping(source):
    machine, _ = make(exploring_starting=True)
    machine.set(source)
    ok, why = machine.check(sm.EXPLORING)
    assert not ok and _EXPLORING_HOW in why
    assert sm.EXPLORING not in machine.allowed()


def test_exploring_is_left_for_mapping_for_a_stop_or_for_failsafe():
    machine, _ = make()
    machine.set(sm.EXPLORING)
    for target in (sm.MAPPING, sm.IDLE, sm.FAILSAFE):
        assert machine.check(target) == (True, ""), target
    assert sorted(machine.allowed()) == sorted([sm.MAPPING, sm.IDLE, sm.FAILSAFE])


@pytest.mark.parametrize("target", (sm.NAVIGATION, sm.MANUAL, sm.CLEANING, sm.DOCKING))
def test_exploring_is_not_left_for_another_working_mode(target):
    """Nothing but the end of the exploration (or a stop) takes the robot out of exploring: a
    change straight to navigation, say, would drop the map being made."""
    machine, _ = make(job_starting=True)
    machine.set(sm.EXPLORING)
    ok, why = machine.check(target)
    assert not ok and "stop exploring first" in why


def test_a_stop_while_exploring_is_carried_out_as_a_change_to_mapping():
    # idle switches slam_toolbox off, which would throw the map away; the robot keeps it instead
    assert sm.carried_out_as(sm.EXPLORING, sm.IDLE) == sm.MAPPING
    # everything else is carried out as asked
    assert sm.carried_out_as(sm.EXPLORING, sm.MAPPING) == sm.MAPPING
    assert sm.carried_out_as(sm.EXPLORING, sm.FAILSAFE) == sm.FAILSAFE
    assert sm.carried_out_as(sm.MAPPING, sm.IDLE) == sm.IDLE  # discarding a map is asked for from mapping
    assert sm.carried_out_as(sm.NAVIGATION, sm.IDLE) == sm.IDLE


@pytest.mark.parametrize("change", [(sm.EXPLORING, sm.MAPPING), (sm.MAPPING, sm.EXPLORING)])
def test_a_stop_that_waited_for_a_change_into_or_out_of_exploring_keeps_the_map(change):
    """Stop pressed just as the exploration ends arrives when the robot is already back in mapping.
    It was meant for the exploration, so it must not discard the map."""
    assert sm.carried_out_as(sm.MAPPING, sm.IDLE, waited_for=change) == sm.MAPPING
    assert sm.carried_out_as(sm.EXPLORING, sm.IDLE, waited_for=change) == sm.MAPPING


def test_a_stop_that_waited_for_another_change_is_an_ordinary_stop():
    assert sm.carried_out_as(sm.MAPPING, sm.IDLE, waited_for=(sm.IDLE, sm.MAPPING)) == sm.IDLE
    assert sm.carried_out_as(sm.NAVIGATION, sm.IDLE, waited_for=(sm.IDLE, sm.NAVIGATION)) == sm.IDLE
    # the change into exploring failed and ended somewhere else: nothing special
    assert sm.carried_out_as(sm.FAILSAFE, sm.IDLE, waited_for=(sm.MAPPING, sm.EXPLORING)) == sm.IDLE


# ---------------------------------------------------------------- a fault must not cost the map
def test_failsafe_also_leads_to_mapping_when_the_map_being_made_can_be_resumed():
    machine, box = make(resume_problem="")
    machine.set(sm.FAILSAFE)
    assert machine.check(sm.MAPPING) == (True, "")
    assert machine.allowed() == [sm.MAPPING]  # idle would lose the map: see the review findings below
    box["resume_problem"] = "the robot was lifted"
    ok, why = machine.check(sm.MAPPING)
    assert not ok and "the robot was lifted" in why
    assert machine.allowed() == [sm.IDLE]


def test_failsafe_does_not_lead_to_mapping_while_a_fault_is_present():
    machine, _ = make(faults=["lidar data stopped 3.0 s ago"], resume_problem="")
    machine.set(sm.FAILSAFE)
    ok, why = machine.check(sm.MAPPING)
    assert not ok and "lidar" in why
    assert machine.allowed() == []


@pytest.mark.parametrize("previous,lifted,slam_running,expected", [
    (sm.MAPPING, False, True, ""),
    (sm.EXPLORING, False, True, ""),
    (sm.NAVIGATION, False, True, "no map was being made"),
    (sm.IDLE, False, True, "no map was being made"),
    (sm.MAPPING, True, True, "lifted"),
    (sm.EXPLORING, False, False, "slam_toolbox"),
])
def test_when_a_map_being_made_can_be_resumed_after_a_fault(previous, lifted, slam_running, expected):
    why = sm.resume_mapping_problem(previous, lifted, slam_running)
    assert (why == "") if expected == "" else (expected in why)


@pytest.mark.parametrize("state,previous,slam_running,expected", [
    (sm.MAPPING, sm.IDLE, True, True),
    (sm.EXPLORING, sm.MAPPING, True, True),
    (sm.FAILSAFE, sm.MAPPING, True, True),     # the map is still there: Reset can go back to it
    (sm.FAILSAFE, sm.EXPLORING, True, True),
    (sm.FAILSAFE, sm.EXPLORING, False, False),  # slam_toolbox is gone, and the map with it
    (sm.FAILSAFE, sm.NAVIGATION, True, False),
    (sm.IDLE, sm.MAPPING, False, False),
    (sm.NAVIGATION, sm.MAPPING, False, False),
    (sm.CLEANING, sm.IDLE, False, False),
])
def test_making_map_says_whether_a_map_is_being_made(state, previous, slam_running, expected):
    assert sm.making_map(state, previous, slam_running) is expected


# ---------------------------------------------------------------- requests that wait instead of "busy"
@pytest.mark.parametrize("current", [sm.IDLE, sm.MAPPING, sm.EXPLORING, sm.NAVIGATION, sm.FAILSAFE])
def test_stops_wait_for_a_change_in_progress(current):
    assert sm.waits_when_busy(current, sm.IDLE)
    assert sm.waits_when_busy(current, sm.FAILSAFE)


def test_a_reset_to_mapping_waits_like_a_reset_to_idle():
    """Failsafe is announced at once, then Nav2 is switched off, which takes a few seconds. A Reset
    pressed meanwhile must not be refused as busy, whether it goes to idle or back to mapping."""
    assert sm.waits_when_busy(sm.FAILSAFE, sm.MAPPING)


@pytest.mark.parametrize("current,target", [(sm.IDLE, sm.MAPPING), (sm.MAPPING, sm.EXPLORING), (sm.IDLE, sm.NAVIGATION)])
def test_other_requests_are_busy_during_a_change(current, target):
    assert not sm.waits_when_busy(current, target)


# ---------------------------------------------------------------- review findings, 2026-11-19
def test_a_stop_in_failsafe_keeps_a_map_that_can_be_resumed():
    """A Stop that waited for a change that failed into failsafe, or one sent by a phone with an old
    state, must not take failsafe to idle: idle switches slam_toolbox off and loses the map. The robot
    is already standing still in failsafe; it stays there, and Reset goes back to mapping."""
    assert sm.carried_out_as(sm.FAILSAFE, sm.IDLE, waited_for=(sm.MAPPING, sm.EXPLORING), map_resumable=True) == sm.FAILSAFE
    assert sm.carried_out_as(sm.FAILSAFE, sm.IDLE, map_resumable=True) == sm.FAILSAFE
    assert sm.carried_out_as(sm.FAILSAFE, sm.IDLE, map_resumable=False) == sm.IDLE  # no map to keep: an ordinary Reset
    assert sm.carried_out_as(sm.FAILSAFE, sm.MAPPING, map_resumable=True) == sm.MAPPING


def test_failsafe_with_a_map_that_can_be_resumed_offers_only_mapping():
    """Discarding the map is done from mapping (Discard), never as a side effect of leaving failsafe."""
    machine, _ = make(resume_problem="")
    machine.set(sm.FAILSAFE)
    assert machine.allowed() == [sm.MAPPING]
    ok, why = machine.check(sm.IDLE)
    assert not ok and "kept" in why


@pytest.mark.parametrize("slam_on,fresh,expected", [
    (True, True, True),
    (False, True, False),  # AMCL's transform, after a failed change out of mapping: no map being made
    (True, False, False),  # slam_toolbox switched on but silent: the map is gone
])
def test_the_map_being_made_is_alive_only_while_slam_toolbox_is_on_and_publishing(slam_on, fresh, expected):
    assert sm.slam_map_alive(slam_on, fresh) is expected
