"""The exploration's course, against a fake robot and a fake explorer. No ROS.

The fake world keeps two clocks (robot time and wall time) that only move when the runner
sleeps, and a script that makes things happen at given robot times.
"""

import threading

import pytest

from vacuum_exploration import runner as rn


class FakeExplorer:
    def __init__(self, world, index):
        self.world = world
        self.index = index
        self.started = world.t
        self.exit = None
        self.answer = world.w
        self.done = False
        self.ended = False

    def exit_code(self):
        return self.exit

    def last_answer(self):
        return self.answer

    def finished(self):
        return self.done

    def end(self):
        self.ended = True
        return "asked to stop; it quit"


class FakeWorld:
    def __init__(self, mode="idle", script=None):
        self.t = 100.0  # robot time
        self.w = 5000.0  # wall time
        self.mode_state = mode
        self.asked = []
        self.refuse = {}
        self.ready_problem = ""
        self.explorers = []
        self.goals = 0
        self.active = False
        self.navigated = []
        self.nav_ok = True
        self.cancels = 0
        self.script = script or (lambda world: None)
        self.where = (1.0, 2.0, 0.5)

    # --- what the runner uses
    def mode(self):
        return self.mode_state, False

    def ask_mode(self, mode):
        self.asked.append(mode)
        if mode in self.refuse:
            return False, self.refuse[mode]
        self.mode_state = mode
        return True, f"now in {mode}"

    def wait_ready(self, abort):
        return self.ready_problem

    def pose(self):
        return self.where

    def now(self):
        return self.t

    def wall(self):
        return self.w

    def start_explorer(self):
        ex = FakeExplorer(self, len(self.explorers))
        self.explorers.append(ex)
        return ex

    def goals_seen(self):
        return self.goals

    def goal_active(self):
        return self.active

    def navigate(self, x, y, yaw, abort):
        self.navigated.append((x, y, yaw))
        return self.nav_ok

    def cancel_goals(self):
        self.cancels += 1

    def sleep(self, seconds):
        self.t += seconds
        self.w += seconds
        for ex in self.explorers:  # a live explorer answers its state requests
            if ex.exit is None and not getattr(ex, "silent", False):
                ex.answer = self.w
        self.script(self)

    # --- helpers for scripts
    def current(self):
        return self.explorers[-1] if self.explorers else None

    def since_start(self):
        ex = self.current()
        return self.t - ex.started if ex else 0.0


SETTINGS = rn.Settings(time_limit=1200.0, quiet_limit=30.0, answer_timeout=5.0, startup_timeout=15.0, tick=0.5)


def run(world, settings=SETTINGS):
    r = rn.ExplorationRunner(world, settings)
    outcome, message = r.run()
    return r, outcome, message


def explores_then_finishes(goal_from=5.0, goal_until=40.0, finish_at=45.0, second_finish_after=8.0):
    """The first explorer drives one goal, then says finished; every later one says finished
    `second_finish_after` s after it starts, without a goal: nothing is left."""
    def script(world):
        ex = world.current()
        if ex is None:
            return
        if ex.index == 0:
            age = world.since_start()
            if goal_from <= age < goal_until and not world.active:
                world.goals += 1
                world.active = True
            if age >= goal_until:
                world.active = False
            if age >= finish_at:
                ex.done = True
        elif world.since_start() >= second_finish_after:
            ex.done = True
    return script


# ---------------------------------------------------------------- the whole course
def test_from_idle_a_new_map_is_begun_explored_confirmed_and_the_robot_comes_back():
    world = FakeWorld("idle", explores_then_finishes())
    r, outcome, message = run(world)
    assert outcome == rn.FINISHED
    assert world.asked == ["mapping", "exploring", "mapping"]  # a new map, exploring, and back to mapping
    assert len(world.explorers) == 2  # the second look
    assert all(ex.ended for ex in world.explorers)
    assert world.navigated == [world.where]  # back to where it started
    assert "nothing left to look at" in message and "back where it started" in message
    assert r.state == rn.FINISHED
    assert r.goals_sent == 1


def test_from_mapping_the_map_being_made_is_continued():
    world = FakeWorld("mapping", explores_then_finishes())
    _, outcome, _ = run(world)
    assert outcome == rn.FINISHED
    assert world.asked == ["exploring", "mapping"]


@pytest.mark.parametrize("mode", ["navigation", "cleaning", "manual", "docking", "failsafe", "exploring"])
def test_not_started_from_other_modes(mode):
    world = FakeWorld(mode)
    _, outcome, message = run(world)
    assert outcome == rn.FAILED and "stop what the robot is doing first" in message
    assert world.asked == [] and world.explorers == []


def test_a_refused_mode_ends_it_as_failed_with_the_reason():
    world = FakeWorld("mapping")
    world.refuse["exploring"] = "still faulty: lidar"
    _, outcome, message = run(world)
    assert outcome == rn.FAILED
    assert "did not switch to exploring" in message and "still faulty: lidar" in message
    assert world.explorers == []
    assert world.asked == ["exploring"]  # still in mapping: nothing to hand back


def test_navigation_not_ready_ends_it_and_goes_back_to_mapping():
    world = FakeWorld("mapping")
    world.ready_problem = "navigation did not start publishing its costmaps"
    _, outcome, message = run(world)
    assert outcome == rn.FAILED and "costmaps" in message
    assert world.explorers == []
    assert world.asked == ["exploring", "mapping"]


# ---------------------------------------------------------------- finished, in its three forms
def test_a_finished_that_the_second_look_overturns_goes_on():
    def script(world):
        ex = world.current()
        age = world.since_start()
        if ex.index == 0 and age >= 20.0:
            ex.done = True  # the costmap was behind: a frontier at the edge was hidden
        if ex.index == 1:
            if 8.0 <= age < 30.0 and not world.active:
                world.goals += 1
                world.active = True
            if age >= 30.0:
                world.active = False
            if age >= 35.0:
                ex.done = True
        if ex.index == 2 and age >= 8.0:
            ex.done = True

    world = FakeWorld("mapping", script)
    r, outcome, message = run(world)
    assert outcome == rn.FINISHED and "nothing left to look at" in message
    assert len(world.explorers) == 3
    assert r.goals_sent == 1


def test_a_quiet_spell_means_nothing_reachable_is_left():
    def script(world):
        age = world.since_start()
        if 5.0 <= age < 25.0 and not world.active:
            world.goals += 1
            world.active = True
        if age >= 25.0:
            world.active = False  # every place left was given up on: no goal, and no "finished"

    world = FakeWorld("mapping", script)
    r, outcome, message = run(world)
    assert outcome == rn.FINISHED and "could not be reached" in message
    assert len(world.explorers) == 1  # no second look: a fresh explorer would only try them again
    assert 25.0 + 30.0 <= world.t - 100.0 <= 25.0 + 30.0 + 1.0
    assert world.navigated == [world.where]


def test_the_quiet_spell_counts_from_the_explorers_start():
    world = FakeWorld("mapping")  # an explorer that never sends a goal and never says finished
    _, outcome, message = run(world)
    assert outcome == rn.FINISHED and "could not be reached" in message
    assert 30.0 <= world.t - 100.0 <= 31.0


def test_the_time_limit_ends_it_early_with_the_map_kept():
    def script(world):
        if not world.active:
            world.goals += 1
            world.active = True  # always busy: an open space, or a loop

    world = FakeWorld("mapping", script)
    r, outcome, message = run(world, rn.Settings(time_limit=120.0, tick=0.5))
    assert outcome == rn.FINISHED and "time limit" in message
    assert 120.0 <= world.t - 100.0 <= 121.0
    assert world.asked[-1] == "mapping"
    assert world.navigated == [world.where]


# ---------------------------------------------------------------- the explorer misbehaving
def test_an_explorer_that_exits_ends_it_as_failed():
    def script(world):
        if world.since_start() >= 12.0:
            world.current().exit = -9

    world = FakeWorld("mapping", script)
    _, outcome, message = run(world)
    assert outcome == rn.FAILED and "the explorer stopped running" in message and "-9" in message
    assert world.navigated == []  # no trip back after a failure
    assert world.asked[-1] == "mapping"


def test_an_explorer_that_stops_answering_ends_it_as_failed():
    def script(world):
        if world.since_start() >= 12.0:
            world.current().silent = True  # frozen: alive, but no answer

    world = FakeWorld("mapping", script)
    _, outcome, message = run(world)
    assert outcome == rn.FAILED and "stopped answering" in message
    assert 12.0 + 5.0 <= world.t - 100.0 <= 12.0 + 5.0 + 1.0
    assert world.current().ended


def test_an_explorer_that_never_answers_ends_it_as_failed():
    world = FakeWorld("mapping")
    world.start_explorer_orig = world.start_explorer

    def start_silent():
        ex = world.start_explorer_orig()
        ex.answer = None
        ex.silent = True
        return ex

    world.start_explorer = start_silent
    _, outcome, message = run(world)
    assert outcome == rn.FAILED and "did not answer" in message
    assert 15.0 <= world.t - 100.0 <= 16.0


# ---------------------------------------------------------------- stopping, and the trip back
def test_a_stop_ends_the_explorer_and_does_not_drive_back():
    world = FakeWorld("mapping")
    r = rn.ExplorationRunner(world, SETTINGS)

    def script(w):
        if w.since_start() >= 10.0 and r.stop_reason is None:
            w.mode_state = "mapping"  # Stop: the mode manager has already gone back to mapping
            r.stop("the robot left exploring mode (now mapping)")

    world.script = script
    outcome, message = r.run()
    assert outcome == rn.STOPPED and "left exploring mode" in message
    assert world.current().ended
    assert world.navigated == []
    assert world.asked == ["exploring"]  # already in mapping: nothing to ask for
    assert world.cancels >= 1


def test_a_stop_reported_as_a_failure_ends_it_as_failed():
    world = FakeWorld("mapping")
    r = rn.ExplorationRunner(world, SETTINGS)

    def script(w):
        if w.since_start() >= 10.0 and r.stop_reason is None:
            w.mode_state = "failsafe"
            r.stop("the robot went to failsafe: lidar data stopped", failed=True)

    world.script = script
    outcome, message = r.run()
    assert outcome == rn.FAILED and "failsafe" in message
    assert world.asked == ["exploring"]  # never asks a robot in failsafe for a mode


def test_a_trip_back_that_fails_still_finishes():
    world = FakeWorld("mapping", explores_then_finishes())
    world.nav_ok = False
    _, outcome, message = run(world)
    assert outcome == rn.FINISHED and "could not get back to where it started" in message
    assert world.asked[-1] == "mapping"


def test_status_while_it_runs():
    seen = []
    world = FakeWorld("idle")
    r = rn.ExplorationRunner(world, SETTINGS)
    base = explores_then_finishes()

    def script(w):
        base(w)
        seen.append((r.state, r.message))

    world.script = script
    r.run()
    states = [s for s, _ in seen]
    assert rn.EXPLORING in states
    assert any("looking again" in m for s, m in seen if s == rn.EXPLORING)  # the second look is visible
    assert r.state == rn.FINISHED
    assert r.exploring_time > 0


def test_stop_is_safe_from_another_thread_before_the_run():
    world = FakeWorld("mapping")
    r = rn.ExplorationRunner(world, SETTINGS)
    t = threading.Thread(target=r.stop, args=("stopped before it began",))
    t.start()
    t.join()
    outcome, message = r.run()
    assert outcome == rn.STOPPED and world.explorers == []
