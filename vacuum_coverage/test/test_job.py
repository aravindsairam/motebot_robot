"""The piece-by-piece driving logic against a fake Nav2: every failure path of section 7.4,
pause and resume, and stopping, without the simulator."""

import threading
import time

import pytest

from vacuum_coverage.job import CANCELLED, FAILED, FINISHED, STOPPED, SUCCEEDED, JobRunner, RunSettings, time_left
from vacuum_coverage.planner import CONNECTOR, EDGE, ROW, TRANSIT, Piece, Settings


def zigzag(rows=4, length=2.0, spacing=0.25):
    """A transit from (0, 0) to the first row, then rows joined by connectors."""
    pieces = [Piece(TRANSIT, (0.0, 0.0), (0.5, 0.5))]
    y = 0.5
    for i in range(rows):
        a, b = (0.5, 2.5) if i % 2 == 0 else (2.5, 0.5)
        if i:
            pieces.append(Piece(CONNECTOR, (a, y - spacing), (a, y), pass_number=1, region=1))
        pieces.append(Piece(ROW, (a, y), (b, y), i + 1, 1, 1))
        y += spacing
    return pieces


class FakeNav:
    """Drives instantly to the end of each goal, unless told otherwise.

    fail: {call number: how far along (0-1) the robot gets before the goal fails}
    block: call numbers that wait until aborted, leaving the robot half way
    """

    def __init__(self, fail=None, block=(), start=(0.0, 0.0)):
        self.position = start
        self.calls = []
        self.cleared = 0
        self.fail = dict(fail or {})
        self.block = set(block)
        self.started = threading.Event()
        self.ready_flag = True

    def pose(self):
        return (self.position[0], self.position[1], 0.0)

    def ready(self):
        return self.ready_flag

    def _drive(self, kind, start, end, abort):
        n = len(self.calls)
        self.calls.append((kind, start, end))
        if n in self.block:
            self.started.set()
            abort.wait(5.0)
            self.position = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            return CANCELLED
        if n in self.fail:
            f = self.fail[n]
            self.position = (start[0] + (end[0] - start[0]) * f, start[1] + (end[1] - start[1]) * f)
            return FAILED
        self.position = end
        return SUCCEEDED

    def follow_line(self, start, end, abort):
        return self._drive("line", start, end, abort)

    def navigate(self, x, y, yaw, abort):
        return self._drive("nav", self.position, (x, y), abort)

    def clear_local_costmap(self):
        self.cleared += 1


FAST = RunSettings(failure_settle=0.0)


def run(pieces, nav, settings=FAST):
    runner = JobRunner(pieces, nav, settings)
    outcome, reason = runner.run()
    return runner, outcome, reason


def test_every_piece_is_driven_in_order():
    pieces = zigzag(3)
    runner, outcome, reason = run(pieces, FakeNav())
    assert outcome == FINISHED and reason == ""
    kinds = [c[0] for c in runner.driver.calls]
    assert kinds == ["nav", "line", "line", "line", "line", "line"]
    assert runner.rows_done == 3 and runner.rows_skipped == 0
    assert runner.driver.calls[1][1:] == (pieces[1].start, pieces[1].end)


def test_a_piece_starts_from_where_the_robot_is():
    pieces = [Piece(ROW, (0.0, 0.0), (2.0, 0.0), 1, 1, 1)]
    nav = FakeNav(start=(0.6, 0.04))  # 4 cm beside the row, 60 cm along it
    run(pieces, nav)
    assert nav.calls == [("line", (0.6, 0.0), (2.0, 0.0))]


def test_a_robot_far_from_its_row_goes_back_to_it_first():
    pieces = [Piece(ROW, (0.0, 0.0), (2.0, 0.0), 1, 1, 1)]
    nav = FakeNav(start=(1.0, 0.8))
    run(pieces, nav)
    assert [c[0] for c in nav.calls] == ["nav", "line"]
    assert nav.calls[0][2] == (1.0, 0.0) and nav.calls[1][1] == (1.0, 0.0)


def test_a_failed_piece_is_tried_again_after_clearing_the_costmap():
    nav = FakeNav(fail={1: 0.5})  # the first row stops half way
    runner, outcome, _ = run(zigzag(2), nav)
    assert outcome == FINISHED
    assert nav.cleared == 1 and runner.rows_skipped == 0 and runner.rows_done == 2
    assert nav.calls[2][1] == (1.5, 0.5)  # the second try continues from where the robot stopped


def test_a_failed_piece_ending_near_its_end_counts_as_done():
    nav = FakeNav(fail={1: 0.98})  # 4 cm short of the end
    runner, outcome, _ = run(zigzag(2), nav)
    assert outcome == FINISHED and nav.cleared == 0 and runner.rows_done == 2


def test_a_row_blocked_twice_is_skipped_and_the_next_row_reached_by_a_transit():
    nav = FakeNav(fail={1: 0.3, 2: 0.0})
    runner, outcome, _ = run(zigzag(3), nav)
    assert outcome == FINISHED
    assert runner.rows_skipped == 1 and runner.rows_done == 2
    # the connector after the skipped row became a transit to the start of the next row
    assert nav.calls[3][0] == "nav" and nav.calls[3][2] == (2.5, 0.75)


def test_a_skipped_row_is_logged_with_its_number_place_and_why():
    logged = []
    nav = FakeNav(fail={1: 0.3, 2: 0.0})
    runner = JobRunner(zigzag(3), nav, FAST, log=logged.append)
    assert runner.run()[0] == FINISHED
    assert len(logged) == 1
    assert logged[0].startswith("skipped row 1 (0.50, 0.50) -> (2.50, 0.50), 2.00 m: ")
    assert "ended" in logged[0] and "m from its end" in logged[0]


def test_three_rows_blocked_one_after_the_other_fail_the_job():
    nav = FakeNav(fail={i: 0.0 for i in range(1, 40)})
    nav.fail.pop(3, None)  # the transits between rows succeed
    nav.fail.pop(6, None)
    runner, outcome, reason = run(zigzag(5), nav)
    assert outcome == FAILED and "blocked" in reason
    assert runner.rows_skipped == 3


def test_a_transit_that_fails_twice_fails_the_job():
    nav = FakeNav(fail={0: 0.2, 1: 0.1})
    runner, outcome, reason = run(zigzag(2), nav)
    assert outcome == FAILED and "could not get to" in reason
    assert len(nav.calls) == 2 and nav.cleared == 1


def test_a_transit_that_fails_once_is_tried_again():
    # Nav2's costmaps can still be filling when the first trip of a job is sent.
    nav = FakeNav(fail={0: 0.2})
    runner, outcome, _ = run(zigzag(2), nav)
    assert outcome == FINISHED and nav.cleared == 1 and [c[0] for c in nav.calls[:2]] == ["nav", "nav"]


def test_a_quick_pause_and_resume_is_not_a_failed_piece():
    nav = FakeNav(block={1})
    runner = JobRunner(zigzag(2), nav, RunSettings(failure_settle=0.2))
    result = {}
    t = threading.Thread(target=lambda: result.update(zip(("outcome", "reason"), runner.run())))
    t.start()
    assert nav.started.wait(2.0)
    runner.pause()
    runner.resume()  # before the runner has even seen the pause
    t.join(3.0)
    assert result["outcome"] == FINISHED
    assert nav.cleared == 0 and runner.rows_skipped == 0


def test_a_failed_edge_is_skipped_like_a_row():
    pieces = [Piece(EDGE, (0.0, 0.0), (1.0, 0.0), 1), Piece(CONNECTOR, (1.0, 0.0), (1.0, 0.3)), Piece(EDGE, (1.0, 0.3), (1.0, 1.0), 2)]
    nav = FakeNav(fail={0: 0.0, 1: 0.0})
    runner, outcome, _ = run(pieces, nav)
    assert outcome == FINISHED and runner.edges_skipped == 1 and runner.edges_done == 1


def test_pause_cancels_the_piece_and_resume_continues_the_same_row():
    pieces = zigzag(2)
    nav = FakeNav(block={1})
    runner = JobRunner(pieces, nav, FAST)
    result = {}
    t = threading.Thread(target=lambda: result.update(zip(("outcome", "reason"), runner.run())))
    t.start()
    assert nav.started.wait(2.0)
    assert runner.pause()
    time.sleep(0.1)
    assert runner.paused and t.is_alive()
    calls_while_paused = len(nav.calls)
    time.sleep(0.2)
    assert len(nav.calls) == calls_while_paused  # nothing is sent while paused
    assert runner.resume()
    t.join(2.0)
    assert result["outcome"] == FINISHED
    assert nav.calls[2] == ("line", (1.5, 0.5), (2.5, 0.5))  # from where it stopped to the same row's end
    assert runner.rows_done == 2 and runner.pauses == 1
    assert sum(1 for c in nav.calls if c[0] == "line" and c[2] == (2.5, 0.5)) == 2  # no finished row again


def test_stop_while_paused_ends_the_job_as_stopped():
    nav = FakeNav(block={1})
    runner = JobRunner(zigzag(3), nav, FAST)
    result = {}
    t = threading.Thread(target=lambda: result.update(zip(("outcome", "reason"), runner.run())))
    t.start()
    assert nav.started.wait(2.0)
    runner.pause()
    time.sleep(0.05)
    runner.stop("stop requested")
    t.join(2.0)
    assert result == {"outcome": STOPPED, "reason": "stop requested"}


def test_stop_mid_row_ends_at_once_and_drives_nothing_more():
    nav = FakeNav(block={1})
    runner = JobRunner(zigzag(3), nav, FAST)
    result = {}
    t = threading.Thread(target=lambda: result.update(zip(("outcome", "reason"), runner.run())))
    t.start()
    assert nav.started.wait(2.0)
    runner.stop("the robot left cleaning mode (now idle)")
    t.join(2.0)
    assert result["outcome"] == STOPPED and "idle" in result["reason"]
    assert len(nav.calls) == 2


def test_a_failure_while_the_mode_is_changing_waits_for_the_stop():
    # Nav2 being switched off makes the goal fail just before the node notices the mode change.
    nav = FakeNav(fail={1: 0.5})
    nav.ready_flag = False
    runner = JobRunner(zigzag(3), nav, RunSettings(failure_settle=0.05, not_ready_wait=2.0))
    threading.Timer(0.2, runner.stop, args=("the robot left cleaning mode (now idle)",)).start()
    outcome, reason = runner.run()
    assert outcome == STOPPED and nav.cleared == 0 and runner.rows_skipped == 0


def test_no_position_fails_the_job():
    class Lost(FakeNav):
        def pose(self):
            return None

    runner, outcome, reason = run([Piece(ROW, (0.0, 0.0), (1.0, 0.0), 1, 1, 1)], Lost(), RunSettings(failure_settle=0.0, pose_wait=0.1))
    assert outcome == FAILED and "where it is" in reason


def test_progress_names_the_current_row():
    pieces = zigzag(3)
    runner = JobRunner(pieces, FakeNav(), FAST)
    assert runner.rows_total == 3 and runner.current_row == 1
    runner.run()
    assert runner.current_row == 0  # finished


def test_what_is_left_starts_at_the_robot():
    pieces = zigzag(2)
    nav = FakeNav(block={1})
    runner = JobRunner(pieces, nav, FAST)
    t = threading.Thread(target=runner.run)
    t.start()
    assert nav.started.wait(2.0)
    runner.pause()
    time.sleep(0.05)
    left = runner.remaining()
    assert left[0].kind == ROW and left[0].start == (1.5, 0.5) and left[0].end == (2.5, 0.5)
    assert [p.kind for p in left[1:]] == [CONNECTOR, ROW]
    runner.stop("done")
    t.join(2.0)


def test_time_left_uses_the_speed_and_turn_time_measured_so_far():
    s = Settings()
    # early on: the planned speed and turn time
    assert time_left(10.0, 4, 0.5, 5.0, 0.0, 0, s) == pytest.approx(10.0 / s.cleaning_speed + 4 * s.turn_time)
    # measured: 20 m in 150 s of which 50 s were 10 turns -> 0.2 m/s and 5 s per turn
    assert time_left(10.0, 4, 20.0, 150.0, 50.0, 10, s) == pytest.approx(10.0 / 0.2 + 4 * 5.0)
