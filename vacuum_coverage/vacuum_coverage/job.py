"""Driving a cleaning plan, one piece at a time. No ROS imports: Nav2 is reached through a Driver,
which the cleaning node implements with Nav2's actions and the tests with a fake.

Rules (docs/2026-11-18-coverage-cleaning-plan.md, section 7.4):
  * One goal per straight piece. Rows, connectors and edges go to the controller as a straight
    line; transits go to Nav2's navigate_to_pose, which plans around furniture.
  * Every piece starts from where the robot is now: a resumed or retried row is sent from the
    robot's place on the row to the row's end. A robot far from the line goes back to it first.
  * When a piece fails: if the robot is within the goal tolerance of its end, carry on; otherwise
    clear the local costmap and try once more; if that fails too, the rest of the row (or edge) is
    skipped and counted, and the next row is reached by a transit. Three rows skipped one after
    the other, or a transit that fails twice, end the job as failed. A goal called off by a pause
    that was resumed at once is simply sent again.
  * Pause cancels the current goal and keeps the place. Resume continues the same piece from where
    the robot is. Finished rows are never driven again. Stop ends the job.
"""

import math
import threading
from dataclasses import dataclass
from typing import Callable, List, Optional, Protocol, Tuple

from .planner import CONNECTOR, EDGE, ROW, TRANSIT, Piece, Settings, count_turns

# What a goal sent to Nav2 came to
SUCCEEDED, FAILED, CANCELLED = "succeeded", "failed", "cancelled"
# How a job ends
FINISHED, STOPPED = "finished", "stopped"  # (FAILED as above)

_TURN = math.radians(30.0)


class Driver(Protocol):
    def pose(self) -> Optional[Tuple[float, float, float]]:
        """Where the robot is on the map (x, y, heading), or None if it does not know."""

    def ready(self) -> bool:
        """Is the robot in cleaning mode, with no mode change under way?"""

    def follow_line(self, start, end, abort: threading.Event) -> str:
        """Drive the straight line start -> end. Blocks; returns SUCCEEDED, FAILED, or CANCELLED once
        `abort` is set."""

    def navigate(self, x: float, y: float, yaw: float, abort: threading.Event) -> str:
        """Go to the pose by a route Nav2 plans. Blocks, like follow_line."""

    def clear_local_costmap(self) -> None:
        ...


@dataclass(frozen=True)
class RunSettings:
    goal_tolerance: float = 0.10  # m: a piece that fails this close to its end counts as done
    rejoin_distance: float = 0.30  # m: farther from the line than this, go back to it by a transit first
    min_drive: float = 0.03  # m: less than this left of a piece counts as done
    max_skips_in_a_row: int = 3  # this many rows skipped one after the other end the job
    failure_settle: float = 1.0  # s to wait after a failure for a stop that may have caused it
    not_ready_wait: float = 30.0  # s to wait for a mode change to finish before giving up
    pose_wait: float = 3.0  # s to wait for the robot's position


class _Interrupted(Exception):
    """Pause or stop: leave the current piece where it is."""


class JobRunner:
    def __init__(self, pieces: List[Piece], driver: Driver, settings: RunSettings = RunSettings(),
                 log: Optional[Callable[[str], None]] = None):
        self.pieces = list(pieces)
        self.driver = driver
        self.settings = settings
        self._log = log or (lambda text: None)  # warnings for the robot's log: a skipped row and why
        self._why = ""  # how the last try at the current piece failed
        self.index = 0
        self.rows_total = sum(1 for p in pieces if p.kind == ROW)
        self.edges_total = sum(1 for p in pieces if p.kind == EDGE)
        self.rows_done = self.rows_skipped = 0
        self.edges_done = self.edges_skipped = 0
        self.pauses = 0
        self.turns_done = 0
        self._skips_in_a_row = 0
        self._last_heading: Optional[float] = None
        self._lock = threading.Lock()
        self._abort = threading.Event()  # set to call off the goal being driven
        self._wake = threading.Event()  # set when a pause ends (resume or stop)
        self._paused = False
        self._stop_reason: Optional[str] = None
        self._running = False

    # ------------------------------------------------------------------ control, from any thread
    @property
    def paused(self) -> bool:
        return self._paused

    def pause(self) -> bool:
        with self._lock:
            if self._stop_reason is not None or self.index >= len(self.pieces):
                return False
            if not self._paused:
                self._paused = True
                self.pauses += 1
                self._abort.set()
            return True

    def resume(self) -> bool:
        with self._lock:
            if not self._paused or self._stop_reason is not None:
                return False
            self._paused = False
            self._abort.clear()
            self._wake.set()
            return True

    def stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_reason is None:
                self._stop_reason = reason
            self._abort.set()
            self._wake.set()

    # ------------------------------------------------------------------ progress
    @property
    def current_piece(self) -> Optional[Piece]:
        return self.pieces[self.index] if self.index < len(self.pieces) else None

    @property
    def current_row(self) -> int:
        """The row being driven, or the next one; 0 when no rows are left."""
        for p in self.pieces[self.index:]:
            if p.kind == ROW:
                return p.number
        return 0

    @property
    def current_edge(self) -> int:
        for p in self.pieces[self.index:]:
            if p.kind == EDGE:
                return p.number
        return 0

    def remaining(self) -> List[Piece]:
        """The pieces still to drive, the current one from the robot's place on it."""
        left = self.pieces[self.index:]
        if not left:
            return []
        first = left[0]
        pose = self.driver.pose()
        if pose is not None and first.kind != TRANSIT:
            start = _project((pose[0], pose[1]), first)
            first = Piece(first.kind, start, first.end, first.number, first.pass_number, first.region)
        elif pose is not None:
            first = Piece(TRANSIT, (pose[0], pose[1]), first.end)
        return [first] + left[1:]

    # ------------------------------------------------------------------ the job
    def run(self) -> Tuple[str, str]:
        """Drive the plan. Blocks until it ends: (FINISHED, ""), (STOPPED, reason) or (FAILED, reason)."""
        while True:
            with self._lock:
                if self._stop_reason is not None:
                    return STOPPED, self._stop_reason
                waiting = self._paused
                if waiting:
                    self._wake.clear()
            if waiting:
                self._wake.wait()
                continue
            if self.index >= len(self.pieces):
                return FINISHED, ""
            piece = self.pieces[self.index]
            try:
                done = self._drive_piece(piece)
            except _Interrupted:
                continue
            except _JobFailed as exc:
                return FAILED, str(exc)
            if done:
                self._finished(piece)
                self.index += 1
            else:
                failed = self._skip(piece)
                if failed:
                    return FAILED, failed

    def _finished(self, piece: Piece) -> None:
        if piece.kind == ROW:
            self.rows_done += 1
            self._skips_in_a_row = 0
        elif piece.kind == EDGE:
            self.edges_done += 1
            self._skips_in_a_row = 0

    def _skip(self, piece: Piece) -> Optional[str]:
        """The piece could not be driven. Returns why the job fails, or None to carry on."""
        if piece.kind == CONNECTOR:
            # Reach the next row another way: Nav2 plans a route there.
            self.pieces[self.index] = Piece(TRANSIT, piece.start, piece.end)
            return None
        if piece.kind == ROW:
            self.rows_skipped += 1
        else:
            self.edges_skipped += 1
        self._skips_in_a_row += 1
        (x0, y0), (x1, y1) = piece.start, piece.end
        self._log(f"skipped {piece.kind} {piece.number} ({x0:.2f}, {y0:.2f}) -> ({x1:.2f}, {y1:.2f}), "
                  f"{math.dist(piece.start, piece.end):.2f} m: {self._why or 'no reason given'}")
        if self._skips_in_a_row >= self.settings.max_skips_in_a_row:
            return f"{self._skips_in_a_row} rows in a row were blocked; the map may be wrong there"
        self.index += 1
        nxt = self.current_piece
        if nxt is not None and nxt.kind == CONNECTOR:
            self.pieces[self.index] = Piece(TRANSIT, nxt.start, nxt.end)
        return None

    # ------------------------------------------------------------------ one piece
    def _check(self) -> None:
        with self._lock:
            if self._stop_reason is not None or self._paused:
                raise _Interrupted()

    def _position(self) -> Tuple[float, float]:
        waited = 0.0
        while True:
            pose = self.driver.pose()
            if pose is not None:
                return pose[0], pose[1]
            if waited >= self.settings.pose_wait:
                raise _JobFailed("the robot does not know where it is on the map")
            if self._abort.wait(0.1):
                self._check()
            waited += 0.1

    def _count_turn(self, heading: float) -> None:
        if self._last_heading is not None:
            change = abs(math.atan2(math.sin(heading - self._last_heading), math.cos(heading - self._last_heading)))
            if change > _TURN:
                self.turns_done += 1
        self._last_heading = heading

    def _drive_piece(self, piece: Piece) -> bool:
        """True when the piece is done, False when it must be skipped. Raises _Interrupted on pause
        or stop, and _JobFailed when the job cannot go on."""
        if piece.kind == TRANSIT:
            return self._transit(piece)
        attempts = 0
        while True:
            self._check()
            here = self._position()
            if math.dist(here, piece.end) <= self.settings.goal_tolerance:
                return True
            start = _project(here, piece)
            if math.dist(here, start) > self.settings.rejoin_distance:
                self._count_turn(math.atan2(start[1] - here[1], start[0] - here[0]))
                result = self.driver.navigate(start[0], start[1], piece.heading, self._abort)
                if result == SUCCEEDED:
                    continue  # back on the line: drive it from there
                self._why = self._failure_text("going back to its line", result, piece)
                self._after_failure(result)
                attempts += 1
                if attempts >= 2:
                    return False
                continue
            if math.dist(start, piece.end) < self.settings.min_drive:
                return True
            self._count_turn(piece.heading)
            pauses = self.pauses
            result = self.driver.follow_line(start, piece.end, self._abort)
            if result == SUCCEEDED:
                return True
            if result == CANCELLED and self.pauses != pauses:
                continue  # a pause called it off and was already resumed: not a failure
            self._why = self._failure_text("driving along it", result, piece)
            self._after_failure(result)
            here = self._position()
            if math.dist(here, piece.end) <= self.settings.goal_tolerance:
                return True
            attempts += 1
            if attempts >= 2:
                return False
            self.driver.clear_local_costmap()

    def _transit(self, piece: Piece) -> bool:
        """Two tries (the costmap cleared between them): Nav2 can fail a trip it would make a moment
        later, for example while its costmaps fill in at the start of a job."""
        attempts = 0
        while True:
            self._check()
            here = self._position()
            if math.dist(here, piece.end) <= self.settings.goal_tolerance:
                return True
            heading = self._heading_after(self.index)
            self._count_turn(math.atan2(piece.end[1] - here[1], piece.end[0] - here[0]))
            pauses = self.pauses
            result = self.driver.navigate(piece.end[0], piece.end[1], heading, self._abort)
            if result == SUCCEEDED:
                self._last_heading = heading
                return True
            if result == CANCELLED and self.pauses != pauses:
                continue  # a pause called it off and was already resumed
            self._after_failure(result)
            if math.dist(self._position(), piece.end) <= self.settings.goal_tolerance:
                return True
            attempts += 1
            if attempts >= 2:
                target = self._target_name(self.index)
                raise _JobFailed(f"could not get to {target}: Nav2 found no way there, twice")
            self.driver.clear_local_costmap()

    def _failure_text(self, doing: str, result: str, piece: Piece) -> str:
        try:
            here = self._position()
            where = f"at ({here[0]:.2f}, {here[1]:.2f}), {math.dist(here, piece.end):.2f} m from its end"
        except Exception:  # the position is only for the log
            where = "where the robot was is unknown"
        return f"{doing} ended {result}, twice; the robot was {where}"

    def _after_failure(self, result: str) -> None:
        """A goal did not succeed. If we called it off (pause, stop), leave the piece. Otherwise wait
        a moment: a mode change (Stop, Dock, failsafe) switches Nav2 off, which fails the goal just
        before the node hears of the change and stops the job."""
        if result == CANCELLED:
            self._check()  # a cancel nobody here asked for counts as a failure
        if self._abort.wait(self.settings.failure_settle):
            self._check()
        waited = 0.0
        while not self.driver.ready():
            if waited >= self.settings.not_ready_wait:
                raise _JobFailed("the robot is not in cleaning mode")
            if self._abort.wait(0.1):
                self._check()
            waited += 0.1

    def _heading_after(self, index: int) -> float:
        for p in self.pieces[index + 1:]:
            if p.length > 1e-9:
                return p.heading
        return self.pieces[index].heading

    def _target_name(self, index: int) -> str:
        for p in self.pieces[index + 1:]:
            if p.kind == ROW:
                return f"row {p.number}"
            if p.kind == EDGE:
                return f"edge {p.number}"
        return "the next part"


class _JobFailed(Exception):
    pass


def _project(point: Tuple[float, float], piece: Piece) -> Tuple[float, float]:
    """The nearest point of the piece to `point`."""
    (x0, y0), (x1, y1) = piece.start, piece.end
    dx, dy = x1 - x0, y1 - y0
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return piece.start
    t = max(0.0, min(1.0, ((point[0] - x0) * dx + (point[1] - y0) * dy) / length2))
    return (x0 + t * dx, y0 + t * dy)


def time_left(remaining_distance: float, remaining_turns: int, distance: float, cleaning_time: float,
              turning_time: float, turns_done: int, s: Settings) -> float:
    """Seconds still to go: the distance left at the speed measured so far in this job, plus the turns
    left at the time per turn measured so far. Until there is enough to measure, the planned values."""
    driving_time = cleaning_time - turning_time
    speed = distance / driving_time if distance >= 1.0 and driving_time >= 10.0 else s.cleaning_speed
    per_turn = turning_time / turns_done if turns_done >= 3 else s.turn_time
    return remaining_distance / max(speed, 0.02) + remaining_turns * per_turn


def remaining_turns(pieces: List[Piece]) -> int:
    return count_turns(pieces)
