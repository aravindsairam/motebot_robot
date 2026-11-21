"""The course of one exploration, from the request to the end. No ROS imports.

    switch to mapping (from idle or docked: a new map) -> switch to exploring -> wait for Nav2
    -> note where the robot stands -> run explorers, one after another, until it is finished
    -> drive back to where it started -> hand back to mapping

The robot, the mode manager, Nav2 and the explorer are reached through a `world` object (see
World below); the exploration node gives the real one, the tests a fake one.

"Finished" has three forms, because the explorer's own "finished" cannot be trusted alone:

  * the explorer says so. Its search may have run on a costmap that lagged behind the growing
    map, so the explorer is ended and a fresh one takes a second look. If that one also says
    finished without sending a goal, nothing is left to look at. If it sends a goal, exploring
    goes on.
  * the explorer is alive but no goal has been running for `quiet_limit` seconds: it has given up
    every place left as unreachable (it then sends neither a goal nor "finished"). No second
    look: a fresh explorer would only try the same places again.
  * the time limit has passed (an open space, or a loop): finished early, the map kept.
"""

import threading
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

# The job's states, as in vacuum_interfaces/msg/ExplorationStatus.
NONE = "none"
STARTING = "starting"
EXPLORING = "exploring"
RETURNING = "returning"
ENDING = "ending"
FINISHED = "finished"
STOPPED = "stopped"
FAILED = "failed"
JOB_STATES = (STARTING, EXPLORING, RETURNING, ENDING)  # the job is still running

MAPPING_MODE = "mapping"
EXPLORING_MODE = "exploring"
NEW_MAP_MODES = ("idle", "docked")  # an exploration from here begins a new map
START_MODES = NEW_MAP_MODES + (MAPPING_MODE,)


@dataclass
class Settings:
    time_limit: float = 1200.0  # s of robot time; then it ends early, with the map kept
    # s of robot time with no goal running, the explorer alive: nothing reachable is left. Longer
    # than the explorer's start-up (6 to 8 s) and its pause between goals; shorter than the 90 s
    # after which it tries given-up places again.
    quiet_limit: float = 30.0
    answer_timeout: float = 5.0  # s of wall time a running explorer may leave its state requests unanswered
    startup_timeout: float = 15.0  # s of wall time a fresh explorer has to answer at all
    tick: float = 0.2  # s between looks


class Explorer(Protocol):
    """One explorer program, started for this exploration."""

    def exit_code(self) -> Optional[int]: ...  # None while it runs
    def last_answer(self) -> Optional[float]: ...  # wall time of its last answer to a state request; None: never
    def finished(self) -> bool: ...  # it has said "finished" since it started
    def end(self) -> str: ...  # stop it (ask, then terminate, then kill); says how it ended


class World(Protocol):
    def mode(self) -> Tuple[str, bool]: ...  # the robot's mode, and whether a change is in progress
    def ask_mode(self, mode: str) -> Tuple[bool, str]: ...  # asks the mode manager; retries while it is busy
    def wait_ready(self, abort: threading.Event) -> str: ...  # Nav2 and slam_toolbox up in exploring; "" or why not
    def pose(self) -> Optional[Tuple[float, float, float]]: ...  # x, y, heading on the map
    def now(self) -> float: ...  # robot time, s
    def wall(self) -> float: ...  # wall time, s (monotonic)
    def start_explorer(self) -> Explorer: ...
    def goals_seen(self) -> int: ...  # Nav2 goals seen under way since the node started (a running count)
    def goal_active(self) -> bool: ...  # is a Nav2 goal under way right now?
    def navigate(self, x: float, y: float, yaw: float, abort: threading.Event) -> bool: ...
    def cancel_goals(self) -> None: ...  # cancel every Nav2 goal
    def sleep(self, seconds: float) -> None: ...


class ExplorationRunner:
    """Runs one exploration on the caller's thread. `stop()` may be called from any thread."""

    def __init__(self, world: World, settings: Settings = Settings()):
        self.world = world
        self.settings = settings
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self.stop_reason: Optional[str] = None
        self._stop_failed = False
        self.state = STARTING
        self.message = "starting"
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.started_at: Optional[float] = None  # robot time when the robot began exploring
        self.ended_at: Optional[float] = None
        self.goals_sent = 0
        self.explorers_started = 0
        self._goals_base: Optional[int] = None
        self._entered = False  # this exploration put the robot in exploring mode
        self._explorer: Optional[Explorer] = None

    # ------------------------------------------------------------------ outside
    def stop(self, reason: str, failed: bool = False) -> None:
        """End the exploration: stopped (by a person, or another mode), or failed (failsafe)."""
        with self._lock:
            if self.stop_reason is None:
                self.stop_reason = reason
                self._stop_failed = failed
        self._abort.set()

    @property
    def exploring_time(self) -> float:
        if self.started_at is None:
            return 0.0
        end = self.ended_at if self.ended_at is not None else self.world.now()
        return max(0.0, end - self.started_at)

    # ------------------------------------------------------------------ the course
    def run(self) -> Tuple[str, str]:
        """Everything, from the first mode change to the hand-back. Returns (outcome, message),
        outcome being FINISHED, STOPPED or FAILED; `state` and `message` then say the same."""
        outcome, message, back = self._explore()
        if outcome == FINISHED and back:
            message = self._return(message)
            if self.stop_reason is not None:
                outcome, message = self._stopped()
        self._set(ENDING, "ending: handing back to mapping, with the map kept")
        handed = self._hand_back()
        if handed:
            message += f" ({handed})"
        self._set(outcome, message)
        return outcome, message

    def _explore(self) -> Tuple[str, str, bool]:
        """Up to the end of the exploring itself. Returns (outcome, message, drive back?)."""
        if self.stop_reason is not None:
            return (*self._stopped(), False)
        mode, _ = self.world.mode()
        if mode not in START_MODES:
            return FAILED, f"stop what the robot is doing first (it is in {mode} mode)", False
        if mode in NEW_MAP_MODES:
            self._set(STARTING, "switching to mapping: a new map begins")
            ok, why = self.world.ask_mode(MAPPING_MODE)
            if not ok:
                return FAILED, f"the robot did not switch to mapping: {why}", False
        if self.stop_reason is not None:
            return (*self._stopped(), False)
        self._set(STARTING, "switching to exploring")
        ok, why = self.world.ask_mode(EXPLORING_MODE)
        if not ok:
            return FAILED, f"the robot did not switch to exploring: {why}", False
        self._entered = True
        self._set(STARTING, "waiting for navigation to start")
        why = self.world.wait_ready(self._abort)
        if self.stop_reason is not None:
            return (*self._stopped(), False)
        if why:
            return FAILED, why, False
        self.start_pose = self.world.pose()
        self.started_at = self.world.now()
        self._goals_base = self.world.goals_seen()
        outcome, message = self._run_explorers()
        self.ended_at = self.world.now()
        return outcome, message, outcome == FINISHED and self.start_pose is not None

    def _run_explorers(self) -> Tuple[str, str]:
        second_look = False
        while True:
            explorer = self.world.start_explorer()
            with self._lock:
                self._explorer = explorer
            self.explorers_started += 1
            self._set(EXPLORING, "looking again: making sure nothing is left" if second_look else "exploring")
            outcome = self._watch(explorer, second_look)
            self._end_explorer(explorer)
            if outcome is None:  # the explorer said finished: a fresh one takes a second look
                second_look = True
                continue
            return outcome

    def _watch(self, explorer: Explorer, second_look: bool) -> Optional[Tuple[str, str]]:
        """Watch one explorer until something decides. None: it said finished, take another look."""
        s, w = self.settings, self.world
        born_wall, born = w.wall(), w.now()
        goals_at_birth = w.goals_seen()
        last_goal = born  # the quiet spell is counted from the explorer's start
        looked_again = second_look
        while True:
            if self.stop_reason is not None:
                return self._stopped()
            code = explorer.exit_code()
            if code is not None:
                return FAILED, f"the explorer stopped running (exit code {code})"
            answered = explorer.last_answer()
            if answered is None or answered < born_wall:
                if w.wall() - born_wall > s.startup_timeout:
                    return FAILED, f"the explorer did not answer within {s.startup_timeout:.0f} s of starting"
            elif w.wall() - answered > s.answer_timeout:
                return FAILED, f"the explorer stopped answering ({w.wall() - answered:.0f} s without an answer)"
            now = w.now()
            self.goals_sent = w.goals_seen() - (self._goals_base or 0)
            if w.goal_active():
                last_goal = now
            if looked_again and w.goals_seen() > goals_at_birth:
                looked_again = False  # the second look found more: exploring goes on
                self._set(EXPLORING, "exploring")
            if explorer.finished():
                if looked_again:
                    return FINISHED, f"{self._explored()}: nothing left to look at"
                return None
            if now - last_goal > s.quiet_limit:
                return FINISHED, f"{self._explored()}: some places could not be reached"
            if self.started_at is not None and now - self.started_at > s.time_limit:
                return FINISHED, f"{self._explored()}: finished early, the time limit of {_duration(s.time_limit)} was reached"
            w.sleep(s.tick)

    def _end_explorer(self, explorer: Explorer) -> None:
        explorer.end()
        with self._lock:
            if self._explorer is explorer:
                self._explorer = None
        # The explorer's goal may still be under way (an explorer that was killed does not cancel
        # it). The mode manager also cancels every goal on leaving exploring; this covers the
        # second look, which stays in exploring.
        self.world.cancel_goals()

    def _return(self, message: str) -> str:
        x, y, yaw = self.start_pose
        self._set(RETURNING, "going back to where it started")
        if self.world.navigate(x, y, yaw, self._abort):
            return message + "; back where it started"
        if self.stop_reason is not None:
            return message
        return message + "; could not get back to where it started"

    def _hand_back(self) -> str:
        """Back to mapping, with the map kept, if the robot is still exploring. Returns a note when
        that did not work, else ""."""
        mode, _ = self.world.mode()
        if not self._entered or mode != EXPLORING_MODE:
            return ""  # already in mapping (a Stop), in failsafe, or this exploration never got there
        ok, why = self.world.ask_mode(MAPPING_MODE)
        return "" if ok else f"could not hand back to mapping: {why}"

    # ------------------------------------------------------------------ helpers
    def _stopped(self) -> Tuple[str, str]:
        with self._lock:
            failed, reason = self._stop_failed, self.stop_reason
        return (FAILED, f"failed: {reason}") if failed else (STOPPED, f"stopped: {reason}")

    def _explored(self) -> str:
        return f"explored for {_duration(self.world.now() - (self.started_at or self.world.now()))}"

    def _set(self, state: str, message: str) -> None:
        with self._lock:
            self.state, self.message = state, message


def _duration(seconds: float) -> str:
    seconds = max(0.0, seconds)
    if seconds < 90:
        return f"{seconds:.0f} s"
    return f"{seconds / 60:.0f} min"
