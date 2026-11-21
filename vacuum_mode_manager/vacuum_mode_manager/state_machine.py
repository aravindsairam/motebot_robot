"""The robot's modes and which changes between them are allowed. No ROS imports.

A mode decides two things here:
  * which groups of nodes are running (see GROUPS and resources_for),
  * which other modes may be requested next (the transition table).
Who may drive the wheels in each mode is decided by the safety gate (vacuum_safety), which reads
the mode this manager publishes.

The rule the table and the resource map exist to guarantee: at any moment at most one thing
publishes the map -> odom transform. slam_toolbox does while a map is being made (mapping and
exploring); AMCL does otherwise.

A map is being made in two modes: mapping (someone drives) and exploring (the robot drives itself
through Nav2). Exploring is entered only from mapping and goes back to mapping, and both keep
slam_toolbox running, so the map survives the change in both directions.
"""

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

IDLE = "idle"
MAPPING = "mapping"
NAVIGATION = "navigation"
MANUAL = "manual"
DOCKING = "docking"  # on its way onto the dock; ends in docked, or in idle if it fails
DOCKED = "docked"  # parked on the dock; entered only by a docking that succeeded
CLEANING = "cleaning"  # a cleaning job runs or is paused; the cleaning node drives through Nav2.
# Entered only while the cleaning node reports a job that is starting (see build_state_machine).
EXPLORING = "exploring"  # the robot maps by itself: slam_toolbox keeps making the map, Nav2 drives.
# Entered only from mapping, and only while the exploration node reports a job that is starting.
FAILSAFE = "failsafe"
STATES = (IDLE, MAPPING, NAVIGATION, MANUAL, CLEANING, EXPLORING, DOCKING, DOCKED, FAILSAFE)
MAP_MAKING = (MAPPING, EXPLORING)  # the modes in which a map is being made

# Groups of lifecycle nodes that are switched together.
SLAM = "slam"  # slam_toolbox: builds a map and publishes map -> odom
LOCALIZATION = "localization"  # map_server + AMCL: publishes map -> odom on a saved map
NAVIGATION_GROUP = "navigation"  # Nav2 servers and the no-go zone servers
DOCKING_GROUP = "docking"  # the docking server
GROUPS = (SLAM, LOCALIZATION, NAVIGATION_GROUP, DOCKING_GROUP)

ON, OFF, IF_MAP = "on", "off", "if_map"

# What each mode needs running. IF_MAP means "on when a saved map exists".
# Failsafe is absent on purpose: it stops navigation and leaves the rest as it was.
_RESOURCES = {
    IDLE: {SLAM: OFF, LOCALIZATION: IF_MAP, NAVIGATION_GROUP: OFF, DOCKING_GROUP: OFF},
    MAPPING: {SLAM: ON, LOCALIZATION: OFF, NAVIGATION_GROUP: OFF, DOCKING_GROUP: OFF},
    NAVIGATION: {SLAM: OFF, LOCALIZATION: ON, NAVIGATION_GROUP: ON, DOCKING_GROUP: OFF},
    MANUAL: {SLAM: OFF, LOCALIZATION: IF_MAP, NAVIGATION_GROUP: OFF, DOCKING_GROUP: OFF},
    # Docking uses Nav2 to reach the spot in front of the dock, then the docking server.
    DOCKING: {SLAM: OFF, LOCALIZATION: ON, NAVIGATION_GROUP: ON, DOCKING_GROUP: ON},
    DOCKED: {SLAM: OFF, LOCALIZATION: IF_MAP, NAVIGATION_GROUP: OFF, DOCKING_GROUP: OFF},
    # Cleaning runs what navigation runs: the cleaning node sends its rows and trips to Nav2.
    CLEANING: {SLAM: OFF, LOCALIZATION: ON, NAVIGATION_GROUP: ON, DOCKING_GROUP: OFF},
    # Exploring is mapping with Nav2: slam_toolbox stays the only source of the robot's place.
    EXPLORING: {SLAM: ON, LOCALIZATION: OFF, NAVIGATION_GROUP: ON, DOCKING_GROUP: OFF},
}


def resources_for(state: str, has_map: bool) -> Dict[str, bool]:
    """Which groups must be running in `state`. Raises KeyError for failsafe, which has no fixed set."""
    wanted = _RESOURCES[state]
    return {g: (wanted[g] == ON or (wanted[g] == IF_MAP and has_map)) for g in GROUPS}


def carried_out_as(source: str, target: str, waited_for: Optional[Tuple[str, str]] = None,
                   map_resumable: bool = False) -> str:
    """The mode a request for `target`, made in `source`, really leads to.

    A stop (idle) while exploring is carried out as a change to mapping: idle would switch
    slam_toolbox off and throw the map away, and Stop is sent by people and programs that only
    mean "stand still". The same holds for a stop that had to wait for a change into or out of
    exploring (`waited_for` = that change's source and target) and finds the robot back in
    mapping: it was meant for the exploration. In failsafe with a map being made that can still be
    gone back to (`map_resumable`), a stop leaves the robot where it is: it is already standing
    still, and idle would lose the map; Reset goes back to mapping. Discarding a map is asked for
    from mapping."""
    if target != IDLE:
        return target
    if source == FAILSAFE and map_resumable:
        return FAILSAFE
    if source == EXPLORING:
        return MAPPING
    if source == MAPPING and waited_for is not None and EXPLORING in waited_for:
        return MAPPING
    return target


def waits_when_busy(current: str, target: str) -> bool:
    """Does a request for `target`, arriving while another mode change is in progress, wait for it
    to finish (True) instead of being refused as busy? Stops always wait: the robot is already held
    still during a change. So does a Reset from failsafe back to mapping: failsafe is announced at
    once and then switches Nav2 off, which takes a few seconds."""
    return target in (IDLE, FAILSAFE) or (current == FAILSAFE and target == MAPPING)


def slam_map_alive(slam_on: bool, transform_fresh: bool) -> bool:
    """Is the map being made still there? slam_toolbox must be the node switched on (AMCL publishes
    the same transform on a saved map, for instance after a failed change out of mapping) and its
    map -> odom transform still arriving."""
    return slam_on and transform_fresh


def making_map(state: str, previous: str, slam_running: bool) -> bool:
    """Is a map being made? True in mapping and exploring, and in a failsafe entered from either
    while slam_toolbox still runs: the map is still there and can be gone back to."""
    if state in MAP_MAKING:
        return True
    return state == FAILSAFE and previous in MAP_MAKING and slam_running


def resume_mapping_problem(previous: str, lifted: bool, slam_running: bool) -> str:
    """Why failsafe cannot be left for mapping with the map kept, or "" if it can.

    previous:      the mode the fault interrupted
    lifted:        was the robot off the floor at any time during this failsafe? It then no longer
                   knows where it is, and driving on would spoil the map
    slam_running:  is slam_toolbox's map -> odom transform still arriving?"""
    if previous not in MAP_MAKING:
        return "no map was being made when the fault happened"
    if lifted:
        return "the robot was lifted or fell, so it no longer knows where it is on the map being made"
    if not slam_running:
        return "slam_toolbox is no longer running, and the map being made went with it"
    return ""


# A condition returns (allowed, reason it is not allowed).
Condition = Callable[[], Tuple[bool, str]]


@dataclass(frozen=True)
class Transition:
    source: str
    target: str
    condition: Optional[Condition] = None
    internal: bool = False  # made by the manager itself when something finishes; cannot be requested


class StateMachine:
    """A current state plus the table of allowed changes. It decides; it does not act."""

    def __init__(self, initial: str = IDLE):
        if initial not in STATES:
            raise ValueError(f"unknown state: {initial}")
        self._state = initial
        self._previous = ""
        self._transitions: List[Transition] = []

    @property
    def state(self) -> str:
        return self._state

    @property
    def previous(self) -> str:
        return self._previous

    def add(self, source: str, target: str, condition: Optional[Condition] = None, internal: bool = False) -> None:
        for s in (source, target):
            if s not in STATES:
                raise ValueError(f"unknown state: {s}")
        self._transitions.append(Transition(source, target, condition, internal))

    def check(self, target: str) -> Tuple[bool, str]:
        """Can `target` be requested from the current state right now? Returns (ok, reason if not)."""
        if target not in STATES:
            return False, f"unknown state '{target}'; known states: {', '.join(STATES)}"
        for t in self._transitions:
            if t.source == self._state and t.target == target:
                if t.internal:
                    return False, f"{target} cannot be requested: the robot enters it by itself"
                if t.condition is None:
                    return True, ""
                return t.condition()
        if target == DOCKED:
            return False, "docked cannot be requested: request docking, and the robot is docked when it arrives"
        if target == CLEANING and self._state == MAPPING:
            return False, "a clean cannot start while mapping: save or discard the map first"
        if target == EXPLORING:
            return False, _EXPLORING_HOW
        if self._state == EXPLORING:
            return False, f"stop exploring first: the robot cannot go from exploring to {target}"
        return False, f"cannot go from {self._state} to {target}"

    def is_internal(self, target: str) -> bool:
        """Is `target` a state the manager may enter by itself from the current one?"""
        return any(t.source == self._state and t.target == target and t.internal for t in self._transitions)

    def allowed(self) -> List[str]:
        """States that could be requested right now."""
        return [t.target for t in self._transitions if t.source == self._state and self.check(t.target)[0]]

    def set(self, target: str) -> None:
        """Record that the robot is now in `target`. The caller has already done the work."""
        if target not in STATES:
            raise ValueError(f"unknown state: {target}")
        if target != self._state:
            self._previous = self._state
            self._state = target


_EXPLORING_HOW = ("exploring mode is started by an exploration, from mapping, not on its own: start one with "
                  "'ros2 run vacuum_exploration explore' or the app's Explore button")


def build_state_machine(
    has_map: Callable[[], bool],
    active_faults: Callable[[], List[str]],
    cleaning_job_starting: Callable[[], bool] = lambda: False,
    exploration_job_starting: Callable[[], bool] = lambda: False,
    resume_mapping_problem: Callable[[], str] = lambda: "no map was being made when the fault happened",
) -> StateMachine:
    """The transition table for the vacuum robot.

    has_map:                   is there a saved map to localize on?
    active_faults:             problems present right now (empty list when healthy)
    cleaning_job_starting:     does the cleaning node report a job that is starting? Cleaning mode is
                               entered only then, so the robot is in it exactly while a job runs.
    exploration_job_starting:  the same for the exploration node and exploring mode.
    resume_mapping_problem:    why failsafe cannot be left for mapping with the map kept, or "".
                               Asked twice a second: it must answer from what is known, not wait.
    """

    def needs_map() -> Tuple[bool, str]:
        if has_map():
            return True, ""
        return False, "no saved map yet: switch to mapping, drive around, and save a map first"

    def cleaning_job() -> Tuple[bool, str]:
        ok, why = needs_map()
        if not ok:
            return ok, why
        if cleaning_job_starting():
            return True, ""
        return False, ("cleaning mode is entered by a cleaning job, not on its own: start one with "
                       "'ros2 run vacuum_coverage clean' or the app's Clean button")

    def faults_cleared() -> Tuple[bool, str]:
        faults = active_faults()
        if not faults:
            return True, ""
        return False, "still faulty: " + "; ".join(faults)

    def exploration_job() -> Tuple[bool, str]:
        if exploration_job_starting():
            return True, ""
        return False, _EXPLORING_HOW

    def idle_from_failsafe() -> Tuple[bool, str]:
        ok, why = faults_cleared()
        if not ok:
            return ok, why
        if not resume_mapping_problem():
            return False, ("the map being made is kept: Reset goes back to mapping, "
                           "and the map can be discarded from there")
        return True, ""

    def map_resumable() -> Tuple[bool, str]:
        ok, why = faults_cleared()
        if not ok:
            return ok, why
        why = resume_mapping_problem()
        if why:
            return False, f"failsafe can only be left for idle now: {why}"
        return True, ""

    sm = StateMachine(IDLE)
    requestable = (IDLE, MAPPING, NAVIGATION, MANUAL, CLEANING, DOCKING)
    for source in (IDLE, MAPPING, NAVIGATION, MANUAL, CLEANING, DOCKING, DOCKED):
        for target in requestable:
            if source == target or (source == MAPPING and target == CLEANING):
                continue
            condition = cleaning_job if target == CLEANING else (needs_map if target in (NAVIGATION, DOCKING) else None)
            sm.add(source, target, condition)
        sm.add(source, FAILSAFE)  # reachable from anywhere, by request (emergency stop) or on a fault
    sm.add(DOCKING, DOCKED, internal=True)  # only a docking that succeeded makes the robot docked
    # Exploring: in from mapping while an exploration is starting; out to mapping (how every
    # exploration ends), by a stop (idle, which carried_out_as turns into mapping), or by a fault.
    sm.add(MAPPING, EXPLORING, exploration_job)
    sm.add(EXPLORING, MAPPING)
    sm.add(EXPLORING, IDLE)
    sm.add(EXPLORING, FAILSAFE)
    # A way out once the fault is gone; while a map being made can be gone back to, that way is mapping.
    sm.add(FAILSAFE, IDLE, idle_from_failsafe)
    # A fault must not cost the map: when one was being made and is still there, failsafe can
    # also be left for mapping, with the map kept.
    sm.add(FAILSAFE, MAPPING, map_resumable)
    return sm
