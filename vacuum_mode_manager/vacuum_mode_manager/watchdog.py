"""Small fault detectors used by the mode manager. No ROS imports; callers pass in the time.

All times are monotonic wall-clock seconds, not simulation time: if the simulator freezes,
simulation time stops too, and a watchdog on simulation time would never fire.
"""

from typing import Optional


class TopicWatchdog:
    """Reports a fault when a topic that should be streaming goes quiet."""

    def __init__(self, label: str, timeout: float, startup_grace: float, started_at: float):
        self.label = label
        self.timeout = timeout
        self.startup_grace = startup_grace
        self._started_at = started_at
        self._last: Optional[float] = None

    def beat(self, now: float) -> None:
        """Call whenever a message arrives."""
        self._last = now

    @property
    def seen(self) -> bool:
        return self._last is not None

    def fault(self, now: float) -> Optional[str]:
        """A description of the problem, or None when healthy."""
        if self._last is None:
            waited = now - self._started_at
            if waited > self.startup_grace:
                return f"no {self.label} data received in the {waited:.0f} s since start-up"
            return None
        silent = now - self._last
        if silent > self.timeout:
            return f"{self.label} data stopped {silent:.1f} s ago"
        return None


class NodeHealth:
    """Tracks one managed node across health polls.

    A single missed reply or a state seen mid-change is not a fault; the same problem on
    `threshold` consecutive polls is.
    """

    def __init__(self, name: str, threshold: int = 2):
        self.name = name
        self.threshold = threshold
        self._silent_polls = 0
        self._wrong_polls = 0
        self._last_state: Optional[str] = None

    def record(self, state: Optional[str], expected: Optional[str]) -> None:
        """state: lifecycle label from the node, or None if it did not answer.
        expected: the label it should have in the current mode, or None for "any"."""
        self._last_state = state
        if state is None:
            self._silent_polls += 1
            self._wrong_polls = 0
            return
        self._silent_polls = 0
        if expected is not None and state != expected:
            self._wrong_polls += 1
        else:
            self._wrong_polls = 0

    def reset(self) -> None:
        self._silent_polls = 0
        self._wrong_polls = 0

    def not_responding(self) -> Optional[str]:
        if self._silent_polls >= self.threshold:
            return f"node {self.name} is not responding"
        return None

    def wrong_state(self, expected: Optional[str]) -> Optional[str]:
        if expected is not None and self._wrong_polls >= self.threshold:
            return f"node {self.name} is {self._last_state}, expected {expected}"
        return None


class JobWatchdog:
    """Watches a node that must keep reporting, with a job, while the robot is in its mode (the
    cleaning node in cleaning mode). Outside that mode its silence is no fault. On entering the
    mode the node gets the full timeout from that moment, whatever it did before. A node that
    reports no job in its mode is a fault too: it was restarted mid-job, and must not carry on."""

    def __init__(self, label: str, timeout: float):
        self.label = label
        self.timeout = timeout
        self._last: Optional[float] = None
        self._has_job = False
        self._active_since: Optional[float] = None

    def beat(self, now: float, has_job: bool = True) -> None:
        self._last = now
        self._has_job = has_job

    def fault(self, now: float, active: bool) -> Optional[str]:
        if not active:
            self._active_since = None
            return None
        if self._active_since is None:
            self._active_since = now
        since = max(self._last if self._last is not None else self._active_since, self._active_since)
        silent = now - since
        if silent > self.timeout:
            return f"the {self.label} stopped reporting {silent:.1f} s ago"
        if self._last is not None and self._last >= self._active_since and not self._has_job:
            return f"the {self.label} reports no job (was it restarted?)"
        return None
