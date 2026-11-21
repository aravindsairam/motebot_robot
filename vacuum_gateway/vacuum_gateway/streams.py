"""Which copies of a stream go to the phones, and what a phone that connects late is sent.
No ROS, no networking. Times are seconds on a steady clock.
"""

from typing import Optional

SEND, HOLD, SKIP = "send", "hold", "skip"


class Stream:
    """One kind of message the robot keeps publishing, such as the robot state or the map.

    offer() decides about each new copy:
      SEND  it differs from the last copy sent, or the last one sent is older than
            `repeat_interval` (None: an unchanged copy is never sent again);
      HOLD  it would be sent, but less than `min_interval` has passed since the last copy sent.
            The caller sends release() once hold_delay() has passed, so the newest copy always
            arrives in the end, at most once per `min_interval`;
      SKIP  nothing new.

    Repeats let the app see that the robot is still there; `min_interval` keeps a large message
    (the map) from being sent more often than a phone needs it.

    The newest copy is kept for phones that connect later, but only while it is younger than
    `replay_max_age` (None: always). An old copy of the robot state would show a robot that may
    have stopped as if it were live; the map, in contrast, stays valid until a new one comes.
    """

    def __init__(self, name: str, repeat_interval: Optional[float] = None, replay_max_age: Optional[float] = None,
                 min_interval: float = 0.0):
        self.name = name
        self.repeat_interval = repeat_interval
        self.replay_max_age = replay_max_age
        self.min_interval = min_interval
        self._latest: Optional[bytes] = None
        self._latest_at = 0.0
        self._sent: Optional[bytes] = None
        self._sent_at = 0.0
        self._held = False

    def offer(self, frame: bytes, now: float) -> str:
        """A new copy has arrived."""
        self._latest, self._latest_at = frame, now
        changed = frame != self._sent
        repeat_due = self.repeat_interval is not None and now - self._sent_at >= self.repeat_interval
        if not (changed or repeat_due):
            return SKIP
        if self._sent is not None and now - self._sent_at < self.min_interval:
            self._held = True
            return HOLD
        self._sent, self._sent_at = frame, now
        return SEND

    def hold_delay(self, now: float) -> float:
        """Seconds until a held copy may go."""
        return max(0.0, self._sent_at + self.min_interval - now)

    def release(self, now: float) -> Optional[bytes]:
        """The held copy, now that its time has come: the newest one, unless it has been sent."""
        if not self._held:
            return None
        self._held = False
        if self._latest == self._sent:
            return None
        self._sent, self._sent_at = self._latest, now
        return self._latest

    def replay(self, now: float) -> Optional[bytes]:
        """The copy for a phone that has just connected, or None if there is none recent enough."""
        if self._latest is None:
            return None
        if self.replay_max_age is not None and now - self._latest_at > self.replay_max_age:
            return None
        return self._latest
