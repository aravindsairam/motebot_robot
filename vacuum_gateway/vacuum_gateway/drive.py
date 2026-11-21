"""Driving by hand from a phone: who may drive, and how fast. No ROS imports.

Only one phone drives at a time. The first phone to send a Drive becomes the driver; another
phone's Drive is refused until the driver has sent nothing for QUIET_HANDOVER seconds (it let go,
or it vanished). The safety gate stops the robot 0.6 s after the last command whatever happens
here; this only keeps two thumbs from fighting over the robot.
"""

import math
from typing import Optional, Tuple

QUIET_HANDOVER = 1.0  # s without a Drive from the driver before another phone may drive

# The Create 3's limits, as in the safety gate's settings (vacuum_bringup/config/safety.yaml).
# The gate clamps too; clamping here keeps what the gateway publishes honest.
MAX_FORWARD = 0.306  # m/s
MAX_TURN = 1.9  # rad/s


def clamp_speeds(forward: float, turn: float) -> Optional[Tuple[float, float]]:
    """The speeds within the base's limits, or None when they are not numbers."""
    if not (math.isfinite(forward) and math.isfinite(turn)):
        return None
    return max(-MAX_FORWARD, min(MAX_FORWARD, forward)), max(-MAX_TURN, min(MAX_TURN, turn))


class DriverLock:
    def __init__(self, quiet_handover: float = QUIET_HANDOVER):
        self._quiet = quiet_handover
        self.driver = None  # the phone driving, or None
        self._last = 0.0

    def claim(self, phone, now: float) -> bool:
        """`phone` sends a Drive at `now`: may it drive?"""
        if self.driver is None or self.driver == phone or now - self._last >= self._quiet:
            self.driver, self._last = phone, now
            return True
        return False

    def release(self, phone) -> bool:
        """`phone` has gone. True if it was driving: the robot should then be told to stop."""
        if self.driver == phone:
            self.driver = None
            return True
        return False
