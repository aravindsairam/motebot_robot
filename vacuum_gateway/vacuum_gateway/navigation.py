"""How the newest navigation goal is going, from Nav2's navigate_to_pose action, and the route
thinned for the phone. No ROS: plain values in and out.

Every goal shows up here, whoever sent it: the app, RViz, the docking server or the explorer.
The gateway reads the action's status and feedback topics; it does not need to have sent the
goal itself.
"""

import math
from typing import List, Optional, Sequence, Tuple

from . import vacuum_pb2 as pb

State = pb.NavigationStatus.State

# action_msgs/msg/GoalStatus values -> what the phone is told
_STATES = {
    1: State.STATE_ACCEPTED,
    2: State.STATE_ON_THE_WAY,
    3: State.STATE_CANCELLING,
    4: State.STATE_REACHED,
    5: State.STATE_CANCELLED,
    6: State.STATE_FAILED,
}
_UNDER_WAY = (State.STATE_ACCEPTED, State.STATE_ON_THE_WAY, State.STATE_CANCELLING)
DISTANCE_STEP = 0.05  # m: smaller changes of the distance left are not news


class NavigationTracker:
    """Follows the newest goal: the one Nav2 accepted last."""

    def __init__(self):
        self.goal_id = b""
        self.state = State.STATE_NONE
        self.distance = 0.0

    def on_status(self, goals: Sequence[Tuple[bytes, float, int]]) -> bool:
        """Nav2's goal list: (goal id, when it was accepted in seconds, GoalStatus value) for each
        goal it still remembers. Returns True when what the phone should see has changed."""
        if not goals:
            return False
        goal_id, _, status = max(goals, key=lambda g: g[1])
        state = _STATES.get(status, State.STATE_NONE)
        if goal_id == self.goal_id and state == self.state:
            return False
        if goal_id != self.goal_id:
            self.distance = 0.0
        self.goal_id, self.state = goal_id, state
        if state not in _UNDER_WAY:
            self.distance = 0.0
        return True

    def on_feedback(self, goal_id: bytes, distance_remaining: float) -> bool:
        """Nav2's feedback for a goal. Only the newest goal's counts, and only while under way."""
        if goal_id != self.goal_id or self.state not in _UNDER_WAY:
            return False
        if abs(distance_remaining - self.distance) < DISTANCE_STEP:
            return False
        self.distance = distance_remaining
        return True

    def message(self) -> Optional[pb.NavigationStatus]:
        if not self.goal_id:
            return None
        return pb.NavigationStatus(goal_id=self.goal_id, state=self.state, distance_remaining=self.distance)


def thin_path(points: Sequence[Tuple[float, float]], spacing: float = 0.05, max_points: int = 2000) -> List[Tuple[float, float]]:
    """Keep a point every `spacing` metres (the first and last always), at most `max_points`."""
    if not points:
        return []
    kept = [points[0]]
    for x, y in points[1:-1]:
        lx, ly = kept[-1]
        if math.hypot(x - lx, y - ly) >= spacing - 1e-9:  # 1e-9: 0.05 often comes out as 0.0499999
            kept.append((x, y))
    if len(points) > 1:
        kept.append(points[-1])
    if len(kept) > max_points:  # a very long route: keep it evenly, ends included
        step = (len(kept) - 1) / (max_points - 1)
        kept = [kept[round(i * step)] for i in range(max_points)]
    return kept
