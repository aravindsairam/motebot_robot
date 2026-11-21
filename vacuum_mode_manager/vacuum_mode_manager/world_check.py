"""Is the robot software running with the maps of the world the simulator simulates?

Each world keeps its own maps folder. A map of one world loaded in another would leave the robot
lost in a place it believes it knows, so the mode manager compares the world it was started for
with the one the simulator announces on /sim/world, and stays in failsafe if they differ.
"""

from typing import Optional

WAITING = "the simulator's world (/sim/world)"


def world_problem(expected: str, announced: Optional[str], waiting_over: bool = False) -> str:
    """'' if the robot may run; otherwise why not, in words for the person watching.

    expected: the world the robot software was started for ('' = no check).
    announced: the world the simulator announced, or None if nothing was heard yet.
    waiting_over: start-up has ended. Before that, nothing heard is only something to wait for;
    after it, it is a fault that does not clear until the simulator says which world it runs.
    """
    if not expected or announced == expected:
        return ""
    if announced is None:
        return "the simulator's world is unknown: nothing heard on /sim/world" if waiting_over else ""
    return (f"wrong world: the simulator runs '{announced}', the robot software was started for "
            f"'{expected}' and its maps")
