"""Drive ROS 2 lifecycle nodes from the mode manager.

Every call here has a timeout and reads the node's real state before acting, so a lost reply or
a repeated request is harmless: the worst case is one extra state query.

The calls block the calling thread. The mode manager runs on a multi-threaded executor, so the
replies are processed by other threads while a caller waits.
"""

import threading
import time
from typing import Iterable, List, Optional, Tuple

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

UNCONFIGURED, INACTIVE, ACTIVE = "unconfigured", "inactive", "active"

# The single step that moves a node from `current` towards `target`.
_NEXT_STEP = {
    (UNCONFIGURED, INACTIVE): Transition.TRANSITION_CONFIGURE,
    (UNCONFIGURED, ACTIVE): Transition.TRANSITION_CONFIGURE,
    (INACTIVE, ACTIVE): Transition.TRANSITION_ACTIVATE,
    (INACTIVE, UNCONFIGURED): Transition.TRANSITION_CLEANUP,
    (ACTIVE, INACTIVE): Transition.TRANSITION_DEACTIVATE,
    (ACTIVE, UNCONFIGURED): Transition.TRANSITION_DEACTIVATE,
}
_STEP_NAMES = {
    Transition.TRANSITION_CONFIGURE: "configure",
    Transition.TRANSITION_ACTIVATE: "activate",
    Transition.TRANSITION_DEACTIVATE: "deactivate",
    Transition.TRANSITION_CLEANUP: "cleanup",
}


def call_service(client, request, timeout: float):
    """Call a service and wait up to `timeout` seconds. Returns the response, or None on failure."""
    if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
        return None
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        client.remove_pending_request(future)
        return None
    try:
        return future.result()
    except Exception:  # the service raised, or the node went away mid-call
        return None


class ManagedNode:
    """One lifecycle node the mode manager controls."""

    def __init__(self, node, name: str, callback_group):
        self.name = name
        self._get = node.create_client(GetState, f"{name}/get_state", callback_group=callback_group)
        self._change = node.create_client(ChangeState, f"{name}/change_state", callback_group=callback_group)
        self._params = node.create_client(SetParameters, f"{name}/set_parameters", callback_group=callback_group)

    def state(self, timeout: float = 2.0) -> Optional[str]:
        """Lifecycle state label ('unconfigured', 'inactive', 'active', ...), or None if no answer."""
        response = call_service(self._get, GetState.Request(), timeout)
        return response.current_state.label if response is not None else None

    def set_string_parameter(self, name: str, value: str, timeout: float = 3.0) -> bool:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
        ]
        response = call_service(self._params, request, timeout)
        return response is not None and all(r.successful for r in response.results)

    def ensure(self, target: str, step_timeout: float = 20.0, deadline: float = 45.0) -> Tuple[bool, str]:
        """Bring the node to `target` one transition at a time. Returns (ok, reason if not)."""
        end = time.monotonic() + deadline
        failures = 0
        while time.monotonic() < end:
            current = self.state()
            if current is None:
                failures += 1
                if failures >= 3:
                    return False, f"{self.name} is not responding"
                time.sleep(0.5)
                continue
            if current == target:
                return True, ""
            step = _NEXT_STEP.get((current, target))
            if step is None:  # mid-transition ('configuring', ...) or finalized
                if current == "finalized":
                    return False, f"{self.name} has shut down"
                time.sleep(0.2)
                continue
            request = ChangeState.Request()
            request.transition.id = step
            response = call_service(self._change, request, step_timeout)
            if response is None or not response.success:
                # The reply may simply have been lost: the next loop reads the real state.
                failures += 1
                if failures >= 3:
                    verb = _STEP_NAMES[step]
                    why = "did not answer" if response is None else "refused"
                    return False, f"{self.name} {why} when asked to {verb}"
                time.sleep(0.3)
        return False, f"{self.name} did not reach {target} within {deadline:.0f} s"


def bring_up(nodes: Iterable[ManagedNode]) -> Tuple[bool, str]:
    """Configure every node, then activate every node, in the given order (as Nav2's own manager does)."""
    nodes = list(nodes)
    for n in nodes:
        if n.state() == UNCONFIGURED:
            ok, why = n.ensure(INACTIVE)
            if not ok:
                return False, why
    for n in nodes:
        ok, why = n.ensure(ACTIVE)
        if not ok:
            return False, why
    return True, ""


def stand_by(nodes: Iterable[ManagedNode], step_timeout: float = 20.0, deadline: float = 45.0) -> Tuple[bool, str]:
    """Bring every node to inactive: configured and ready, but doing nothing.

    For nodes that cannot be configured twice. Such a node is switched off by deactivating it,
    never by cleaning it up. Keeps going after a failure.
    """
    problems: List[str] = []
    for n in list(nodes)[::-1]:
        ok, why = n.ensure(INACTIVE, step_timeout, deadline)
        if not ok:
            problems.append(why)
    return (not problems), "; ".join(problems)


def bring_down(nodes: Iterable[ManagedNode], step_timeout: float = 20.0, deadline: float = 45.0) -> Tuple[bool, str]:
    """Deactivate then clean up every node, in reverse order. Keeps going after a failure."""
    nodes = list(nodes)[::-1]
    problems: List[str] = []
    for n in nodes:
        if n.state() == ACTIVE:
            ok, why = n.ensure(INACTIVE, step_timeout, deadline)
            if not ok:
                problems.append(why)
    for n in nodes:
        ok, why = n.ensure(UNCONFIGURED, step_timeout, deadline)
        if not ok:
            problems.append(why)
    return (not problems), "; ".join(problems)
