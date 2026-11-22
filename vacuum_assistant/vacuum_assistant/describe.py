"""The robot's messages as plain data for the language model, with words it can repeat to a user.
No ROS imports: anything with the right fields will do (the tests use plain objects)."""

from typing import Optional

MODES = {  # from vacuum_interfaces/msg/RobotState.msg
    "idle": "standing still, waiting",
    "mapping": "making a new map while someone drives it",
    "navigation": "ready to go to a point on the map, or on its way",
    "manual": "being driven with the joystick",
    "cleaning": "cleaning (or a clean is paused)",
    "exploring": "making a new map by driving around on its own",
    "docking": "driving onto its dock",
    "docked": "parked on its dock",
    "failsafe": "stopped because something went wrong; it waits for a reset",
}
_ACTIVE = ("starting", "cleaning", "paused", "ending")


def _r(value, digits=1):
    return round(float(value), digits)


def clean_numbers(s) -> dict:
    """One clean's numbers (a CleaningStatus), in units people use."""
    out = {
        "what": s.target or "everything",
        "state": s.state,
        "message": s.message,
        "cleaned_m2": _r(s.cleaned_area),
        "planned_m2": _r(s.planned_area),
        "share_of_planned_floor_percent": _r(s.coverage, 0),
        "minutes_cleaning": _r(s.cleaning_time / 60.0),
        "distance_m": _r(s.distance),
        "rows_done": int(s.rows_done),
        "rows_total": int(s.rows_total),
        "rows_skipped_blocked": int(s.rows_skipped),
        "bumps": int(s.bumps),
    }
    if s.state in _ACTIVE:
        out["minutes_left"] = _r(s.time_left / 60.0)
    return out


def settings_numbers(s) -> dict:
    return {
        "cleaning_speed_m_per_s": _r(s.cleaning_speed, 2),
        "joystick_speed_m_per_s": _r(s.manual_speed, 2),
        "row_spacing_cm": _r(s.row_spacing * 100.0, 0),
        "row_direction": "automatic, room by room" if s.row_angle_auto else f"{s.row_angle:g} degrees from the map's x axis",
    }


def status(robot_state, cleaning_status, settings) -> dict:
    """What the robot is doing now. Any of the three may be None (not heard from yet)."""
    out = {}
    if robot_state is None:
        out["mode"] = "unknown: the mode manager has not been heard from"
    else:
        out["mode"] = robot_state.state
        out["mode_meaning"] = MODES.get(robot_state.state, "")
        if robot_state.transitioning:
            out["changing_mode"] = True
        if robot_state.reason:
            out["why_this_mode"] = robot_state.reason
        if robot_state.faults:
            out["problems"] = list(robot_state.faults)
        if robot_state.making_map:
            out["map"] = "making a new map (not saved yet)"
        else:
            out["map"] = robot_state.current_map or "no saved map yet"
    if cleaning_status is not None and cleaning_status.state in _ACTIVE:
        out["clean_under_way"] = clean_numbers(cleaning_status)
    elif cleaning_status is not None and cleaning_status.state not in ("none", ""):
        out["clean_under_way"] = None
        out["last_clean_since_robot_start"] = clean_numbers(cleaning_status)
    if settings is not None:
        out["settings"] = settings_numbers(settings)
    return out


def history(records, totals, map_name: Optional[str]) -> dict:
    """The cleans (CleaningRecord, newest first) and the totals (CleaningTotals). Times stay in
    seconds since 1970 here; the tool turns them into words."""
    cleans = []
    for r in records:
        entry = clean_numbers(r.status)
        entry.pop("state")
        entry = {"started": r.started, "ended": r.ended, "ended_as": r.status.state, **entry}
        cleans.append(entry)
    return {
        "map": map_name or "no saved map",
        "cleans": cleans,
        "totals_over_every_clean": {
            "cleans": int(totals.cleans),
            "area_m2": _r(totals.area, 0),
            "hours": _r(totals.time / 3600.0),
            "distance_m": _r(totals.distance, 0),
        },
    }
