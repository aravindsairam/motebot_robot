"""The settings a user may change from the app, as plain data: their defaults, their limits, and
the file the robot keeps them in. No ROS imports.

The limits are the constants of vacuum_interfaces/msg/RobotSettings; they are repeated here so
this module needs no ROS, and test_robot_settings.py checks that both say the same.
"""

import math
import os
import tempfile
from dataclasses import asdict, dataclass, fields, replace
from typing import Optional, Tuple

import yaml

CLEANING_SPEED_MIN, CLEANING_SPEED_MAX = 0.10, 0.30  # m/s; the safety gate allows 0.306
MANUAL_SPEED_MIN, MANUAL_SPEED_MAX = 0.10, 0.30  # m/s
ROW_SPACING_MIN, ROW_SPACING_MAX = 0.10, 0.30  # m; wider than the cleaning width (0.30) would leave strips
ROW_SPACING_STEP = 0.05  # m: one map cell, the planner's own step
MANUAL_TURN_PER_SPEED = 4.0  # rad/s of turning at full joystick, per m/s of manual speed (0.30 -> 1.2)
_EPS = 1e-6


@dataclass(frozen=True)
class RobotSettings:
    cleaning_speed: float = 0.20  # as CleanRow's desired_linear_vel in nav2.yaml
    manual_speed: float = 0.30
    row_spacing: float = 0.25  # as row_spacing in coverage.yaml
    row_angle_auto: bool = True
    row_angle: float = 0.0  # degrees from the map's x axis, 0 up to (not including) 180

    @property
    def manual_turn_speed(self) -> float:
        return self.manual_speed * MANUAL_TURN_PER_SPEED

    @property
    def row_direction(self) -> str:
        """The rows' direction as a cleaning job names it: "auto", or the angle ("45")."""
        return "auto" if self.row_angle_auto else f"{self.row_angle:g}"


def _between(name: str, value: float, low: float, high: float, unit: str, scale: float = 1.0, digits: int = 2) -> str:
    if not math.isfinite(value):
        return f"the {name} is not a number"
    if value < low - _EPS or value > high + _EPS:
        return (f"the {name} must be between {low * scale:.{digits}f} and {high * scale:.{digits}f} {unit} "
                f"(asked for {value * scale:.{digits}f})")
    return ""


def check(s: RobotSettings) -> str:
    """Why these settings cannot be used, or "" if they can."""
    why = (_between("cleaning speed", s.cleaning_speed, CLEANING_SPEED_MIN, CLEANING_SPEED_MAX, "m/s")
           or _between("joystick speed", s.manual_speed, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX, "m/s")
           or _between("row spacing", s.row_spacing, ROW_SPACING_MIN, ROW_SPACING_MAX, "cm", 100.0, 0))
    if why:
        return why
    steps = s.row_spacing / ROW_SPACING_STEP
    if abs(steps - round(steps)) > 1e-3:
        return f"the row spacing must be a whole number of {ROW_SPACING_STEP * 100:.0f} cm steps (asked for {s.row_spacing * 100:.1f} cm)"
    if not math.isfinite(s.row_angle) or not 0.0 <= s.row_angle < 180.0:
        return f"the row direction must be at least 0 and below 180 degrees (asked for {s.row_angle:g})"
    return ""


def tidy(s: RobotSettings) -> RobotSettings:
    """The same settings with the float noise of the 32-bit message taken off (0.2f is 0.200000003)."""
    return replace(s, cleaning_speed=round(float(s.cleaning_speed), 4), manual_speed=round(float(s.manual_speed), 4),
                   row_spacing=round(float(s.row_spacing), 4), row_angle_auto=bool(s.row_angle_auto),
                   row_angle=round(float(s.row_angle), 3))


def load(path: str) -> Tuple[RobotSettings, str]:
    """(the settings in the file, a problem found). A missing file gives the defaults and no
    problem; a broken file, or one with values outside the limits, gives the defaults and says why."""
    if not os.path.exists(path):
        return RobotSettings(), ""
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError("it does not hold a list of settings")
        known = {f.name for f in fields(RobotSettings)}
        values = {}
        for name, value in data.items():
            if name not in known:
                continue  # a setting of a newer version: kept out, not an error
            values[name] = bool(value) if name == "row_angle_auto" else float(value)
        s = tidy(replace(RobotSettings(), **values))
    except Exception as exc:
        return RobotSettings(), f"could not read {path}: {exc}; using the defaults"
    why = check(s)
    if why:
        return RobotSettings(), f"{path}: {why}; using the defaults"
    return s, ""


def save(path: str, s: RobotSettings) -> None:
    """Write the settings so that a crash halfway leaves the old file whole."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="." + os.path.basename(path) + ".", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            f.write("# The robot's settings, as last changed from the app. Delete this file for the defaults.\n")
            yaml.safe_dump(asdict(s), f, sort_keys=False)
        os.replace(temporary, path)
    except BaseException:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise


def describe(s: RobotSettings) -> str:
    direction = "auto" if s.row_angle_auto else f"{s.row_angle:g}°"
    return (f"cleaning {s.cleaning_speed:.2f} m/s, joystick {s.manual_speed:.2f} m/s, rows {s.row_spacing * 100:.0f} cm apart, "
            f"row direction {direction}")


def changes(old: Optional[RobotSettings], new: RobotSettings) -> str:
    """What is different, in a few words ("" if nothing)."""
    if old is None:
        return describe(new)
    parts = []
    if abs(old.cleaning_speed - new.cleaning_speed) > _EPS:
        parts.append(f"cleaning speed {new.cleaning_speed:.2f} m/s")
    if abs(old.manual_speed - new.manual_speed) > _EPS:
        parts.append(f"joystick speed {new.manual_speed:.2f} m/s")
    if abs(old.row_spacing - new.row_spacing) > _EPS:
        parts.append(f"rows {new.row_spacing * 100:.0f} cm apart")
    if old.row_angle_auto != new.row_angle_auto or (not new.row_angle_auto and abs(old.row_angle - new.row_angle) > _EPS):
        parts.append("row direction " + ("auto" if new.row_angle_auto else f"{new.row_angle:g}°"))
    return ", ".join(parts)
