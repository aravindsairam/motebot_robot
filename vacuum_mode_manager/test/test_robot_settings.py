"""The settings a user may change: limits, checks, and the file they are kept in."""

import re
from dataclasses import replace
from pathlib import Path

import pytest

from vacuum_mode_manager import robot_settings
from vacuum_mode_manager.robot_settings import RobotSettings, check, load, save

MSG = Path(__file__).resolve().parents[2] / "vacuum_interfaces" / "msg" / "RobotSettings.msg"


def test_the_limits_here_are_the_messages_limits():
    constants = dict(re.findall(r"^float32 ([A-Z_]+) = ([0-9.]+)", MSG.read_text(), re.M))
    assert constants, f"no constants found in {MSG}"
    for name, value in constants.items():
        assert getattr(robot_settings, name) == pytest.approx(float(value)), name


def test_the_defaults_are_todays_behaviour_and_allowed():
    s = RobotSettings()
    assert (s.cleaning_speed, s.manual_speed, s.row_spacing, s.row_angle_auto) == (0.20, 0.30, 0.25, True)
    assert check(s) == ""
    assert s.manual_turn_speed == pytest.approx(1.2)  # the app's joystick turned at 1.2 rad/s before
    assert s.row_direction == "auto"
    assert replace(s, row_angle_auto=False, row_angle=37.5).row_direction == "37.5"


@pytest.mark.parametrize("change, words", [
    (dict(cleaning_speed=0.05), "cleaning speed must be between 0.10 and 0.30 m/s (asked for 0.05)"),
    (dict(cleaning_speed=0.5), "cleaning speed"),
    (dict(cleaning_speed=float("nan")), "not a number"),
    (dict(manual_speed=0.35), "joystick speed must be between 0.10 and 0.30 m/s"),
    (dict(row_spacing=0.35), "row spacing must be between 10 and 30 cm (asked for 35)"),
    (dict(row_spacing=0.05), "row spacing"),
    (dict(row_spacing=0.17), "whole number of 5 cm steps"),
    (dict(row_angle=180.0), "below 180 degrees"),
    (dict(row_angle=-5.0), "at least 0"),
])
def test_settings_outside_the_limits_are_refused_with_a_reason(change, words):
    assert words in check(replace(RobotSettings(), **change))


def test_the_ends_of_every_range_are_allowed_and_32_bit_noise_does_not_matter():
    for cleaning, manual, spacing, angle in ((0.10, 0.10, 0.10, 0.0), (0.30, 0.30, 0.30, 179.5)):
        assert check(RobotSettings(cleaning, manual, spacing, False, angle)) == ""
    noisy = RobotSettings(cleaning_speed=0.30000001192, manual_speed=0.1, row_spacing=0.15000000596, row_angle=45.0)
    assert check(noisy) == ""
    assert robot_settings.tidy(noisy).row_spacing == 0.15


def test_saved_settings_come_back_and_a_missing_file_gives_the_defaults(tmp_path):
    path = str(tmp_path / "robot_settings.yaml")
    assert load(path) == (RobotSettings(), "")
    s = RobotSettings(cleaning_speed=0.15, manual_speed=0.2, row_spacing=0.15, row_angle_auto=False, row_angle=45.0)
    save(path, s)
    assert load(path) == (s, "")
    assert not [p for p in tmp_path.iterdir() if p.name != "robot_settings.yaml"]  # no temporary file left


@pytest.mark.parametrize("text, words", [
    ("cleaning_speed: [1, 2\n", "could not read"),
    ("- a list\n", "could not read"),
    ("cleaning_speed: 0.9\n", "cleaning speed must be between"),
    ("cleaning_speed: fast\n", "could not read"),
])
def test_a_broken_file_gives_the_defaults_and_says_why(tmp_path, text, words):
    path = tmp_path / "robot_settings.yaml"
    path.write_text(text)
    s, problem = load(str(path))
    assert s == RobotSettings()
    assert words in problem


def test_a_file_with_a_setting_this_version_does_not_know_still_loads(tmp_path):
    path = tmp_path / "robot_settings.yaml"
    path.write_text("cleaning_speed: 0.25\nsuction: high\n")
    assert load(str(path)) == (RobotSettings(cleaning_speed=0.25), "")


def test_what_changed_is_said_in_a_few_words():
    old = RobotSettings()
    assert robot_settings.changes(old, old) == ""
    new = replace(old, cleaning_speed=0.25, row_angle_auto=False, row_angle=45.0)
    assert robot_settings.changes(old, new) == "cleaning speed 0.25 m/s, row direction 45°"
    assert "joystick 0.30 m/s" in robot_settings.changes(None, old)
