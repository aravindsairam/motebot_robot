import math

from vacuum_gateway import drive


def test_one_phone_drives_until_it_has_been_quiet_for_a_second():
    lock = drive.DriverLock(quiet_handover=1.0)
    assert lock.claim("a", 0.0)
    assert lock.claim("a", 0.05)
    assert not lock.claim("b", 0.5)
    assert not lock.claim("b", 1.04)
    assert lock.claim("b", 1.06)  # a sent nothing for 1.01 s
    assert lock.driver == "b"
    assert not lock.claim("a", 1.1)


def test_a_phone_that_leaves_frees_the_robot_at_once():
    lock = drive.DriverLock()
    assert lock.claim("a", 0.0)
    assert not lock.release("b")
    assert lock.release("a")
    assert lock.driver is None
    assert lock.claim("b", 0.1)


def test_speeds_are_kept_within_the_base_limits():
    assert drive.clamp_speeds(0.1, -0.5) == (0.1, -0.5)
    assert drive.clamp_speeds(5.0, -9.0) == (drive.MAX_FORWARD, -drive.MAX_TURN)
    assert drive.clamp_speeds(-5.0, 9.0) == (-drive.MAX_FORWARD, drive.MAX_TURN)
    assert drive.clamp_speeds(math.nan, 0.0) is None
    assert drive.clamp_speeds(0.0, math.inf) is None
