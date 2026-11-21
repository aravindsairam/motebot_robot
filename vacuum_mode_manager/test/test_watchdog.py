from vacuum_mode_manager.watchdog import JobWatchdog, NodeHealth, TopicWatchdog


def test_topic_watchdog_is_quiet_during_startup_grace():
    w = TopicWatchdog("lidar", timeout=1.5, startup_grace=20.0, started_at=100.0)
    assert w.fault(110.0) is None
    assert not w.seen


def test_topic_watchdog_reports_data_that_never_arrived():
    w = TopicWatchdog("lidar", timeout=1.5, startup_grace=20.0, started_at=100.0)
    assert "no lidar data" in w.fault(121.0)


def test_topic_watchdog_reports_silence_and_recovers():
    w = TopicWatchdog("odometry", timeout=1.0, startup_grace=20.0, started_at=0.0)
    w.beat(5.0)
    assert w.seen
    assert w.fault(5.9) is None
    assert "odometry data stopped" in w.fault(6.5)
    w.beat(7.0)
    assert w.fault(7.5) is None


def test_node_health_ignores_a_single_missed_reply():
    h = NodeHealth("amcl", threshold=2)
    h.record(None, "active")
    assert h.not_responding() is None
    h.record("active", "active")
    h.record(None, "active")
    assert h.not_responding() is None  # the miss count restarted


def test_node_health_reports_a_silent_node():
    h = NodeHealth("amcl", threshold=2)
    h.record(None, "active")
    h.record(None, "active")
    assert h.not_responding() == "node amcl is not responding"


def test_node_health_reports_a_node_in_the_wrong_state():
    h = NodeHealth("controller_server", threshold=2)
    h.record("unconfigured", "active")
    assert h.wrong_state("active") is None
    h.record("unconfigured", "active")
    assert h.wrong_state("active") == "node controller_server is unconfigured, expected active"
    assert h.not_responding() is None


def test_node_health_accepts_any_state_when_nothing_is_expected():
    h = NodeHealth("slam_toolbox", threshold=2)
    h.record("inactive", None)
    h.record("inactive", None)
    assert h.wrong_state(None) is None


def test_node_health_reset():
    h = NodeHealth("amcl", threshold=2)
    h.record(None, "active")
    h.record(None, "active")
    h.reset()
    assert h.not_responding() is None


def test_a_job_node_is_only_watched_while_its_mode_is_on():
    w = JobWatchdog("cleaning node", timeout=2.0)
    assert w.fault(100.0, active=False) is None  # never heard, but not needed
    assert w.fault(100.0, active=True) is None  # just entered the mode: it gets the timeout from now
    assert w.fault(101.9, active=True) is None
    assert "cleaning node" in w.fault(102.1, active=True)
    w.beat(102.2)
    assert w.fault(103.0, active=True) is None
    assert w.fault(104.3, active=True) is not None
    assert w.fault(104.3, active=False) is None  # the mode was left: no longer a fault


def test_a_job_node_silent_since_before_the_mode_is_given_the_timeout_from_entry():
    w = JobWatchdog("cleaning node", timeout=2.0)
    w.beat(10.0)
    assert w.fault(50.0, active=False) is None
    assert w.fault(50.1, active=True) is None
    assert w.fault(52.2, active=True) is not None
    w.fault(53.0, active=False)
    assert w.fault(60.0, active=True) is None  # a new entry starts a new grace


def test_a_job_node_that_says_it_has_no_job_is_a_fault_in_its_mode():
    # A cleaning node restarted in the middle of a clean has no job, and must never start one.
    w = JobWatchdog("cleaning node", timeout=1.0)
    w.beat(10.0, has_job=True)
    assert w.fault(10.1, active=True) is None
    w.beat(10.2, has_job=False)
    assert "no job" in w.fault(10.3, active=True)
    assert w.fault(10.3, active=False) is None
