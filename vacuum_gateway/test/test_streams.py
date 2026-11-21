from vacuum_gateway.streams import HOLD, SEND, SKIP, Stream


def make():
    return Stream("test", repeat_interval=0.5, replay_max_age=2.0)


def test_first_copy_is_sent():
    assert make().offer(b"a", 0.0) == SEND


def test_unchanged_copy_waits_for_the_repeat_interval():
    s = make()
    assert s.offer(b"a", 0.0) == SEND
    assert s.offer(b"a", 0.1) == SKIP
    assert s.offer(b"a", 0.49) == SKIP
    assert s.offer(b"a", 0.5) == SEND
    assert s.offer(b"a", 0.9) == SKIP


def test_changed_copy_is_sent_at_once():
    s = make()
    assert s.offer(b"a", 0.0) == SEND
    assert s.offer(b"b", 0.01) == SEND
    assert s.offer(b"a", 0.02) == SEND


def test_held_back_copy_still_counts_as_latest():
    s = make()
    s.offer(b"a", 0.0)
    s.offer(b"a", 0.3)  # not sent, but it is the newest
    assert s.replay(2.2) == b"a"  # 1.9 s after the newest copy, 2.2 s after the one sent


def test_nothing_to_replay_before_the_first_copy():
    assert make().replay(0.0) is None


def test_old_copy_is_not_replayed():
    s = make()
    s.offer(b"a", 10.0)
    assert s.replay(12.0) == b"a"
    assert s.replay(12.01) is None


def test_replay_gives_the_newest_copy():
    s = make()
    s.offer(b"a", 0.0)
    s.offer(b"b", 0.1)
    assert s.replay(0.2) == b"b"


# ---------------------------------------------------------------- the map's kind of stream
def map_stream():
    return Stream("map", repeat_interval=None, replay_max_age=None, min_interval=1.0)


def test_an_unchanged_map_is_never_sent_again():
    s = map_stream()
    assert s.offer(b"m1", 0.0) == SEND
    assert s.offer(b"m1", 5.0) == SKIP
    assert s.offer(b"m1", 500.0) == SKIP


def test_the_map_is_always_given_to_a_late_phone():
    s = map_stream()
    s.offer(b"m1", 0.0)
    assert s.replay(3600.0) == b"m1"


def test_a_change_too_soon_is_held_and_the_newest_released():
    s = map_stream()
    assert s.offer(b"m1", 0.0) == SEND
    assert s.offer(b"m2", 0.3) == HOLD
    assert abs(s.hold_delay(0.3) - 0.7) < 1e-9
    assert s.offer(b"m3", 0.6) == HOLD
    assert s.release(1.0) == b"m3"  # the newest, not the one that started the hold
    assert s.release(1.1) is None  # nothing more held
    assert s.offer(b"m4", 1.5) == HOLD  # 0.5 s after the released copy
    assert s.release(2.0) == b"m4"


def test_a_held_copy_that_changed_back_is_not_sent():
    s = map_stream()
    s.offer(b"m1", 0.0)
    assert s.offer(b"m2", 0.3) == HOLD
    s.offer(b"m1", 0.6)  # back to what the phones already have
    assert s.release(1.0) is None
