from vacuum_gateway import navigation
from vacuum_gateway import vacuum_pb2 as pb

State = pb.NavigationStatus.State
A, B = b"a" * 16, b"b" * 16
EXECUTING, SUCCEEDED, CANCELED, ABORTED, ACCEPTED, CANCELING = 2, 4, 5, 6, 1, 3


def test_nothing_to_say_before_the_first_goal():
    t = navigation.NavigationTracker()
    assert t.message() is None
    assert not t.on_status([])


def test_follows_the_newest_goal():
    t = navigation.NavigationTracker()
    assert t.on_status([(A, 10.0, EXECUTING)])
    assert t.message().goal_id == A and t.message().state == State.STATE_ON_THE_WAY
    # an older goal finishing does not matter once a newer one exists, in whatever order listed
    assert t.on_status([(B, 20.0, ACCEPTED), (A, 10.0, CANCELED)])
    assert (t.message().goal_id, t.message().state) == (B, State.STATE_ACCEPTED)


def test_states():
    t = navigation.NavigationTracker()
    for status, state in ((ACCEPTED, State.STATE_ACCEPTED), (EXECUTING, State.STATE_ON_THE_WAY),
                          (CANCELING, State.STATE_CANCELLING), (SUCCEEDED, State.STATE_REACHED),
                          (CANCELED, State.STATE_CANCELLED), (ABORTED, State.STATE_FAILED)):
        t.on_status([(A, 1.0, status)])
        assert t.message().state == state


def test_the_same_status_again_is_not_news():
    t = navigation.NavigationTracker()
    assert t.on_status([(A, 1.0, EXECUTING)])
    assert not t.on_status([(A, 1.0, EXECUTING)])


def test_distance_left_only_for_the_newest_goal_while_under_way():
    t = navigation.NavigationTracker()
    t.on_status([(A, 1.0, EXECUTING)])
    assert t.on_feedback(A, 3.2) and t.message().distance_remaining == 3.2
    assert not t.on_feedback(A, 3.18)  # under 5 cm of change: not news
    assert not t.on_feedback(B, 1.0)  # another goal's feedback
    t.on_status([(A, 1.0, SUCCEEDED)])
    assert t.message().distance_remaining == 0.0
    assert not t.on_feedback(A, 0.5)  # arrived: late feedback is ignored


def test_thin_path_keeps_the_ends_and_a_point_every_5_cm():
    pts = [(i * 0.01, 0.0) for i in range(101)]  # 1 m in 1 cm steps
    thin = navigation.thin_path(pts)
    assert thin[0] == (0.0, 0.0) and thin[-1] == (1.0, 0.0)
    assert len(thin) == 21  # every 5 cm, ends included
    assert navigation.thin_path([(1.0, 2.0)]) == [(1.0, 2.0)]
    assert navigation.thin_path([]) == []


def test_thin_path_caps_a_very_long_route():
    pts = [(i * 0.1, 0.0) for i in range(5000)]
    thin = navigation.thin_path(pts, max_points=100)
    assert len(thin) == 100 and thin[0] == pts[0] and thin[-1] == pts[-1]
