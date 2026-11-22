"""What the robot has seen, and where, kept per map."""

from vacuum_assistant.memory import Memory


def test_a_thing_seen_again_nearby_is_the_same_thing_and_keeps_its_best_sighting(tmp_path):
    m = Memory(str(tmp_path / "home_things.yaml"))
    m.add("Sofa", 1.0, 1.0, 0.5, when=10.0)
    m.add("sofa", 1.4, 1.2, 0.7, when=20.0)   # the same sofa, seen better
    m.add("sofa", 1.2, 1.0, 0.4, when=30.0)   # the same, seen worse: place kept, time updated
    m.add("sofa", 6.0, 1.0, 0.6, when=40.0)   # another sofa
    assert [(t.x, t.y, t.score, t.seen) for t in m.things] == [(1.4, 1.2, 0.7, 30.0), (6.0, 1.0, 0.6, 40.0)]
    assert m.best("sofa").x == 1.4 and m.best("SOFA", min_score=0.65).x == 1.4 and m.best("sofa", min_score=0.9) is None


def test_names_match_in_part_and_the_file_round_trips(tmp_path):
    path = str(tmp_path / "home_things.yaml")
    m = Memory(path)
    m.add("dining chair", 2.0, 3.0, 0.5)
    m.add("chair", 5.0, 3.0, 0.45)
    assert m.best("chair").name == "chair" and m.best("dining chair").name == "dining chair"
    m.save()
    again = Memory(path)
    assert [(t.name, t.x) for t in again.things] == [("dining chair", 2.0), ("chair", 5.0)]
    (tmp_path / "broken_things.yaml").write_text("things: [{name: x\n")
    assert Memory(str(tmp_path / "broken_things.yaml")).things == []
