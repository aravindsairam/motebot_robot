from vacuum_mode_manager.world_check import WAITING, world_problem


def test_no_world_given_means_nothing_to_compare():
    assert world_problem("", None) == ""
    assert world_problem("", "home") == ""


def test_same_world_is_fine():
    assert world_problem("house", "house") == ""
    assert world_problem("home", "home") == ""


def test_not_heard_yet_is_not_a_problem_but_something_to_wait_for():
    assert world_problem("home", None) == ""
    assert "/sim/world" in WAITING


def test_other_world_names_both():
    why = world_problem("house", "home")
    assert why.startswith("wrong world")
    assert "'home'" in why and "'house'" in why


def test_nothing_heard_after_start_up_is_a_fault_that_does_not_clear():
    # start-up waits for /sim/world; once it is over, silence must keep the robot in failsafe
    # (else Reset would clear it and the robot would run on maps of a world it never checked)
    why = world_problem("home", None, waiting_over=True)
    assert why.startswith("the simulator's world is unknown")
    assert world_problem("home", None, waiting_over=False) == ""
    assert world_problem("", None, waiting_over=True) == ""
    assert world_problem("home", "home", waiting_over=True) == ""
