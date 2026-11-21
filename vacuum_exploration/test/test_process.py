"""Starting and ending the explorer program, with stand-in programs. No ROS."""

import os
import signal
import subprocess
import sys
import textwrap
import time

import pytest

from vacuum_exploration import process

SLEEPER = [sys.executable, "-c", "import time; time.sleep(60)"]
STUBBORN = [sys.executable, "-c",
            "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); print('ready', flush=True); time.sleep(60)"]


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # a zombie (dead, not yet reaped) counts as gone
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().split(")")[-1].split()[0] != "Z"
    except FileNotFoundError:  # gone between the two looks
        return False


def wait_gone(pid, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if not alive(pid):
            return True
        time.sleep(0.05)
    return False


@pytest.fixture(scope="module")
def launcher():
    return process.Launcher()


def test_a_program_is_started_with_its_output_in_a_log_file(launcher, tmp_path):
    log = tmp_path / "explorer.log"
    popen = launcher.start([sys.executable, "-c", "print('hello from the child')"], str(log))
    assert popen.wait(10) == 0
    text = log.read_text()
    assert "hello from the child" in text
    assert "starting:" in text  # a header line per start, so runs can be told apart


def test_it_is_started_from_the_launchers_own_thread(launcher, tmp_path):
    popen = launcher.start(SLEEPER, str(tmp_path / "log"))
    try:
        assert launcher.thread_ident is not None
        assert launcher.last_start_thread == launcher.thread_ident
    finally:
        popen.kill()
        popen.wait()


def test_end_asks_first_and_is_done_when_the_program_quits(launcher, tmp_path):
    popen = launcher.start(SLEEPER, str(tmp_path / "log"))
    asked = []

    def ask_stop():
        asked.append(True)
        popen.terminate()  # stands in for the explorer quitting after its stop request
        return True

    child = process.Child(popen, ask_stop)
    how = child.end(ask_wait=3.0, term_wait=3.0, kill_wait=3.0)
    assert asked and "quit when asked" in how
    assert child.exit_code() is not None


def test_end_terminates_a_program_that_was_not_asked_or_did_not_quit(launcher, tmp_path):
    popen = launcher.start(SLEEPER, str(tmp_path / "log"))
    child = process.Child(popen, lambda: False)  # the stop request found no answer
    t = time.monotonic()
    how = child.end(ask_wait=0.5, term_wait=3.0, kill_wait=3.0)
    assert "terminated" in how
    assert time.monotonic() - t < 2.0
    assert child.exit_code() is not None


def test_end_kills_a_program_that_ignores_terminate(launcher, tmp_path):
    log = tmp_path / "log"
    popen = launcher.start(STUBBORN, str(log))
    end = time.monotonic() + 5.0
    while "\nready\n" not in log.read_text() and time.monotonic() < end:  # its own line, not the header
        time.sleep(0.05)
    child = process.Child(popen, lambda: False)
    how = child.end(ask_wait=0.2, term_wait=0.5, kill_wait=3.0)
    assert "killed" in how
    assert child.exit_code() == -signal.SIGKILL


def test_end_of_a_program_that_has_already_exited(launcher, tmp_path):
    popen = launcher.start([sys.executable, "-c", "raise SystemExit(3)"], str(tmp_path / "log"))
    popen.wait(10)
    child = process.Child(popen, lambda: True)
    assert "had already exited (exit code 3)" in child.end()


def test_the_program_dies_with_the_process_that_started_it(tmp_path):
    """Linux ties the parent-death signal to the thread that started the child: the launcher's
    thread lives as long as the process, so the child dies when the process dies, and only then."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script = textwrap.dedent(f"""
        import sys, time, threading
        sys.path.insert(0, {here!r})
        from vacuum_exploration import process
        launcher = process.Launcher()
        # started from a short-lived thread: must survive that thread, as the explorer does when
        # the exploration node's job thread ends
        out = []
        t = threading.Thread(target=lambda: out.append(launcher.start({SLEEPER!r}, {str(tmp_path / 'log')!r})))
        t.start(); t.join()
        print(out[0].pid, flush=True)
        time.sleep(60)
    """)
    parent = subprocess.Popen([sys.executable, "-c", script], stdout=subprocess.PIPE, text=True)
    try:
        child_pid = int(parent.stdout.readline())
        time.sleep(1.0)
        assert alive(child_pid), "the child died with the short-lived thread that asked for it"
        parent.kill()
        parent.wait()
        assert wait_gone(child_pid, 3.0), "the child outlived the process that started it"
    finally:
        parent.kill()


def test_leftovers_are_found_and_killed(tmp_path):
    marker = f"leftover-marker-{os.getpid()}"
    stray = subprocess.Popen([sys.executable, "-c", "import time, sys; time.sleep(60)", marker])
    try:
        time.sleep(0.2)
        killed = process.kill_leftovers(marker)
        assert stray.pid in killed
        assert stray.wait(5) == -signal.SIGKILL
        assert process.kill_leftovers(marker) == []
    finally:
        stray.kill()
