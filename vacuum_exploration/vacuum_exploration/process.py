"""Starting and ending a child program: the explorer. No ROS imports.

Three rules, all learnt from what can go wrong with a child program in a node like this:

  * It must die with the node. Linux can send a child a signal when its parent dies (the
    "parent-death signal"), but it ties that to the THREAD that started the child, not the
    process. So every child is started from one thread that lives as long as the node (Launcher),
    through `setpriv --pdeathsig KILL`. Python's preexec_fn is not used: it is unsafe in a
    program with threads.
  * Its output goes to a log file. An output pipe nobody reads fills up and freezes the child.
  * It is ended in steps, each with a time limit: asked to stop (the explorer cancels its Nav2
    goal and quits), then told to terminate, then killed. It always ends.
"""

import os
import queue
import signal
import subprocess
import threading
import time
from typing import Callable, List, Optional


class Launcher:
    """Starts programs from its own thread, which lives as long as this process."""

    def __init__(self):
        self._requests: "queue.Queue" = queue.Queue()
        self.thread_ident: Optional[int] = None
        self.last_start_thread: Optional[int] = None
        started = threading.Event()
        self._thread = threading.Thread(target=self._serve, args=(started,), name="explorer-launcher", daemon=True)
        self._thread.start()
        started.wait()

    def start(self, args: List[str], log_path: str, timeout: float = 10.0) -> subprocess.Popen:
        """Start `args` with the parent-death signal set; output to `log_path` (appended).
        Raises what starting it raised (a missing program, say)."""
        done = threading.Event()
        box = {}
        self._requests.put((args, log_path, box, done))
        if not done.wait(timeout):
            raise RuntimeError("the launcher thread did not start the program in time")
        if "error" in box:
            raise box["error"]
        return box["popen"]

    def _serve(self, started: threading.Event) -> None:
        self.thread_ident = threading.get_ident()
        started.set()
        while True:
            args, log_path, box, done = self._requests.get()
            try:
                os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
                with open(log_path, "ab") as log:
                    log.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} starting: {' '.join(args)}\n".encode())
                    log.flush()
                    box["popen"] = subprocess.Popen(
                        ["setpriv", "--pdeathsig", "KILL", *args],
                        stdin=subprocess.DEVNULL,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,  # a Ctrl+C in the launch terminal is for the node, not it
                        close_fds=True,
                    )
                self.last_start_thread = threading.get_ident()
            except Exception as exc:  # reported to the caller
                box["error"] = exc
            finally:
                done.set()


class Child:
    """One started program, and how to end it."""

    def __init__(self, popen: subprocess.Popen, ask_stop: Callable[[], bool]):
        self.popen = popen
        self.pid = popen.pid
        self._ask_stop = ask_stop  # asks the program to stop and quit; True if it said yes
        self._lock = threading.Lock()

    def exit_code(self) -> Optional[int]:
        """None while it runs; its exit code once it has ended (minus the signal if killed)."""
        return self.popen.poll()

    def end(self, ask_wait: float = 3.0, term_wait: float = 3.0, kill_wait: float = 3.0) -> str:
        """End it, whatever it takes. Returns how it ended, in words. Safe to call twice."""
        with self._lock:
            code = self.popen.poll()
            if code is not None:
                return f"had already exited (exit code {code})"
            try:
                asked = self._ask_stop()
            except Exception:
                asked = False
            if asked and self._wait(ask_wait):
                return "quit when asked"
            self.popen.terminate()
            if self._wait(term_wait):
                return "terminated" + ("" if asked else " (it did not answer the stop request)")
            self.popen.kill()
            self._wait(kill_wait)
            return "killed: it ignored the stop request and the terminate signal"

    def _wait(self, seconds: float) -> bool:
        try:
            self.popen.wait(timeout=seconds)
            return True
        except subprocess.TimeoutExpired:
            return False


def kill_leftovers(marker: str) -> List[int]:
    """Kill every process of this user, other than this one, whose command line contains
    `marker`: explorers left from a node that died before it could end them. Returns their pids."""
    killed = []
    me = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
            if marker not in cmdline or os.stat(f"/proc/{entry}").st_uid != os.getuid():
                continue
            os.kill(int(entry), signal.SIGKILL)
            killed.append(int(entry))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return killed
