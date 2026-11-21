"""The gateway needs two Python packages from pip, because Ubuntu 22.04's are too old:
websockets (13 or newer, for its asyncio server) and protobuf (6.31 or newer, to read the code
tools/gen_proto.sh generates). Check them before anything imports them, and say plainly what to
install instead of failing with an import error that points nowhere.
"""

import sys

NEEDED = {"websockets": (13, 0), "google.protobuf": (6, 31)}
INSTALL = "pip3 install --user 'websockets>=13' 'protobuf>=6.31'"


def _version(text: str):
    """(major, minor) from a version string. Always two numbers: (13,) would sort before (13, 0)."""
    parts = []
    for piece in (text.split(".") + ["0", "0"])[:2]:
        digits = "".join(ch for ch in piece if ch.isdigit())
        parts.append(int(digits) if digits else 0)
    return tuple(parts)


def problems():
    found = []
    for module, minimum in NEEDED.items():
        try:
            mod = __import__(module, fromlist=["__version__"])
        except ImportError:
            found.append(f"{module} is not installed")
            continue
        version = getattr(mod, "__version__", "0")
        if _version(version) < minimum:
            found.append(f"{module} {version} is too old (needs {minimum[0]}.{minimum[1]} or newer, from {mod.__file__})")
    return found


def check_or_exit():
    found = problems()
    if found:
        print("vacuum_gateway cannot start: " + "; ".join(found) + ". Install with: " + INSTALL, file=sys.stderr)
        sys.exit(1)
