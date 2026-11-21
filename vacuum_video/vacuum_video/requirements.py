"""The video node needs aiortc (WebRTC, with av for encoding) and aiohttp (the offer endpoint)
from pip. Check them before anything imports them, and say plainly what to install."""

import sys

NEEDED = ("aiortc", "av", "aiohttp")
INSTALL = "pip3 install --user aiortc aiohttp"


def problems():
    found = []
    for module in NEEDED:
        try:
            __import__(module)
        except ImportError as exc:
            found.append(f"{module} cannot be imported ({exc})")
    return found


def check_or_exit():
    found = problems()
    if found:
        print("vacuum_video cannot start: " + "; ".join(found) + ". Install with: " + INSTALL, file=sys.stderr)
        sys.exit(1)
