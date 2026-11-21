# Lets the tests import vacuum_gateway (and the mode manager's map_files, which zones.py uses) from
# the source tree, so they also run without a build:
#   python3 -m pytest ros2_ws/src/motebot_robot/vacuum_gateway/test
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC / "vacuum_gateway"))
sys.path.insert(0, str(_SRC / "vacuum_mode_manager"))
