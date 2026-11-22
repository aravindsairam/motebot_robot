# MOTEbot robot software

The brain of [MOTEbot](https://github.com/aravindsairam/motebot), a robot vacuum cleaner built and tested fully in simulation. It runs on ROS 2 Humble and Nav2.

## What the robot can do

- **Map a home.** Drive it around yourself, or let it explore every room alone.
- **Go anywhere.** It knows where it is on the saved map and drives to any point you pick.
- **Clean.** It sweeps in neat back and forth rows, one room at a time, around furniture and no-go zones.
- **Your settings.** Cleaning speed, joystick speed, row spacing and row direction, kept on the robot.
- **Rooms.** It splits the map at the doorways, drives to each room and names it by looking. Clean one room, or ask how clean it is.
- **Chat.** Ask in your own words: what it sees, to go to the sofa, to clean the kitchen. It acts only through the same robot services the app uses, so the safety rules still hold.
- **Dock.** One command sends it home to park on its charger.
- **Stay safe.** It stops at stairs and backs away from bumps, whoever is driving.
- **Talk to the phone.** The Android app gets the live map, the camera and full control.

## How it is built

- **One mode at a time.** A mode manager decides what the robot is doing and switches the right parts on and off.
- **One hand on the wheels.** Every drive command goes through a safety gate, the only part allowed to move the robot.
- **Safe when things break.** If a part stops answering, the robot stops and waits for a reset.

## Setup

You need Ubuntu 22.04 and ROS 2 Humble. Get the code through the [main repo](https://github.com/aravindsairam/motebot), which brings every part in the right version:

```bash
git clone --recursive https://github.com/aravindsairam/motebot
cd motebot/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
pip3 install --user 'websockets>=13' 'protobuf>=6.31' aiortc aiohttp 'anthropic>=0.93' torch transformers
colcon build --symlink-install
```

The first build takes a few minutes. The object finder downloads its model (about 700 MB) the first time it starts.

The chat needs an Anthropic API key, kept in a file only you can read:

```bash
mkdir -p ~/.config/motebot
nano ~/.config/motebot/anthropic_api_key   # paste the key and save
chmod 600 ~/.config/motebot/anthropic_api_key
```

## Run

Start the simulator first. The main repo shows how. Then start the robot:

```bash
source install/setup.bash
ros2 launch vacuum_bringup vacuum.launch.py
```

The robot wakes up in `idle`. Open a second terminal and tell it what to do:

```bash
ros2 run vacuum_mode_manager mode navigation    # then pick a goal in RViz with "2D Goal Pose"
ros2 run vacuum_coverage clean                  # clean the whole home, then dock
ros2 run vacuum_mode_manager mode docking       # drive back to the charger
ros2 run vacuum_exploration explore             # make a new map without help
ros2 run vacuum_mode_manager mode               # show what the robot is doing now
ros2 run vacuum_mode_manager settings           # show the settings (add row_angle=45 and the like to change them)
ros2 run vacuum_assistant ask "how clean is the kitchen?"   # chat with the robot
```

Add `world:=home` to the launch line when the simulator runs the bigger home.

## Thanks to

[Nav2](https://docs.nav2.org), [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2), [apriltag_ros](https://github.com/christianrauch/apriltag_ros) and [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO) for finding things in the camera picture.

## License

Apache 2.0
