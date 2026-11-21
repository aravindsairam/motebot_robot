"""Bring up the whole robot software stack once. The mode manager switches between modes.

The simulator must already be running (sim/scripts/run_sim.py) with the robot at its start pose
in front of the dock.

Everything is started here, but slam_toolbox, AMCL, map_server and the Nav2 servers are
lifecycle nodes that start switched off. The mode manager turns on what each mode needs.

Velocity commands all go through the safety gate, the only node that talks to the wheels:

    keyboard or app  >  /cmd_vel      >  safety_gate  >  /cmd_vel_safe  >  wheels
    Nav2             >  /cmd_vel_nav  >  safety_gate

Commands:

    ros2 run vacuum_mode_manager mode                 # show the current mode
    ros2 run vacuum_mode_manager mode mapping         # idle | mapping | navigation | manual | docking | failsafe (cleaning and exploring are entered by a job)
    ros2 run vacuum_mode_manager save_map house       # while in mapping mode (--replace to overwrite)
    ros2 run vacuum_mode_manager zones                # the map's no-go zones (add NAME X0 Y0 X1 Y1, delete NAME)
    ros2 run vacuum_coverage clean                    # clean everything, then dock (--area, --spot, --plan, --stay)
    ros2 run vacuum_exploration explore               # map the house by itself (--minutes N); Stop: mode mapping

Arguments:
    rviz:=false          do not open RViz
    exploration:=false   leave the exploration node out (phase 28 stands in for it)
    world:=home          the simulator's world (default house). Each world has its own maps folder:
                         ros2_ws/maps for the house, ros2_ws/maps_<world> for any other. The mode
                         manager compares it with the world the simulator announces on /sim/world
                         and stays in failsafe if they differ. tools/start_stack.sh sets it for you.
"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def maps_folder(workspace, world):
    """Each world keeps its own maps: a map of one world must never be loaded in another."""
    return os.path.join(workspace, "maps" if world == "house" else f"maps_{world}")


def _stack(context):
    world = LaunchConfiguration("world").perform(context)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", world):
        raise RuntimeError(f"world:={world!r} is not a world name (lower-case letters, digits, _)")
    share = get_package_share_directory("vacuum_bringup")
    # share is <workspace>/install/vacuum_bringup/share/vacuum_bringup
    workspace = os.path.normpath(os.path.join(share, "..", "..", "..", ".."))
    config = os.path.join(share, "config")
    trees = os.path.join(share, "behavior_trees")
    navigation_tree = os.path.join(trees, "navigate_to_pose_w_replanning_and_recovery.xml")
    # Exploring: the same tree without the spin and the back-up (see the file's own note).
    exploring_tree = os.path.join(trees, "navigate_to_pose_exploring.xml")
    slam_params = os.path.join(config, "slam_mapping.yaml")
    localization_params = os.path.join(config, "localization.yaml")
    nav2_params = os.path.join(config, "nav2.yaml")
    safety_params = os.path.join(config, "safety.yaml")
    apriltag_params = os.path.join(config, "apriltag.yaml")
    docking_params = os.path.join(config, "docking.yaml")
    coverage_params = os.path.join(config, "coverage.yaml")
    exploration_params = os.path.join(config, "exploration.yaml")  # the exploration node's and the explorer's
    # Each saved map keeps its no-go zones next to it, as <map>_zones.yaml (see map_files).
    maps_dir = maps_folder(workspace, world)
    # The dock's place on the map, from the one file that says it: no new no-go zone may come near it.
    with open(docking_params) as f:
        dock_pose = [float(v) for v in yaml.safe_load(f)["docking_server"]["ros__parameters"]["home_dock"]["pose"]]
    with open(os.path.join(share, "urdf", "vacuum.urdf")) as f:
        robot_description = f.read()

    sim_time = {"use_sim_time": True}

    def managed(package, executable, name, params, remappings=None, extra=None):
        """A lifecycle node the mode manager controls. If it crashes it is restarted (switched
        off); the mode manager notices the wrong state and puts the robot in failsafe."""
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[params, sim_time] + ([extra] if extra else []),
            remappings=remappings,
            respawn=True,
            respawn_delay=2.0,
        )

    # Nav2 does not drive the wheels directly: its commands go to the safety gate.
    to_gate = [("cmd_vel", "cmd_vel_nav")]

    return [
            # mapping
            managed("slam_toolbox", "async_slam_toolbox_node", "slam_toolbox", slam_params),
            # localization on a saved map
            managed("nav2_map_server", "map_server", "map_server", localization_params),
            managed("nav2_amcl", "amcl", "amcl", localization_params),
            # navigation and no-go zones
            managed("nav2_map_server", "map_server", "filter_mask_server", nav2_params),
            managed("nav2_map_server", "costmap_filter_info_server", "costmap_filter_info_server", nav2_params),
            managed("nav2_controller", "controller_server", "controller_server", nav2_params, to_gate),
            managed("nav2_planner", "planner_server", "planner_server", nav2_params),
            managed("nav2_behaviors", "behavior_server", "behavior_server", nav2_params, to_gate),
            # Nav2's stock behaviour trees with the goal checker named: nav2.yaml has a second one
            # for cleaning, and a tree that names none would make every navigate_to_pose abort.
            managed("nav2_bt_navigator", "bt_navigator", "bt_navigator", nav2_params, extra={
                "default_nav_to_pose_bt_xml": navigation_tree,  # the mode manager sets it again on every Nav2 start
                "default_nav_through_poses_bt_xml": os.path.join(trees, "navigate_through_poses_w_replanning_and_recovery.xml"),
            }),
            # docking: drives onto the dock, steering on the tag. Its commands share Nav2's input
            # to the safety gate; the two never drive at the same time.
            managed("opennav_docking", "opennav_docking", "docking_server", docking_params, to_gate),
            # the one node that decides which of the above are running. If it crashes, the safety
            # gate stops the robot (no mode, nobody drives) and the restarted manager returns to idle.
            Node(
                package="vacuum_mode_manager",
                executable="mode_manager",
                name="mode_manager",
                output="screen",
                parameters=[
                    sim_time,
                    {
                        "maps_dir": maps_dir,
                        "world": world,
                        "dock_pose": dock_pose,
                        "navigation_tree": navigation_tree,
                        "exploring_tree": exploring_tree,
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
            # the one node that commands the wheels. If it crashes, the base stops by itself when
            # commands end, and the mode manager goes to failsafe.
            Node(
                package="vacuum_safety",
                executable="safety_gate",
                name="safety_gate",
                output="screen",
                parameters=[safety_params, sim_time],
                respawn=True,
                respawn_delay=2.0,
            ),
            # cleaning jobs: plans the rows, drives them through Nav2, records what was cleaned, keeps
            # saved plans and the history. It asks the mode manager for cleaning mode and drives only
            # through Nav2 and the safety gate. If it dies during a clean, the mode manager goes to
            # failsafe within 2 s; the restarted node has no job, its plans and history are on disk.
            Node(
                package="vacuum_coverage",
                executable="cleaning_node",
                name="cleaning_node",
                output="screen",
                parameters=[coverage_params, sim_time, {"maps_dir": maps_dir}],
                respawn=True,
                respawn_delay=2.0,
            ),
            # exploring: the robot maps the house by itself. Owns the explorer (frontier_exploration_ros2)
            # as a child program, one per exploration, which dies with it; asks the mode manager for
            # mapping, then exploring, and mapping again at the end, so the map is always kept. If it
            # dies during an exploration, the mode manager goes to failsafe within 1 s (the explorer
            # with it), and Reset goes back to mapping with the map kept.
            Node(
                package="vacuum_exploration",
                executable="exploration_node",
                name="exploration_node",
                output="screen",
                parameters=[exploration_params, sim_time, {
                    "explorer_params": exploration_params,
                    "explorer_log": os.path.join(workspace, "..", "logs", "frontier_explorer.log"),
                }],
                condition=IfCondition(LaunchConfiguration("exploration")),
                respawn=True,
                respawn_delay=2.0,
            ),
            # passes messages between the robot and the Android app (WebSocket, port 9090). It
            # holds no robot logic: the robot works the same with or without it. Restarted if it
            # dies; a connected app reconnects by itself.
            Node(
                package="vacuum_gateway",
                executable="gateway",
                name="vacuum_gateway",
                output="screen",
                parameters=[{"maps_dir": maps_dir}],
                respawn=True,
                respawn_delay=2.0,
            ),
            # the camera as live video for the app (WebRTC; the phone posts its offer to port 8080).
            # Subscribes to the camera only while a phone watches. Restarted if it dies.
            Node(
                package="vacuum_video",
                executable="video",
                name="vacuum_video",
                output="screen",
                parameters=[sim_time],
                respawn=True,
                respawn_delay=2.0,
            ),
            # turns the tag sighting into the dock pose the docking server follows
            Node(
                package="vacuum_docking",
                executable="dock_pose_publisher",
                name="dock_pose_publisher",
                output="screen",
                parameters=[sim_time],
                respawn=True,
                respawn_delay=2.0,
            ),
            # finds the dock's AprilTag in the camera image
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[apriltag_params, sim_time],
                remappings=[("image_rect", "camera/image_raw"), ("camera_info", "camera/camera_info")],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}, sim_time],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "rviz", "vacuum.rviz")],
                parameters=[sim_time],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="log",
            ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="open RViz"),
            DeclareLaunchArgument("exploration", default_value="true", description="start the exploration node"),
            DeclareLaunchArgument("world", default_value="house", description="the simulator's world (house or home)"),
            OpaqueFunction(function=_stack),
        ]
    )
