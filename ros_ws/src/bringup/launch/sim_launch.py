#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# AutoDRIVE simulator bring-up: the planner and Foxglove, plus Cartographer on
# request. The algorithm itself is planner_node's src/algorithm.cpp.
#
# MAPPING. Off by default; turn it on with `mapping:=true`. Cartographer 2D,
# driven by LIDAR SCAN MATCHING ONLY - no odometry, no IMU, and never the
# bridge's ground-truth pose (see config/cartographer_sim.lua for the frames).
# The planner does not read the map.
#
# The bridge stamps with the wall clock and nothing publishes /clock, so
# use_sim_time stays false throughout.
#
# WHAT RUNS WHERE. This launch file lives in the ONBOARD container and can only
# start packages from ros_ws. The AutoDRIVE bridge is a different package
# (autodrive_roboracer) in the DEVKIT container, so it is not launched here:
#
#   devkit container   ros2 launch autodrive_roboracer bringup_headless.launch.py
#   onboard container  ros2 launch bringup sim_launch.py
#   host               ./autodrive_simulator.sh   (the Unity binary)
#
# The simulator talks to the bridge over the websocket on :4567; the bridge and
# the planner talk to each other over ROS 2, so the two containers have to share
# a DDS domain (compose.yaml maps the ports, but `ipc: host` is commented out -
# if `ros2 topic list` in the onboard container does not show
# /autodrive/roboracer_1/lidar, that discovery is the thing to fix, not this
# file).
#
# TOPIC WIRING. What the bridge offers, and who uses it:
#
#   bridge  --> /autodrive/roboracer_1/lidar              sensor_msgs/LaserScan
#               remapped onto the planner's /scan below
#   planner --> /autodrive/roboracer_1/throttle_command   std_msgs/Float32 [-1,1]
#   planner --> /autodrive/roboracer_1/steering_command   std_msgs/Float32 [-1,1]
#
# The scan is stamped in the 'lidar' frame and the bridge broadcasts
# lidar -> roboracer_1 -> world, so it lines up in Foxglove with no static
# transform from this file. Set the 3D panel's display frame to 'world'.
#
# For the map, set the display frame to 'map' and add /map. The car and scan
# then draw where Cartographer estimates them, so a smeared or doubled wall is
# a scan-matching error, not the display.
# ---------------------------------------------------------------------------

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The bridge's lidar topic. The planner subscribes to /scan (the name the SICK
# publishes on the real car), so exactly one remap is what makes the same node
# binary work in both places.
SIM_SCAN_TOPIC = "/autodrive/roboracer_1/lidar"


def generate_launch_description():
    # Foxglove: connect a browser or Foxglove Studio to ws://localhost:8765
    #
    # Useful topics in the 3D panel: the scan above and
    # /autodrive/roboracer_1/imu.
    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",  # reachable from outside the container
            "max_qos_depth": 10,
            "use_compression": False,
        }],
    )

    planning_node = Node(
        package="planner_node",
        executable="planner",
        name="planner",
        output="screen",
        # The one rename that makes the algorithm work here unchanged.
        remappings=[("/scan", SIM_SCAN_TOPIC)],
        parameters=[{
            # Drive the AutoDRIVE command topics instead of the VESC ones. This
            # was a compile-time #define, so a sim run needed a source edit and
            # a rebuild; miss either and the planner came up publishing
            # /commands/motor/speed, which nothing in the sim subscribes to, and
            # the car just sat there.
            #
            # output_mode:=sim ALSO means no e-stop: the console is a real-car
            # thing and there is none in the sim stack, so the planner drives as
            # soon as scans start arriving.
            "output_mode": "sim",

            # AutoDRIVE's throttle is normalized [-1, 1] and open-loop. This is
            # what the algorithm's throttle +1.0 becomes: 0.10 runs ~2.2 m/s and
            # throttle-to-speed is not linear at the low end.
            "sim_max_throttle": 0.25,
            # What the algorithm's throttle -1.0 becomes. AutoDRIVE has no brake,
            # so this is sent as reverse throttle, and holding it backs the car up.
            "sim_max_brake": 0.25,
            # Flip to -1.0 if the car mirrors every turn. Both the AutoDRIVE
            # steering command and the LaserScan count LEFT as positive, so +1.0
            # should be right - but it is the one thing that cannot be confirmed
            # without watching it drive.
            "sim_steer_sign": 1.0,
        }],
    )

    # ---- mapping: Cartographer, scan matching only ---------------------------
    mapping = IfCondition(LaunchConfiguration("mapping"))

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        condition=mapping,
        arguments=[
            "-configuration_directory",
            os.path.join(get_package_share_directory("bringup"), "config"),
            "-configuration_basename", "cartographer_sim.lua",
        ],
        remappings=[("scan", SIM_SCAN_TOPIC)],
        parameters=[{"use_sim_time": False}],
    )

    # Cartographer keeps its map as submaps; this renders them onto /map.
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        condition=mapping,
        arguments=["-resolution", "0.05", "-publish_period_sec", "0.5"],
        parameters=[{"use_sim_time": False}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        "mapping",
        default_value="false",
        description="Run Cartographer on the sim lidar, scan matching only.",
    ))
    ld.add_action(foxglove_node)
    ld.add_action(planning_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    return ld
