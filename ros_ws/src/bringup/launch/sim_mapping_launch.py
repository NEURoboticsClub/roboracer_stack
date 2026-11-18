#!/usr/bin/env python3
"""Build a map of the AutoDRIVE sim track with slam_toolbox, using the
simulator's own odometry.

This is the sim counterpart of mapping_launch.py. sim_launch.py's mapping:=true
is a DIFFERENT experiment: Cartographer on scan matching alone, no odometry.
Do not run the two together. Both publish /map and the 'map' frame, and the TF
tree ends up with two parents for 'world'.

    ODOMETRY

slam_toolbox takes odometry from TF only (odom_frame -> base_frame), never from
an Odometry topic. The bridge already broadcasts world -> roboracer_1, the same
pose it publishes on /autodrive/roboracer_1/odom. So this file starts no
odometry node and no static transforms. config/mapper_params_sim.yaml points
slam_toolbox at those frames, and the tree becomes:

    map -> world -> roboracer_1 -> lidar
    ^ slam_toolbox  ^ bridge       ^ bridge

That odometry is ground truth (no slip, no drift), so expect a clean map and a
map -> world that barely moves.

    HOW TO RUN

    host               ./autodrive_simulator.sh
    devkit container   ros2 launch autodrive_roboracer bringup_headless.launch.py
    onboard container  ros2 launch bringup sim_mapping_launch.py

  By default the planner drives the lap, capped at sim_max_throttle. To drive by
  hand instead, pass planner:=false and run the devkit's teleop:
    ros2 run autodrive_roboracer teleop_keyboard

  Watch it in Foxglove at ws://localhost:8765. In the 3D panel set the display
  frame to 'map' and add /map and /autodrive/roboracer_1/lidar.

    WHEN THE LAP IS DONE

  Leave everything running and save. Only ros_ws/src is mounted from the host
  (compose.yaml), so save under it or the map disappears with the container.
  Both calls want an absolute path with no extension:

    mkdir -p /home/ros_ws/src/bringup/maps
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ros_ws/src/bringup/maps/sim_track'}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ros_ws/src/bringup/maps/sim_track'}"

  (One line each, so they paste straight into a shell. These are for THIS
  launch file's slam_toolbox; sim_launch.py mapping:=true runs Cartographer,
  which saves differently - see that file.)

  save_map writes the .pgm + .yaml occupancy grid. serialize_map writes the
  .data + .posegraph that slam_toolbox can reload to localize against. Save both.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The bridge's lidar topic. The planner listens on /scan and is remapped onto
# this; slam_toolbox reads it directly (scan_topic in mapper_params_sim.yaml).
SIM_SCAN_TOPIC = "/autodrive/roboracer_1/lidar"


def generate_launch_description():
    planner_arg = DeclareLaunchArgument(
        "planner",
        default_value="true",
        description="Let the planner drive the mapping lap. false = drive it "
                    "yourself with the devkit's teleop_keyboard.",
    )
    # Same meaning as in sim_launch.py: what the algorithm's throttle +1.0
    # becomes. 0.10 is ~2.2 m/s. Odometry is perfect in the sim, so speed costs
    # the map much less than on the car; the main limit is the scan matcher
    # keeping up.
    max_throttle_arg = DeclareLaunchArgument(
        "sim_max_throttle",
        default_value="0.2",
        description="AutoDRIVE throttle [0, 1] at the algorithm's throttle +1.0.",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",  # reachable from outside the container
            # An occupancy grid of the whole track, re-sent every
            # map_update_interval, overruns the 10 MB default and the bridge
            # drops the client mid-map.
            "send_buffer_limit": 100000000,
            "max_qos_depth": 10,
            "use_compression": False,
        }],
    )

    # Identical to sim_launch.py apart from the throttle cap being an argument.
    planning_node = Node(
        package="planner",
        executable="planner",
        name="planner",
        output="screen",
        condition=IfCondition(LaunchConfiguration("planner")),
        # No IMU remap: type mismatch, see the note in sim_launch.py.
        remappings=[("/scan", SIM_SCAN_TOPIC)],
        parameters=[{
            "output_mode": "sim",
            "sim_max_throttle": ParameterValue(
                LaunchConfiguration("sim_max_throttle"), value_type=float),
            "sim_max_brake": 0.25,
            "sim_steer_sign": 1.0,
        }],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("bringup"),
                         "config", "mapper_params_sim.yaml"),
            {"use_sim_time": False},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(planner_arg)
    ld.add_action(max_throttle_arg)
    ld.add_action(foxglove_node)
    ld.add_action(planning_node)
    ld.add_action(slam_toolbox_node)
    return ld
