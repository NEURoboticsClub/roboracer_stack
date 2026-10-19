#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# AutoDRIVE simulator bring-up: the planner, Foxglove and Cartographer.
#
# MAPPING. Cartographer 2D, driven by LIDAR SCAN MATCHING ONLY - no odometry,
# no IMU, and never the bridge's ground-truth pose (see
# config/cartographer_sim.lua for the frames). The planner itself is purely
# reactive and does not read the map. Turn mapping off with `mapping:=false`.
#
# The bridge stamps with the wall clock and nothing publishes /clock, so
# use_sim_time stays false throughout.
#
# WHAT RUNS WHERE. This launch file lives in the ONBOARD container and can only
# start packages from ros_ws. The AutoDRIVE bridge is a different package
# (autodrive_roboracer) in the DEVKIT container, so it is not launched here:
#
#   devkit container   ros2 launch autodrive_roboracer bringup_headless.launch.py
#   onboard container  ros2 launch neo_telemetry neo_telemetry_sim_launch.py
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
#   planner --> /visualization_markers                    the chosen gap+heading
#
# The scan is stamped in the 'lidar' frame and the bridge broadcasts
# lidar -> roboracer_1 -> world, so the markers (stamped with the scan's own
# frame) line up in Foxglove with no static transform from this file. Set the
# 3D panel's display frame to 'world' for those.
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
    # Useful topics in the 3D panel: /visualization_markers (the planner's
    # chosen gap and heading), the scan above, and /autodrive/roboracer_1/imu.
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
        executable="planner_gap_follow",
        name="gap_follower_node",
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

            # ---- throttle ----------------------------------------------------
            # AutoDRIVE's throttle is normalized [-1, 1] and open-loop. This is
            # the TOP speed, held only with slowdown_distance or more of clear
            # track ahead; the throttle ramp below brings it down for corners.
            # Raise in small steps.
            #
            # MAPPING SPEED FOR NOW: 0.06. Scan matching wants small steps
            # between scans; 0.10 still ran ~2.2 m/s (throttle-to-speed is not
            # linear at the low end), against the real car's ~1.2 m/s mapping
            # pace. Racing values were 0.25 here with corner_erpm 3600.
            "sim_cruise_throttle": 0.06,
            # Brake = coast. On the real car the Brake state only eases the
            # speed down to brake_erpm (4000 of 6000 eRPM); a negative value
            # here is REVERSE throttle in AutoDRIVE, and -0.05 against a 0.1
            # cruise all but stopped the car at every corner entry.
            "sim_brake_throttle": 0.05,
            # No ESC dead zone in the sim, so no pulsing. Left at the real car's
            # 0.1 this would chop the corner throttle into a 2 Hz stutter,
            # because corner throttle is a FRACTION of cruise and lands below
            # the floor.
            "sim_throttle_floor": 0.0,
            # Corner throttle = sim_cruise_throttle * corner_erpm / cruise_erpm.
            # The eRPM names are the real car's, but in sim mode only their
            # RATIO is read: 0.36 here, which holds corners at 0.09 throttle -
            # the speed they were taken at cleanly before the top speed went
            # up. Keep the product sim_cruise_throttle x ratio near 0.09 when
            # raising cruise, or corners speed up with it.
            "cruise_erpm": 10000.0,
            # MAPPING: 0.8, so corners barely slow (0.08 vs 0.10) and the scan
            # matcher sees a near-constant speed. Racing value was 3600.
            "corner_erpm": 8000.0,

            # ---- steering ----------------------------------------------------
            # Requested heading at which the command reaches full lock (+-1.0).
            # This is a GAIN: lower turns in harder for the same heading error.
            # It is also the denominator the corner speed gate uses, so the two
            # stay consistent.
            "max_steer_angle": 0.80,
            # Flip to -1.0 if the car mirrors every turn. Both the AutoDRIVE
            # steering command and the LaserScan count LEFT as positive, so +1.0
            # should be right - but it is the one thing that cannot be confirmed
            # without watching it drive.
            "sim_steer_sign": 1.0,
            # Past this fraction of full lock, cruise drops to corner speed
            # however much room there is ahead.
            "corner_lock_fraction": 0.30,

            # ---- geometry ----------------------------------------------------
            # The AutoDRIVE RoboRacer matches the real car closely enough that
            # the trackside values carry over. The sim lidar is 10 m range, same
            # as the planner's internal clamp.
            "corridor_half_width": 0.18,
            "corner_distance": 2.0,
            # Throttle ramp: full cruise with 6 m or more clear ahead, easing
            # linearly down to corner speed at corner_distance (2 m). AutoDRIVE
            # has no real brake here (see sim_brake_throttle), so the car sheds
            # speed by drag alone and needs the long run-in at 0.25 cruise.
            # Shorten it if the car slows too early; lengthen it if it still
            # arrives hot. The sim lidar reads at most 10 m, so keep it below.
            "slowdown_distance": 6.0,
            "front_cone_half_angle_deg": 20.0,

            # ---- closing-speed brake -----------------------------------------
            # Engage within brake_activation_distance when closing at
            # brake_closing_speed (m/s) or faster; release at
            # brake_release_speed. On a head-on approach closing speed IS the
            # car's speed, so the release value is what the brake slows the car
            # to. At 0.8/0.6 m/s the sim car is faster than that on EVERY corner
            # entry, so the brake fired at every corner on top of the corner
            # slowdown. 2.0/1.5 leaves corners to the Corner state and keeps the
            # brake for genuine overspeed. Activation is pulled in from 3.0 to
            # corner_distance so it does not fire before the corner starts.
            "brake_activation_distance": 2.0,
            "brake_closing_speed": 2.0,
            "brake_release_speed": 1.5,
            "min_gap_depth": 0.8,
            "min_gap_width": 0.40,
            "deep_band": 0.20,

            # ---- corner cutting ----------------------------------------------
            # The sim car is ~0.27 m wide at the wheels; 0.35 pads the inside
            # apex well past the real car's 0.22, aiming the car further off
            # it. Was 0.30, which still took corners a little tight.
            "bubble_radius": 0.35,
            # Hold straight rather than turn toward a wall closer than this
            # beside the car. The planner aims the LIDAR past the apex, but the
            # car pivots on its rear axle 0.27 m behind it, so the flank cuts
            # inside that line - this is what was hitting the inside walls.
            "side_clearance": 0.25,
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
            os.path.join(get_package_share_directory("neo_telemetry"), "config"),
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
        default_value="true",
        description="Run Cartographer on the sim lidar, scan matching only.",
    ))
    ld.add_action(foxglove_node)
    ld.add_action(planning_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    return ld
