#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# AutoDRIVE simulator bring-up: the planner and nothing else.
#
# NO MAPPING. planner_gap_follow is purely reactive - it reads one LaserScan
# and publishes a command, with no odometry, no TF lookups and no nav_msgs
# dependency - so slam_toolbox never fed it anything. It used to be launched
# here anyway, which meant a sim run spent most of its CPU building a map that
# nothing read, and failed noisily on the missing odom -> base_link transform
# that no node in this file publishes.
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
# 3D panel's display frame to 'world'.
# ---------------------------------------------------------------------------

from launch import LaunchDescription
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
            # AutoDRIVE's throttle is normalized [-1, 1] and open-loop. 0.1 is
            # deliberately slow for a first run; raise it once the car is
            # following the track to see how the gap logic holds up with speed.
            "sim_cruise_throttle": 0.1,
            "sim_brake_throttle": -0.05,
            # No ESC dead zone in the sim, so no pulsing. Left at the real car's
            # 0.1 this would chop the corner throttle into a 2 Hz stutter,
            # because corner throttle is a FRACTION of cruise and lands below
            # the floor.
            "sim_throttle_floor": 0.0,
            # Corner throttle = sim_cruise_throttle * corner_erpm / cruise_erpm.
            # The eRPM names are the real car's, but in sim mode only their
            # RATIO is read: 0.6 here, so corners run at 60% of cruise.
            "cruise_erpm": 10000.0,
            "corner_erpm": 6000.0,

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
            "front_cone_half_angle_deg": 20.0,
            "min_gap_depth": 0.8,
            "min_gap_width": 0.40,
            "deep_band": 0.20,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(foxglove_node)
    ld.add_action(planning_node)
    return ld
