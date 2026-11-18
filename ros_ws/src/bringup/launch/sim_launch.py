#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# AutoDRIVE simulator bring-up: the planner and Foxglove, plus Cartographer on
# request. The algorithm itself is planner's src/algorithm.cpp.
#
# MAPPING. Off by default; turn it on with `mapping:=true`. Cartographer 2D,
# driven by lidar scan matching plus the sim IMU's gyro and accelerometer - no
# odometry, and never the bridge's ground-truth pose. `mapping_imu:=false` drops
# the IMU and goes back to scan matching alone (config/cartographer_sim.lua for
# that and the frames, config/cartographer_sim_imu.lua for the IMU version).
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
#
# SAVING THE MAP (Cartographer; mapping keeps running). Only ros_ws/src is
# mounted from the host, so save under it. One line each, paste as-is:
#
#   mkdir -p /home/ros_ws/src/bringup/maps
#   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: /home/ros_ws/src/bringup/maps/sim_track.pbstream, include_unfinished_submaps: true}"
#   ros2 run nav2_map_server map_saver_cli -f /home/ros_ws/src/bringup/maps/sim_track --ros-args -p map_subscribe_transient_local:=false
#
# write_state saves Cartographer's own state (.pbstream), which it can reload
# to keep mapping or to localize. map_saver_cli saves /map as .pgm + .yaml for
# looking at or handing to a map server. map_subscribe_transient_local:=false
# makes the saver subscribe to /map with plain volatile QoS; its default
# transient-local subscription can fail to match a volatile /map publisher and
# then just waits. Saved this way it worked first time.
# ---------------------------------------------------------------------------

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# The bridge's lidar topic. The planner subscribes to /scan (the name the SICK
# publishes on the real car), so exactly one remap is what makes the same node
# binary work in both places.
SIM_SCAN_TOPIC = "/autodrive/roboracer_1/lidar"
SIM_IMU_TOPIC = "/autodrive/roboracer_1/imu"
SIM_IMU_GRAVITY_TOPIC = "/autodrive/roboracer_1/imu_with_gravity"
SIM_SCAN_INSTANT_TOPIC = "/autodrive/roboracer_1/lidar_instant"


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
            # /map is re-sent at 10 Hz when mapping; the 10 MB default can
            # overrun and the bridge drops the client mid-map.
            "send_buffer_limit": 100000000,
            "max_qos_depth": 10,
            "use_compression": False,
        }],
    )

    planning_node = Node(
        package="planner",
        executable="planner",
        name="planner",
        output="screen",
        # The one rename that makes the algorithm work here unchanged.
        #
        # The IMU is deliberately NOT remapped. The planner's /sensors/imu is
        # vesc_msgs/VescImuStamped and the sim's IMU is sensor_msgs/Imu; ROS 2
        # never connects mismatched types, so the remap delivered nothing and
        # only left the sim topic with two types, which breaks
        # `ros2 topic echo` on it. In the sim the algorithm's imu is null until
        # the planner learns to convert sensor_msgs/Imu.
        remappings=[("/scan", SIM_SCAN_TOPIC)],
        parameters=[{
            # Drive the AutoDRIVE command topics instead of the VESC ones. This
            # was a compile-time #define, so a sim run needed a source edit and
            # a rebuild; miss either and the planner came up publishing
            # /commands/motor/speed, which nothing in the sim subscribes to, and
            # the car just sat there.
            #
            # output_mode:=sim ALSO means no e-stop: the kill-switch is a real-car
            # thing and there is none in the sim stack, so the planner drives as
            # soon as scans start arriving.
            "output_mode": "sim",

            # AutoDRIVE's throttle is normalized [-1, 1] and open-loop. This is
            # what the algorithm's throttle +1.0 becomes: 0.10 runs ~2.2 m/s and
            # throttle-to-speed is not linear at the low end.
            "sim_max_throttle": 0.2,
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

    # ---- mapping: Cartographer, scan matching (+ IMU) ------------------------
    mapping = IfCondition(LaunchConfiguration("mapping"))

    # cartographer_sim_imu.lua is cartographer_sim.lua with the IMU switched on
    # and the tracking frame moved to it; nothing else differs, so flipping
    # mapping_imu is a clean with/without comparison.
    cartographer_config = PythonExpression([
        "'cartographer_sim_imu.lua' if '", LaunchConfiguration("mapping_imu"),
        "'.lower() == 'true' else 'cartographer_sim.lua'",
    ])

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        condition=mapping,
        arguments=[
            "-configuration_directory",
            os.path.join(get_package_share_directory("bringup"), "config"),
            "-configuration_basename", cartographer_config,
        ],
        # Cartographer subscribes to 'imu' only when the config has
        # use_imu_data = true, so this remap is harmless with mapping_imu:=false.
        # It takes the RELAYED scan and IMU, not the bridge's: see the two relay
        # nodes below.
        remappings=[("scan", SIM_SCAN_INSTANT_TOPIC), ("imu", SIM_IMU_GRAVITY_TOPIC)],
        parameters=[{"use_sim_time": False}],
    )

    # The bridge labels each scan a 25 ms sweep but sends one every ~17 ms, so
    # Cartographer sees consecutive scans overlap in time: it drops points, and
    # with the IMU on it aborts on the first out-of-order IMU sample. The sim
    # takes every beam in one tick, so this zeroes time_increment to match. See
    # scripts/scan_instant_relay.py.
    scan_instant_relay_node = Node(
        package="bringup",
        executable="scan_instant_relay.py",
        name="scan_instant_relay",
        output="screen",
        condition=mapping,
        remappings=[("scan_in", SIM_SCAN_TOPIC), ("scan_out", SIM_SCAN_INSTANT_TOPIC)],
    )

    # The sim's accelerometer leaves gravity out (measured: ~0 on every axis at
    # rest), and Cartographer needs it to know which way is up. This adds it
    # back, as a real accelerometer would read it. See
    # scripts/imu_gravity_relay.py.
    imu_gravity_relay_node = Node(
        package="bringup",
        executable="imu_gravity_relay.py",
        name="imu_gravity_relay",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mapping"), "'.lower() == 'true' and '",
            LaunchConfiguration("mapping_imu"), "'.lower() == 'true'",
        ])),
        remappings=[("imu_in", SIM_IMU_TOPIC), ("imu_out", SIM_IMU_GRAVITY_TOPIC)],
    )

    # Cartographer keeps its map as submaps; this renders them onto /map.
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        condition=mapping,
        # 0.10 m cells to match TRAJECTORY_BUILDER_2D's submap resolution in
        # cartographer_sim.lua. Republished at 10 Hz (was 2 Hz, which is what
        # made the map lag in Foxglove).
        arguments=["-resolution", "0.10", "-publish_period_sec", "0.1"],
        parameters=[{"use_sim_time": False}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        "mapping",
        default_value="false",
        description="Run Cartographer on the sim lidar (and IMU, see mapping_imu).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "mapping_imu",
        default_value="true",
        description="Feed the sim IMU's gyro and accelerometer to Cartographer. "
                    "false = lidar scan matching only.",
    ))
    ld.add_action(foxglove_node)
    ld.add_action(planning_node)
    ld.add_action(scan_instant_relay_node)
    ld.add_action(imu_gravity_relay_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    return ld
