#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Drivetrain / steering calibration.
#
# These four numbers are the whole job of vesc_to_odom: it reads sensors/core
# (electrical RPM, as the VESC reports it) and sensors/servo_position_command
# (the driver's echo of the 0..1 servo command we sent) and dead-reckons a
# bicycle model from them. Get one wrong and the odometry is wrong in a way
# slam_toolbox cannot fully paper over - the map comes out stretched, sheared,
# or mirrored.
#
# speed_to_erpm_gain       eRPM per m/s. Sanity check against the planner's own
#                          speeds below: 15000 cruise -> 3.25 m/s, 7000 corner
#                          -> 1.52 m/s, 5000 brake -> 1.08 m/s. Those are
#                          believable for this car, but this is the ONE value
#                          here NOT measured on it - see CALIBRATING SCALE at
#                          the bottom of this file.
#
# steering_angle_to_servo_gain
#                          Servo units per radian of road-wheel angle, and it is
#                          NEGATIVE on this car: servo 0.117 is full LEFT (that
#                          is what circle_test commands by default) while
#                          REP-103 counts left as a POSITIVE angle. Get this
#                          sign wrong and every turn integrates the wrong way,
#                          so the map comes out mirrored.
#
# steering_angle_to_servo_offset
#                          Servo value at true mechanical center. 0.500 is
#                          measured on this car (see the comment in
#                          vesc_config.yaml); the stock f1tenth 0.5304 is some
#                          other car's trim. With this car's 0.117/0.895 servo
#                          limits, 0.500 and the gain below put full lock at
#                          18.1-18.6 deg - which is exactly the 18.6 deg the
#                          planner has measured on the car.
#
# wheelbase                Front-to-rear axle spacing, meters. Derived from the
#                          planner's two measured numbers rather than guessed:
#                          R = L / tan(delta), so L = 0.965 * tan(18.6 deg)
#                          = 0.325 m. (The vendored vesc_ackermann launch file
#                          ships 0.2, which is nobody's car - do not copy it.)
# ---------------------------------------------------------------------------
SPEED_TO_ERPM_GAIN = 4614.0
STEERING_ANGLE_TO_SERVO_GAIN = -1.2135
STEERING_ANGLE_TO_SERVO_OFFSET = 0.500
WHEELBASE = 0.3240


def generate_launch_description():

  # -------------------------------------------------------------------------
  # MAPPING IS DISABLED IN THIS LAUNCH FILE (race configuration).
  #
  # planner_gap_follow is purely reactive - it subscribes to /scan and /estop and
  # nothing else, with no odometry, no TF lookups and no nav_msgs dependency. So
  # SLAM and the odometry that feeds it were pure overhead during a race, and the
  # Jetson's CPU is better spent on the lidar and the control loop.
  #
  # Commented out below, all marked "DISABLED FOR THE RACE":
  #   slam_toolbox_node    the mapping itself, and by far the biggest CPU win
  #   vesc_to_odom_node    odom -> base_link dead reckoning, only SLAM used it
  #   rf2o_odometry_node   the alternative odometry source (was already off)
  #   slam_params_file     config path for slam_toolbox
  #   odom_source_arg      launch arg that only chose between the two odometries
  #   use_slam_arg         launch arg that only gated slam_toolbox
  #
  # STILL RUNNING: lidar, planner, killswitch, vesc_driver, the base_link -> cloud
  # static transform (a one-shot latched publish, no measurable cost, and Foxglove
  # needs it to place /scan), and foxglove_bridge.
  #
  # TO MAP AGAIN, use the launch file that is for it, rather than reviving these:
  #     ros2 launch neo_telemetry neo_telemetry_mapping_launch.py
  #
  # Note the SPEED_TO_ERPM_GAIN / STEERING_* / WHEELBASE constants at the top are
  # now unused here - they only ever fed vesc_to_odom. Left in place so
  # uncommenting is a clean revert.
  # -------------------------------------------------------------------------


  # Which node owns the odom -> base_link transform. EXACTLY ONE may run:
  # two publishers of the same transform make the TF tree jump between them
  # and slam_toolbox's pose graph goes to pieces.
  #
  #   vesc  - dead reckoning from wheel eRPM + the commanded steering angle.
  #           Metrically honest (it knows how far the wheels actually turned)
  #           but has no gyro, so heading drifts and wheelspin reads as
  #           distance. This is the default and what slam_toolbox wants: a
  #           smooth, causal motion prior it can then correct.
  #   rf2o  - scan-to-scan laser odometry. No drivetrain calibration needed, but
  #           it is derived from the same scans slam_toolbox matches, so its
  #           errors are correlated with the scan matcher's and it degenerates
  #           in feature-poor corridors.
  # --- DISABLED FOR THE RACE (only selected mapping odometry) ---
#   odom_source_arg = DeclareLaunchArgument(
#     "odom_source",
#     default_value="vesc",
#     choices=["vesc", "rf2o"],
#     description="Which node publishes odom -> base_link.",
#   )

  # --- DISABLED FOR THE RACE (only gated slam_toolbox) ---
#   use_slam_arg = DeclareLaunchArgument(
#     "use_slam",
#     default_value="true",
#     choices=["true", "false"],
#     description="Run slam_toolbox. Set false to just drive without mapping.",
#   )

  # SICK TIM lidar driver.
  # We run sick_generic_caller directly (instead of including the wrapper
  # .launch.py) because that wrapper only reads param overrides from sys.argv,
  # so IncludeLaunchDescription's launch_arguments are silently ignored.
  # sick_generic_caller takes the .launch XML plus `name:=value` overrides.
  #
  # tf_publish_rate:=0.0 DISABLES the driver's own TF. By default the driver
  # publishes map -> cloud (tf_base_frame_id=map, frame_id=cloud), which fights
  # our base_link -> cloud static transform and splits the TF tree. We want the
  # clean chain  map -> odom -> base_link -> cloud  instead.
  sick_launch_file = os.path.join(
    get_package_share_directory("sick_scan_xd"),
    "launch",
    "sick_tim_5xx.launch",
  )
  lidar_node = Node(
    package="sick_scan_xd",
    executable="sick_generic_caller",
    output="screen",
    arguments=[
      sick_launch_file,
      "hostname:=169.254.133.43",
      "frame_id:=cloud",      # scans are stamped in the 'cloud' frame
      "tf_publish_rate:=0.0", # do not let the driver publish map -> cloud
      # Stamp scans with the host send-time, NOT the lidar's internal clock.
      # The device clock runs ~2 s ahead of the Jetson, which puts scans in the
      # "future" and makes slam_toolbox's tf2 message filter drop every scan
      # after the first ("discarding message because the queue is full").
      "use_generation_timestamp:=0",
      # Turn the driver's range filter ON. It ships DEACTIVATED
      # (range_filter_handling=0), which is why the scan arrives with
      # range_min=0.0 and a handful of 0.0 ranges in it.
      #
      # That matters far more than it looks. The SICK encodes a no-return as 0.0,
      # and with range_min=0.0 nothing downstream filters it: those beams become
      # valid points AT THE SENSOR ORIGIN. planner_gap_follow already works
      # around this in its own preprocessing, but rf2o and slam_toolbox consume
      # the raw scan and do not:
      #   - rf2o solves a dense range-flow least squares weighted BY RANGE
      #     GRADIENT, and a 0.0 sitting next to a 5 m return is an enormous
      #     spurious gradient, so a dozen junk beams can dominate the fit.
      #   - a point at the origin is rotation-invariant, so it anchors
      #     slam_toolbox's scan matcher against rotating - which is exactly the
      #     3.6% rotational correction slam_diag.py measured.
      # It also explains why the drift looked identical under rf2o and under
      # vesc_to_odom: whatever produced the odometry, the matcher was equally
      # handicapped at correcting it.
      #
      # handling 5 = RANGE_FILTER_TO_NAN, the ROS convention for "no return".
      # Both consumers discard NaN, and the planner already maps non-finite to
      # open space, so its behaviour does not change.
      "range_min:=0.05",
      "range_max:=25.0",
      "range_filter_handling:=5",
    ],
  )

  # Foxglove: connect a browser or Foxglove Studio to  ws://<jetson-ip>:8765
  #
  # What to add in the 3D panel to see the map being built:
  #   - frame:  map        (so the car moves and the map stays put; pick 'odom'
  #                         instead to watch raw odometry drift away from it)
  #   - topics: /map       occupancy grid from slam_toolbox
  #             /scan      live lidar
  #             /odom      the VESC dead-reckoned pose
  #             /visualization_markers   the planner's chosen gap + heading
  foxglove_node = Node(
    package="foxglove_bridge",
    executable="foxglove_bridge",
    name="foxglove_bridge",
    output="screen",
    parameters=[{
      "port": 8765,
      "address": "0.0.0.0",   # listen on the LAN, not just localhost
      # A 0.05 m occupancy grid of a whole track is a big message and it is
      # republished every map_update_interval. The 10 MB default send buffer
      # overflows and the bridge drops the client mid-map; 100 MB rides it out.
      "send_buffer_limit": 100000000,
      # /map is latched (transient_local). Keeping the bridge's depth at 1 for
      # it is fine, but the scan/marker streams want a little slack.
      "max_qos_depth": 10,
      "use_compression": False,  # the Jetson's CPU is better spent on SLAM
    }],
  )

  # The planner publishes the VESC command topics directly:
  #   /commands/motor/speed   (electrical RPM)
  #   /commands/motor/brake   (amps)
  #   /commands/servo/position (0..1 -> PPM pulse on the VESC servo header)
  #
  # The planner picks its own speed from the lidar (cruise / corner / brake). It
  # is STOPPED unless killswitch_node (below) says otherwise, so the operator's
  # handheld remote has to be armed before anything moves.
  #
  # The old keyboard console (ros2 run planner_node estop) still exists for bench
  # work, but it publishes the SAME /estop topic - run one or the other, NEVER
  # both, or the planner sees the two of them alternating at 100 Hz.
  #
  # NOTE for mapping: /commands/servo/position is also what vesc_to_odom needs
  # to know the steering angle (the driver echoes it back on
  # sensors/servo_position_command). vesc_to_odom publishes NOTHING until it has
  # seen one, so with odom_source:=vesc there is no odom -> base_link transform
  # until this node is up and receiving scans. That is fine in normal use - the
  # planner publishes a servo command every scan, holding center while the
  # e-stop is engaged - but it is why "slam_toolbox sees no TF" usually means
  # "the planner or the VESC is down", not a SLAM problem.
  planning_node = Node(
    package='planner_node',
    executable='planner_gap_follow',
    output='screen',
    parameters=[{
      # Electrical RPM = mechanical RPM x motor pole pairs. The VESC speed PID
      # has a minimum ERPM below which it will not START the motor: 2000
      # measured as a no-op on this car, 5000 holds cleanly at ~4.4 A. So 5000
      # anchors the set - corner sits on it, straight-line reaches past it, and
      # braking drops below it while staying well clear of the dead zone.
      'cruise_erpm': 15000.0,   # straight-line max
      'corner_erpm': 7000.0,   # tight clearance ahead 
      'brake_erpm': 5000.0,    # closing on a wall: wind down, don't lock up
      'brake_current': 4.0,    # hard stop only (e-stop / Ctrl+C)

      # Half-width (m) of the lane the car needs clear ahead to stay at
      # cruise_erpm. Only obstacles within this band of the centerline count as
      # blocking; walls further out are passing the car down the side, which is
      # what keeps narrow-but-straight sections at straight-line speed.
      #
      # The car is 0.30 m wide, so 0.15 is a hard floor - going under it means
      # the check ignores things the car will physically hit. Raise it toward
      # 0.25-0.30 if the car carries too much speed into a narrowing section.
      'corridor_half_width': 0.18,
    }],
  )

  # Remote kill-switch receiver - the car's end of the operator's handheld remote.
  #
  #   gamepad --BT--> laptop --UDP :5005--> here --> /estop + /commands/motor/brake
  #
  # On the laptop:  python tools/killswitch_remote.py --host <this jetson's ip>
  #
  # STOPPED BY DEFAULT. The car drives only while valid ARMED packets keep
  # arriving; WiFi dropping, the laptop sleeping, the script dying and the
  # gamepad's battery going flat all look the same from here (packets stop) and
  # all brake the car within timeout_ms. That is the fallback the whole design
  # exists for - test it by turning the laptop's WiFi off with the car on blocks.
  killswitch_node = Node(
    package='planner_node',
    executable='killswitch',
    name='killswitch_node',
    output='screen',
    parameters=[{
      'port': 5005,
      # 400 ms rides out an ordinary WiFi hiccup without ending a run, and caps
      # the worst case at ~1.3 m of travel at cruise. Tune from the measured
      # stop time in the WiFi-drop test.
      'timeout_ms': 400,
      # Must match --token on the laptop. Its job is narrow: at a competition
      # every team runs a hotspot in the same hall, and a stray packet that could
      # ARM the car is worth four bytes of defence. A mismatch is fail-safe - the
      # car simply refuses to arm.
      'token': 0x5252534B,
      'brake_current': 4.0,
    }],
  )

  # VESC driver: owns the USB link to the VESC (/dev/ttyACM0 - the VESC is a
  # CDC-ACM device, so it never shows up as /dev/ttyUSB*). Turns the command
  # topics above into motor commands and a servo PPM pulse.
  # NOTE: the servo output must be enabled in VESC Tool
  # (App Settings -> General -> Enable Servo Output) or steering stays dead.
  #
  # It also publishes what vesc_to_odom reads back:
  #   sensors/core                    motor state, incl. signed electrical RPM
  #   sensors/servo_position_command  echo of the clipped servo command
  vesc_config_file = os.path.join(
    get_package_share_directory('vesc_driver'),
    'params',
    'vesc_config.yaml',
  )
  vesc_driver_node = Node(
    package='vesc_driver',
    executable='vesc_driver_node',
    name='vesc_driver_node',
    output='screen',
    parameters=[vesc_config_file],
  )

  # Static transform: where the SICK TIM sits on the car, relative to base_link.
  # This is the ONLY publisher of base_link -> cloud (the driver's own TF is
  # disabled above via tf_publish_rate:=0.0).
  # args = x y z yaw pitch roll parent child   (meters / radians)
  #
  # MEASURE THIS. The x offset matters more now than it did under rf2o, and it
  # is currently 0, which is almost certainly wrong. vesc_to_odom integrates a
  # bicycle model about the REAR AXLE, so base_link IS the rear axle - and on
  # this class of car the scanner sits roughly a wheelbase forward of it. Every
  # time the car yaws, a lidar that is really 0.25 m forward sweeps an arc that
  # a 0-offset transform says it did not, and slam_toolbox spends the whole run
  # fighting its own motion prior: smeared walls, and loop closures that snap.
  # z (scanner height) is cosmetic for 2D mapping; yaw matters if the scanner is
  # rotated on its mount.
  base_link_to_lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_cloud',
    arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'cloud']
  )

  # ---- odom -> base_link, option A (default): VESC dead reckoning ----------
  # Reads sensors/core + sensors/servo_position_command, publishes /odom and the
  # odom -> base_link transform. See the calibration block at the top of this
  # file for where each number comes from.
  # --- DISABLED FOR THE RACE (odometry is only needed for SLAM) ---
#   vesc_to_odom_node = Node(
#     package='vesc_ackermann',
#     executable='vesc_to_odom_node',
#     name='vesc_to_odom_node',
#     output='screen',
#     condition=LaunchConfigurationEquals('odom_source', 'vesc'),
#     parameters=[{
#       'odom_frame': 'odom',
#       'base_frame': 'base_link',
#       'publish_tf': True,
#       'speed_to_erpm_gain': SPEED_TO_ERPM_GAIN,
#       'speed_to_erpm_offset': 0.0,
#       # Without this the odometry has no heading at all - it integrates every
#       # motion straight along +x and the "map" is a corridor. On means yaw comes
#       # from the bicycle model, v * tan(steer) / wheelbase.
#       'use_servo_cmd_to_calc_angular_velocity': True,
#       'steering_angle_to_servo_gain': STEERING_ANGLE_TO_SERVO_GAIN,
#       'steering_angle_to_servo_offset': STEERING_ANGLE_TO_SERVO_OFFSET,
#       'wheelbase': WHEELBASE,
#     }],
#   )

  # ---- odom -> base_link, option B: laser odometry -------------------------
  # The previous default. Kept so it is one launch argument to go back and
  # compare (odom_source:=rf2o) if a map comes out badly and you want to know
  # whether to blame the drivetrain calibration.
  # --- DISABLED FOR THE RACE (alternative mapping odometry) ---
#   rf2o_odometry_node = Node(
#     package='rf2o_laser_odometry',
#     executable='rf2o_laser_odometry_node',
#     name='rf2o_laser_odometry',
#     output='screen',
#     condition=LaunchConfigurationEquals('odom_source', 'rf2o'),
#     parameters=[{
#       'laser_scan_topic': '/scan',
#       'odom_topic': '/odom',
#       'publish_tf': True,
#       'base_frame_id': 'base_link',
#       'odom_frame_id': 'odom',
#       'init_pose_from_topic': '',
#       'freq': 15.0,  # match the SICK TIM scan rate
#     }]
#   )

  # --- DISABLED FOR THE RACE (SLAM config) ---
#   slam_params_file = os.path.join(
#     get_package_share_directory('neo_telemetry'),
#     'config',
#     'mapper_params_irl.yaml',
#   )

  # --- DISABLED FOR THE RACE (the mapping itself - biggest CPU win) ---
#   slam_toolbox_node = Node(
#     package='slam_toolbox',
#     executable='async_slam_toolbox_node',
#     name='slam_toolbox',
#     output='screen',
#     condition=LaunchConfigurationEquals('use_slam', 'true'),
#     parameters=[slam_params_file, {'use_sim_time': False}],
#   )

  # Define LaunchDescription variable
  ld = LaunchDescription()

#   ld.add_action(odom_source_arg)
#   ld.add_action(use_slam_arg)
  ld.add_action(lidar_node)
  ld.add_action(foxglove_node)
  ld.add_action(planning_node)
  ld.add_action(killswitch_node)
  ld.add_action(vesc_driver_node)
  ld.add_action(base_link_to_lidar_tf)
#   ld.add_action(vesc_to_odom_node)
#   ld.add_action(rf2o_odometry_node)
#   ld.add_action(slam_toolbox_node)
  # ld.add_action(camera_node)

  return ld


# ---------------------------------------------------------------------------
# CALIBRATING SCALE (speed_to_erpm_gain)
#
# 4614.0 is the stock f1tenth value for this drivetrain, not a measurement off
# this car, and it is the only number above that is not. It sets the SCALE of
# the map: too low and the map comes out longer than the track, too high and it
# comes out shorter. Everything else can look perfect while this is wrong.
#
# To check it, on blocks with the wheels free to spin is not enough - you need
# ground contact. Mark a straight line on the floor, then:
#
#     ros2 topic echo --field pose.pose.position.x /odom
#
# Push or drive the car a tape-measured distance (5 m is plenty) in a straight
# line and compare the change in x against the tape:
#
#     speed_to_erpm_gain_new = 4614.0 * (odom_delta_x / tape_measured_metres)
#
# Repeat it in the other direction; a big asymmetry means the servo offset above
# is off center, not that the scale is wrong.
# ---------------------------------------------------------------------------
