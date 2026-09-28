#!/usr/bin/env python3
"""One slow, deliberate lap to build a map of the track.

This is NOT the racing launch. Everything here is traded away from lap time and
toward a map that closes cleanly:
  - the car crawls at ~1.1-1.3 m/s instead of ~3.2 m/s
  - it drives a CENTRED line rather than hugging apexes
  - slam_toolbox gets a wider scan-match window and a denser pose graph
    (config/mapper_params_mapping.yaml)

Use neo_telemetry_irl_launch.py to actually drive the track.

    HOW TO RUN

    terminal 1:  ros2 launch neo_telemetry neo_telemetry_mapping_launch.py \
                     lidar_x:=0.25 odom_source:=rf2o
    terminal 2:  ros2 run planner_node estop        # press r to release, SPACE to stop

  Nothing moves until the e-stop console releases the car - that is deliberate,
  and it is also how you stop the lap when you have been round once.

  Watch it build in Foxglove at ws://<jetson-ip>:8765. In the 3D panel set the
  fixed frame to 'map' (NOT 'odom' - in odom the whole map slides around every
  time slam_toolbox corrects the pose, which looks exactly like the map losing
  track even when it is fine) and add /map, /scan and /odom.

    WHEN THE LAP IS DONE

  Stop the car with the e-stop FIRST, leave everything else running, then save.
  Both of these want an absolute path with no extension:

    mkdir -p ~/maps
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: '/home/jetson-dhee/maps/track'}}"
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson-dhee/maps/track'}"

  save_map writes track.pgm + track.yaml, the occupancy grid you can look at and
  hand to a map server. serialize_map writes track.data + track.posegraph, which
  is the one slam_toolbox itself can reload later to localize against, so save
  BOTH - the .pgm alone cannot be continued.

    IF THE MAP STILL DOUBLES UP

  Suspect lidar_x before anything in the SLAM config. See the argument below.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Drivetrain / steering calibration. Identical to the racing launch - see the
# long comment block in neo_telemetry_irl_launch.py for where each value comes
# from. Odometry calibration has nothing to do with how fast we choose to drive,
# so these must NOT be re-tuned here; if one is wrong it is wrong in both files.
SPEED_TO_ERPM_GAIN = 4614.0
STEERING_ANGLE_TO_SERVO_GAIN = -1.2135
STEERING_ANGLE_TO_SERVO_OFFSET = 0.500
WHEELBASE = 0.3240


def generate_launch_description():

  # ---------------------------------------------------------------------------
  # The single most likely cause of a map that doubles up on itself.
  #
  # base_link is the REAR AXLE, because that is the point vesc_to_odom's bicycle
  # model integrates about. The scanner is NOT at the rear axle - on this class
  # of car it sits somewhere around a wheelbase forward of it. If this offset is
  # 0 but the scanner is really 0.25 m forward, then every time the car turns,
  # SLAM places the scan up to 0.25 * sqrt(2) = 0.35 m away from where it was
  # actually taken. On a 0.05 m grid that is SEVEN CELLS of error, appearing and
  # disappearing as the car rotates - which is precisely how you get two copies
  # of the same wall a hand's width apart, and why it looks fine on the straights
  # and falls apart in the corners.
  #
  # MEASURE IT: tape measure from the rear axle centreline to the scanner's
  # rotation centre, in metres, and pass it as lidar_x. It is worth more than
  # every SLAM parameter in this launch put together.
  #
  # No tape measure? Set it by bisection instead: run a couple of slow laps at
  # 0.0 and at 0.30 and see which corners are sharper. Or hold one steering
  # position with `ros2 run planner_node circle_test` so the car traces a steady
  # circle - a wrong lidar_x smears that circle's walls into a ring, and the
  # value that makes the ring collapse to a single line is the right one.
  # ---------------------------------------------------------------------------
  lidar_x_arg = DeclareLaunchArgument(
    "lidar_x",
    default_value="0.0",
    description="Metres from the rear axle (base_link) FORWARD to the scanner. "
                "MEASURE THIS - 0.0 is a placeholder, not a real car.",
  )
  lidar_z_arg = DeclareLaunchArgument(
    "lidar_z",
    default_value="0.18",
    description="Scanner height. Cosmetic for 2D mapping; does not affect the map.",
  )

  # ---------------------------------------------------------------------------
  # Which odometry feeds slam_toolbox. Defaults to rf2o HERE, the opposite of the
  # racing launch, and the reason is the whole point of this file.
  #
  # vesc_to_odom does not measure heading. It INFERS it, integrating
  # yaw += v * tan(steering_angle) / wheelbase, where the steering angle is the
  # one we COMMANDED, not the one the wheels took. So the yaw estimate is a
  # product of three numbers that are not calibrated on this car - the steering
  # trim offset, the wheelbase, and speed_to_erpm_gain - and a systematic error
  # in any of them shows up as a fixed FRACTION of every turn. 20 deg of error
  # per lap is a 5.6% scale error (20/360); a wheelbase off by 18 mm does that on
  # its own, and so does a steering trim off by 0.0034 in servo units.
  #
  # That error also gets WORSE as the car slows down, which sounds backwards but
  # is not: at racing speed the car understeers, so it turns less than the model
  # predicts and the understeer partly cancels an over-estimating model. Drive
  # slowly and there is no understeer left to hide behind, so the full
  # calibration error lands in the map. Slowing down to get a better map makes
  # VESC yaw worse, not better.
  #
  # rf2o inverts every one of those properties. It reads heading straight off
  # consecutive scans, so no drivetrain constant enters, and its weakness -
  # scan distortion while the lidar sweeps - is a HIGH speed problem. At 15 Hz
  # and 1.1 m/s the car moves 0.07 m per revolution instead of 0.22 m, and
  # consecutive scans overlap heavily. Slow is exactly where rf2o is strongest.
  #
  # Keep vesc for racing, where scans smear and the drivetrain is the more
  # reliable witness. Use rf2o for this: a slow, deliberate mapping lap.
  # ---------------------------------------------------------------------------
  odom_source_arg = DeclareLaunchArgument(
    "odom_source",
    default_value="rf2o",
    choices=["rf2o", "vesc"],
    description="Publisher of odom -> base_link. rf2o (default here) reads heading "
                "from the scans and needs no drivetrain calibration; vesc infers it "
                "from commanded steering and drifts if the trim, wheelbase or eRPM "
                "gain are off.",
  )

  # Slowest speed the car will reliably move at. The VESC speed PID refuses to
  # start the motor below some threshold - 2000 eRPM is measured as a no-op on
  # this car, 5000 holds cleanly at ~4.4 A - so 5000 (~1.08 m/s) is the floor,
  # not a choice. Going lower does not give a slower lap, it gives a lap that
  # stalls. 6000 eRPM is ~1.3 m/s via SPEED_TO_ERPM_GAIN.
  cruise_erpm_arg = DeclareLaunchArgument(
    "cruise_erpm",
    default_value="6000.0",
    description="Straight-line speed for the mapping lap, in electrical RPM. "
                "Do not go below 5000 - the VESC will not start the motor.",
  )

  # SICK TIM lidar driver. Same invocation as the racing launch: run
  # sick_generic_caller directly, because the wrapper .launch.py only reads
  # overrides from sys.argv and silently ignores launch_arguments.
  #
  # tf_publish_rate:=0.0 disables the driver's own map -> cloud transform, which
  # would otherwise fight our base_link -> cloud static transform and split the
  # TF tree in two.
  #
  # use_generation_timestamp:=0 stamps scans with host send-time instead of the
  # lidar's internal clock, which runs ~2 s ahead of the Jetson. Left on, every
  # scan arrives from the future and slam_toolbox's message filter throws them
  # all away ("discarding message because the queue is full").
  sick_launch_file = os.path.join(
    get_package_share_directory("sick_scan_xd"), "launch", "sick_tim_5xx.launch")
  lidar_node = Node(
    package="sick_scan_xd",
    executable="sick_generic_caller",
    output="screen",
    arguments=[
      sick_launch_file,
      "hostname:=169.254.133.43",
      "frame_id:=cloud",
      "tf_publish_rate:=0.0",
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

  foxglove_node = Node(
    package="foxglove_bridge",
    executable="foxglove_bridge",
    name="foxglove_bridge",
    output="screen",
    parameters=[{
      "port": 8765,
      "address": "0.0.0.0",
      # An occupancy grid of a whole track, re-sent every map_update_interval,
      # overruns the 10 MB default and the bridge drops the client mid-map.
      "send_buffer_limit": 100000000,
      "max_qos_depth": 10,
      "use_compression": False,
    }],
  )

  # The planner, detuned for mapping. Same node as the racing launch - the car
  # still drives itself round by following gaps - but slower and on a different
  # line.
  planning_node = Node(
    package="planner_node",
    executable="planner_gap_follow",
    output="screen",
    parameters=[{
      # Crawl. corner and brake both sit at 5000 because that is the one speed
      # measured good on this car, and a mapping lap has no reason to explore
      # the 2000-5000 region where the motor may simply not start.
      "cruise_erpm": ParameterValue(LaunchConfiguration("cruise_erpm"),
                                    value_type=float),
      "corner_erpm": 5000.0,
      "brake_erpm": 5000.0,
      "brake_current": 4.0,

      # Drive a CENTRED line, not the racing line. deep_band is how far back
      # from a gap's deepest reading still counts as "the deep end" the car aims
      # at; 0.20 (the racing default) aims at the deep sliver by the inside apex,
      # which is fast and correct for a lap time and wrong for mapping.
      #
      # Measured offline by raycasting a wide corner and asking how close the
      # aimed heading passes to a wall: at 0.20 it passes 1.4 m from the apex, at
      # 0.50 it passes 5.3 m away - while still turning +31.7 deg into the corner
      # versus +34.3 deg. So this buys a centred car for almost no turn-in.
      #
      # That matters for the map, not just for safety. Centred, both walls are
      # at moderate range and hit the scanner near perpendicular; hugging an apex
      # puts one wall 0.2 m away at grazing incidence, where it returns few
      # points and scan matching is badly conditioned.
      "deep_band": 0.50,

      # Wider than the racing 0.18, so the car eases off earlier into anything
      # narrowing. Deliberately conservative: a mapping lap that brushes a wall
      # is a mapping lap you do again.
      "corridor_half_width": 0.25,
      # Drop to corner speed at a smaller steering request than the 0.30 racing
      # default, so the car is already slow BEFORE it starts rotating - yaw is
      # where the odometry is weakest.
      "corner_lock_fraction": 0.20,
    }],
  )

  vesc_config_file = os.path.join(
    get_package_share_directory("vesc_driver"), "params", "vesc_config.yaml")
  vesc_driver_node = Node(
    package="vesc_driver",
    executable="vesc_driver_node",
    name="vesc_driver_node",
    output="screen",
    parameters=[vesc_config_file],
  )

  # base_link -> cloud. The only publisher of it; the lidar driver's own TF is
  # off. args = x y z yaw pitch roll parent child
  base_link_to_lidar_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="base_link_to_cloud",
    arguments=[LaunchConfiguration("lidar_x"), "0", LaunchConfiguration("lidar_z"),
               "0", "0", "0", "base_link", "cloud"],
  )

  # odom -> base_link from wheel eRPM + the COMMANDED steering angle. Only one of
  # this and rf2o below ever runs: two publishers of the same transform would
  # fight, and the TF tree would flicker between them.
  vesc_to_odom_node = Node(
    package="vesc_ackermann",
    executable="vesc_to_odom_node",
    name="vesc_to_odom_node",
    output="screen",
    condition=LaunchConfigurationEquals("odom_source", "vesc"),
    parameters=[{
      "odom_frame": "odom",
      "base_frame": "base_link",
      "publish_tf": True,
      "speed_to_erpm_gain": SPEED_TO_ERPM_GAIN,
      "speed_to_erpm_offset": 0.0,
      "use_servo_cmd_to_calc_angular_velocity": True,
      "steering_angle_to_servo_gain": STEERING_ANGLE_TO_SERVO_GAIN,
      "steering_angle_to_servo_offset": STEERING_ANGLE_TO_SERVO_OFFSET,
      "wheelbase": WHEELBASE,
    }],
  )

  # Scan-to-scan laser odometry: the default source for this launch. freq must
  # track the real scan rate (the SICK TIM runs at 15 Hz) or rf2o integrates over
  # the wrong interval and its own scale comes out wrong.
  rf2o_odometry_node = Node(
    package="rf2o_laser_odometry",
    executable="rf2o_laser_odometry_node",
    name="rf2o_laser_odometry",
    output="screen",
    condition=LaunchConfigurationEquals("odom_source", "rf2o"),
    parameters=[{
      "laser_scan_topic": "/scan",
      "odom_topic": "/odom",
      "publish_tf": True,
      "base_frame_id": "base_link",
      "odom_frame_id": "odom",
      "init_pose_from_topic": "",
      "freq": 15.0,
    }],
  )

  slam_toolbox_node = Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    name="slam_toolbox",
    output="screen",
    parameters=[
      os.path.join(get_package_share_directory("neo_telemetry"),
                   "config", "mapper_params_mapping.yaml"),
      {"use_sim_time": False},
    ],
  )

  ld = LaunchDescription()
  ld.add_action(odom_source_arg)
  ld.add_action(lidar_x_arg)
  ld.add_action(lidar_z_arg)
  ld.add_action(cruise_erpm_arg)
  ld.add_action(lidar_node)
  ld.add_action(foxglove_node)
  ld.add_action(planning_node)
  ld.add_action(vesc_driver_node)
  ld.add_action(base_link_to_lidar_tf)
  ld.add_action(vesc_to_odom_node)
  ld.add_action(rf2o_odometry_node)
  ld.add_action(slam_toolbox_node)
  return ld
