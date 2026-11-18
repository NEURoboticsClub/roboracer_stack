-- Cartographer 2D for the AutoDRIVE sim: LIDAR SCAN MATCHING ONLY.
--
-- No odometry, no IMU, and no use of the bridge's ground-truth pose. Every
-- pose comes from matching the current scan against the map built so far.
--
-- Frames. The bridge's tree is world -> roboracer_1 -> lidar, and the scan is
-- stamped 'lidar'. Cartographer tracks the lidar itself (tracking_frame) and
-- publishes map -> world (published_frame), solved so that map -> lidar is
-- exactly its OWN estimate. world -> lidar is only looked up to place that
-- published transform; it never enters the estimate. So in Foxglove's 'map'
-- frame the car and scan are drawn where Cartographer thinks they are.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "lidar",
  published_frame = "world",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  -- How often the submaps behind /map are re-sent. 0.3 made Foxglove lag.
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D without an IMU must say so, or Cartographer waits for one forever.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- One scan per insertion: the sim lidar is a full 270 deg sweep each message.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- The sim lidar's limits (range_min 0.06 m, range_max 10 m).
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0
-- With no odometry the only motion prior is the extrapolation of past scan
-- matches, so let the fast correlative matcher search around it before the
-- fine Ceres match. Windows sized for ~2 m/s at ~17 Hz (~0.12 m and a few
-- degrees per scan, more in corners).
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Grid cell size, 0.05 m (default) -> 0.10 m. Four times fewer cells to match
-- against and render, so matching and /map updates run faster. The cost is
-- detail: a wall is located to the nearest 10 cm instead of 5 cm, which on a
-- tight track is visible. Keep the occupancy grid node's -resolution in
-- sim_launch.py the same.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.10

-- The motion filter. Every scan is still matched, but only scans that pass
-- this filter are INSERTED into the map. Defaults are 0.2 m / 1 deg / 5 s,
-- which at ~2 m/s inserts only ~10 scans a second and makes the map fill in
-- slowly. Insert after 0.1 m, 0.5 deg, or 1 s, whichever comes first.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0

-- Global matching: how many inserted scans between pose-graph optimizations
-- (the step that searches for loop closures and re-aligns the submaps).
-- Default 90. With the denser insertion above, 90 nodes would be ~9 m of
-- track between optimizations; 30 is ~3 m.
POSE_GRAPH.optimize_every_n_nodes = 30

return options
