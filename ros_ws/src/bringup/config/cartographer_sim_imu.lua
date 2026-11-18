-- Cartographer 2D for the AutoDRIVE sim: LIDAR SCAN MATCHING + IMU.
--
-- Everything in cartographer_sim.lua, plus the sim IMU, via
-- /autodrive/roboracer_1/imu_with_gravity (remapped onto 'imu' in
-- sim_launch.py). Still no odometry and no use of the bridge's ground-truth
-- pose.
--
-- What the IMU adds. In 2D Cartographer reads only two fields of it:
--   angular_velocity     integrated into the motion prediction between scans,
--                        so the scan matcher starts from the right heading in
--                        corners instead of extrapolating the last few scans;
--   linear_acceleration  to keep track of which way gravity points.
-- It IGNORES the message's orientation field. That matters in this sim, where
-- the bridge fills orientation with the simulator's exact pose: none of that
-- ground truth reaches the estimate.
--
-- The IMU's linear_acceleration must INCLUDE GRAVITY (about +9.81 on z with the
-- car at rest - the ROS convention for a real accelerometer). Cartographer
-- estimates "up" from it and aborts (a CHECK failure in ImuTracker) if it reads
-- ~0. The sim's raw IMU DOES read ~0 at rest (measured), which is why the
-- launch file runs scripts/imu_gravity_relay.py to add gravity back in.

local options = include "cartographer_sim.lua"

-- Cartographer requires the IMU to sit AT the tracking frame: it refuses to
-- move linear acceleration between frames, because on a turning car that adds
-- centripetal acceleration the IMU never felt. The bridge puts 'imu' 0.08 m
-- ahead of the rear axle and 'lidar' 0.2733 m ahead, so tracking moves from the
-- lidar to the IMU. The scan is transformed into it through the bridge's TF;
-- the published frame (map -> world) does not change.
options.tracking_frame = "imu"

TRAJECTORY_BUILDER_2D.use_imu_data = true

return options
