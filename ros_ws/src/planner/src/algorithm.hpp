#pragma once

#include <vector>
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"

// What your algorithm hands back each scan. Both values are normalized, so the
// same command means the same thing in the sim and on the real car.
struct DriveCommand
{
  // +1.0  full lock LEFT
  //  0.0  straight
  // -1.0  full lock RIGHT
  float steering = 0.0f;

  // +1.0  max throttle (irl_max_throttle_erpm / sim_max_throttle in the launch file)
  //  0.0  coast
  // -1.0  max brake (irl_max_brake_amps / sim_max_brake in the launch file)
  //
  // Values in between scale linearly. On the real car braking stops the car, but
  // the sim has no brake, so it is reverse throttle and holding it backs the car up.
  float throttle = 0.0f;
};

// No-return or anything beyond this is treated as open space at this range.
constexpr float kMaxRange = 10.0f;

// Called once per lidar scan; whatever it returns goes straight to the car.
// Implemented in algorithm.cpp.
DriveCommand computeDrive(const std::vector<float> &ranges, const vesc_msgs::msg::VescImuStamped *imu);

// The last scan's internals, for visualization only.
struct DebugInfo
{
  struct Edge
  {
    int index; // jump between beams index and index + 1
    float a, b;
  };

  std::vector<float> ranges; // cleaned scan with disparity bubbles applied
  std::vector<char> blocked; // beam lies inside a bubble
  std::vector<Edge> edges;   // disparities that created bubbles
  float heading = 0.0f;      // rad, chosen heading
};
const DebugInfo &debugInfo();

// Angle (radians) of beam `index`: 0 = straight ahead, positive = LEFT.
float ind2angle(int index);

// Header stamp of the current scan in seconds, or 0 if the publisher left it unset.
double scanTime();

// True when the node runs with output_mode:=sim.
bool simMode();

// Forward wheel speed in m/s from the VESC (/sensors/core). -1 = unknown (sim,
// or it's gone quiet). Wheelspin reads high, locked wheels read 0.
float wheelSpeed();

// Normalized throttle that asks the real car for `mps` m/s, using
// irl_max_throttle_erpm and speed_to_erpm_gain from the launch file.
float speedToThrottle(float mps);
