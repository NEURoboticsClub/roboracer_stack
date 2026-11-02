#pragma once

#include <vector>

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

// Called once per lidar scan; whatever it returns goes straight to the car.
// Implemented in algorithm.cpp.
DriveCommand computeDrive(const std::vector<float> &ranges);

// Angle (radians) of beam `index`: 0 = straight ahead, positive = LEFT.
float ind2angle(int index);
