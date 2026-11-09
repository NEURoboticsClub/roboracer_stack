#include "algorithm.hpp"


DriveCommand computeDrive(const std::vector<float> &ranges)
{
  (void)ranges;

  DriveCommand cmd;
  cmd.steering = 0.0f;
  cmd.throttle = 0.0f;
  return cmd;
}
