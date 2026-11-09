#include "algorithm.hpp"
#include <algorithm>


DriveCommand computeDrive(const std::vector<float> &ranges, const vesc_msgs::msg::VescImuStamped *imu)
{
  (void)ranges;
  (void)imu;
  DriveCommand cmd;

  cmd.steering = 0.0f;
  cmd.throttle = 0.0f;

  return cmd;
}
