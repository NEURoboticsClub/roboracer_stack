#include "algorithm.hpp"
#include <algorithm>
#include <cmath>

namespace
{
  constexpr float kMaxRange = 10.0f;      // treat no-return as this far
  constexpr float kDisparity = 0.2f;      // m jump between beams = an edge
  constexpr float kCoverMultiplier = 0.8f;  // scales the fake walls generated around disparties
  constexpr float kCarHalfWidth = 0.22f;  // m, bubble around each edge
  constexpr float kFov = 1.5708f;         // only consider +-90 deg
  constexpr float kMaxSteerAngle = 0.80f; // rad that maps to full lock
  constexpr float kCorridorHalf = 0.18f;  // m, width of the speed check strip
  constexpr float kSlowDist = 1.0f;       // m ahead => slow speed
  constexpr float kFastDist = 3.0f;       // m ahead => full speed
  constexpr float kSlowThrottle = 0.25f;  // 7500 eRPM at 30000 max, 5250 at full lock
  constexpr float kFastThrottle = 0.35f;  // 10500 eRPM at 30000 max

  std::vector<float> r; // reused buffer, no allocation after the first scan
}

DriveCommand computeDrive(const std::vector<float> &ranges, const vesc_msgs::msg::VescImuStamped *imu)
{
  (void)imu;
  DriveCommand cmd;
  const int n = static_cast<int>(ranges.size());
  if (n == 0)
    return cmd;

  // 1. clean
  r.resize(n);
  for (int i = 0; i < n; i++)
  {
    const float v = ranges[i];
    r[i] = (std::isfinite(v) && v > 0.0f) ? std::min(v, kMaxRange) : kMaxRange;
  }

  // 2. disparity extension: push the near edge outward over the far side
  const float inc = ind2angle(1) - ind2angle(0);
  for (int i = 0; i < n - 1; i++)
  {
    const float a = r[i], b = r[i + 1];
    if (std::abs(a - b) < kDisparity)
      continue;
    const float near = std::min(a, b);
    const int cover = static_cast<int>(std::ceil(std::atan2(kCarHalfWidth, near) / inc));
    if (a < b)
      for (int k = 1; k <= cover && i + k < n; k++)
        r[i + k] = std::min(r[i + k], near);
    else
      for (int k = 0; k < cover && i - k >= 0; k++)
        r[i - k] = std::min(r[i - k], near);
  }

  // 3. aim at the farthest beam inside the forward FOV,
  //    and measure clearance in a straight strip ahead
  int best = -1;
  float front = kMaxRange;
  for (int i = 0; i < n; i++)
  {
    const float ang = ind2angle(i);
    if (std::abs(ang) > kFov)
      continue;
    if (best < 0 || r[i] > r[best])
      best = i;
    const float x = ranges[i] * std::cos(ang), y = ranges[i] * std::sin(ang);
    if (std::isfinite(ranges[i]) && x > 0.0f && std::abs(y) < kCorridorHalf)
      front = std::min(front, x);
  }
  if (best < 0)
    return cmd;

  // 4. steer toward it, normalized to [-1, 1]
  cmd.steering = std::clamp(ind2angle(best) / kMaxSteerAngle, -1.0f, 1.0f);

  // 5. speed from clearance ahead, slowed further by hard steering
  const float t = std::clamp((front - kSlowDist) / (kFastDist - kSlowDist), 0.0f, 1.0f);
  cmd.throttle = (kSlowThrottle + t * (kFastThrottle - kSlowThrottle)) *
                 (1.0f - 0.3f * std::abs(cmd.steering));
  if (front < 0.4f)
    cmd.throttle = -1.0f; // about to hit something: brake
  return cmd;
}
