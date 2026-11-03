#include "algorithm.hpp"
#include <algorithm>
#include <cmath>

// ===========================================================================
// YOUR ALGORITHM GOES HERE. This is the only file you need to edit.
//
// `ranges` is one lidar scan: ranges[i] is the distance in meters seen by beam
// i, at angle ind2angle(i). The beam count differs between the sim and the real
// car, so use ind2angle() rather than hard-coding indices.
//
// The scan is passed through raw: a beam with no return can read 0, inf or nan.
//
// To keep state between scans, use static variables in this file. The scan
// callback waits for this function, so a slow algorithm lowers the command rate.
//
// See algorithm.hpp for what the returned command means.
// ===========================================================================

// Follow-the-gap with a safety bubble:
//   1. clean the scan
//   2. bubble: blank out everything within bubble_radius of the closest point
//   3. find the widest open stretch left and steer at its deepest point
//   4. speed: slow down linearly near a wall or in a hard turn, emergency brake
//      if anything is within stop_distance

// ---- speed ----
// Throttle is a fraction of sim_max_throttle (0.25 in sim_launch.py), so 0.4 is
// 0.1 in the sim. Real car: the VESC won't move below ~0.85 (5000 of 6000 eRPM).
constexpr float max_throttle = 0.4f;      // wide open, going straight
constexpr float min_throttle = 0.2f;      // right above stop_distance, or full lock
constexpr float slowdown_distance = 2.0f; // m: start easing off this far from a wall

// ---- emergency brake: change these to tune it ----
// Wall closer than this in front => stop. Meters, measured from the lidar.
constexpr float stop_distance = 0.4f;
// What "stop" sends. 0.0 = cut throttle, the car rolls to a stop. -1.0 = full
// brake on the real car (4 A), but in the sim negative is REVERSE, so keep 0.0
// there or it backs away and drives back in.
constexpr float brake_throttle = 0.0f;
// How wide "in front" is: beams within +-this of straight ahead. 0.35 rad is
// ~20 deg, about the car's width at 0.4 m.
constexpr float ahead_half_angle = 0.35f;

static_assert(slowdown_distance > stop_distance, "slowdown_distance must be past stop_distance");

// ---- steering ----
constexpr float bubble_radius = 0.30f;   // m: car half-width 0.15 + margin
constexpr float min_gap_dist = 1.0f;     // m: beams shorter than this aren't "open"
constexpr float look_half_angle = 1.57f; // only steer toward +-90 deg
constexpr float full_lock_angle = 0.80f; // heading (rad) that maps to full lock
constexpr float max_range = 10.0f;       // sim lidar range

// Working copy of the scan, cleaned then bubbled. Kept between scans so it
// doesn't reallocate every frame.
static std::vector<float> proc;

// Straight ahead at `throttle` (0..1).
DriveCommand moveForward(float throttle)
{
  DriveCommand cmd;
  cmd.steering = 0.0f;
  cmd.throttle = throttle;
  return cmd;
}

// Range used for steering. No return (0, inf, nan) means nothing out to max
// range, so open. Treating it as a 0 m wall would put the bubble on it and
// block everything.
float cleanRange(float r)
{
  if (!std::isfinite(r) || r <= 0.0f || r > max_range)
    return max_range;
  return r;
}

// Closest real return within +-ahead_half_angle of straight ahead. Skips
// no-return beams (0, inf, nan). inf if nothing in front returned.
float closestAhead(const std::vector<float> &ranges)
{
  float closest = INFINITY;
  for (int i = 0; i < static_cast<int>(ranges.size()); i++)
  {
    const float r = ranges[i];
    if (std::abs(ind2angle(i)) <= ahead_half_angle && std::isfinite(r) && r > 0.0f)
      closest = std::min(closest, r);
  }
  return closest;
}

// Follow-the-gap with bubble, linear slowdown and emergency brake (steps 1-4
// at the top of the file).
DriveCommand followGap(const std::vector<float> &ranges)
{
  DriveCommand cmd; // straight, stopped
  const int n = static_cast<int>(ranges.size());
  if (n < 2)
    return cmd;
  const float inc = ind2angle(1) - ind2angle(0);
  if (inc <= 0.0f)
    return cmd;

  // 1. Clean.
  proc.resize(n);
  for (int i = 0; i < n; i++)
    proc[i] = cleanRange(ranges[i]);

  // 2. Bubble. Find the closest point ahead and blank out every beam that
  //    falls within bubble_radius of it, so the car can't aim anywhere it would
  //    clip that obstacle.
  int closest_i = -1;
  for (int i = 0; i < n; i++)
    if (std::abs(ind2angle(i)) <= look_half_angle && (closest_i < 0 || proc[i] < proc[closest_i]))
      closest_i = i;
  if (closest_i < 0)
    return cmd; // no forward beams at all

  // Angle the bubble covers each side at that distance (90 deg if we're inside it).
  const float closest = proc[closest_i];
  const float bubble_angle = closest > bubble_radius
    ? std::asin(bubble_radius / closest)
    : static_cast<float>(M_PI) / 2.0f;
  const int cover = static_cast<int>(std::ceil(bubble_angle / inc));
  for (int i = std::max(0, closest_i - cover); i <= std::min(n - 1, closest_i + cover); i++)
    proc[i] = 0.0f;

  // 3. Widest gap: longest run of forward beams that see at least min_gap_dist.
  int best_start = -1, best_len = 0, run_start = -1;
  for (int i = 0; i <= n; i++) // one past the end closes a run still open at the last beam
  {
    const bool open = i < n && std::abs(ind2angle(i)) <= look_half_angle && proc[i] >= min_gap_dist;
    if (open)
    {
      if (run_start < 0)
        run_start = i;
      continue;
    }
    if (run_start >= 0 && i - run_start > best_len)
    {
      best_len = i - run_start;
      best_start = run_start;
    }
    run_start = -1;
  }

  // Aim at the deepest point in that gap. Ties go to the straightest, so a
  // wide-open view drives straight instead of swinging to one side.
  float target_angle = 0.0f; // no gap: hold straight, the brake below handles it
  if (best_start >= 0)
  {
    int target_i = best_start;
    for (int i = best_start; i < best_start + best_len; i++)
      if (proc[i] > proc[target_i] ||
          (proc[i] == proc[target_i] && std::abs(ind2angle(i)) < std::abs(ind2angle(target_i))))
        target_i = i;
    target_angle = ind2angle(target_i);
  }
  cmd.steering = std::clamp(target_angle / full_lock_angle, -1.0f, 1.0f);

  // 4. Speed. Emergency brake first; steering still follows the gap so the car
  //    is pointed somewhere open.
  const float ahead = closestAhead(ranges);
  if (ahead <= stop_distance)
  {
    cmd.throttle = brake_throttle;
    return cmd;
  }

  // Wall: 0 at stop_distance, rising linearly to 1 at slowdown_distance.
  const float wall_t = std::clamp(
    (ahead - stop_distance) / (slowdown_distance - stop_distance), 0.0f, 1.0f);
  // Turn: 1 going straight, falling linearly to 0 at full lock.
  const float turn_t = 1.0f - std::abs(cmd.steering);
  // Whichever wants it slower wins, then map 0..1 onto min..max throttle.
  const float t = std::min(wall_t, turn_t);
  cmd.throttle = min_throttle + t * (max_throttle - min_throttle);

  return cmd;
}

// Emergency brake if ANY beam (all the way round, not just in front) sees
// something closer than half the car's length. Otherwise follow the gap.
constexpr float car_length = 0.6f; // m, bumper to bumper - measure yours
DriveCommand driveForward(const std::vector<float> &ranges)
{
  for (const float r : ranges)
  {
    // Skip no-returns (0, inf, nan): the real SICK reads 0 for "nothing there",
    // so counting 0 as close would brake forever.
    if (std::isfinite(r) && r > 0.0f && r < car_length / 2.0f)
      return moveForward(0.0f); // emergency brake
  }
  return followGap(ranges); // checked every beam, nothing too close
}

// Called once per scan. Pick which algorithm drives the car.
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  //no emergency brake, just follow the gap
  //return followGap(ranges);


  //uses emergency brake if anything is too close, otherwise follow the gap
  return driveForward(ranges);
}
