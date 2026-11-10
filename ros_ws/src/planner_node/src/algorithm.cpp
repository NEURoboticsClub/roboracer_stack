#include "algorithm.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

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

// ---- debug helpers ----
// Logger and clock are statics so they're built once, not every scan.
static const rclcpp::Logger &dbgLog()
{
  static const rclcpp::Logger l = rclcpp::get_logger("planner.algo");
  return l;
}
static rclcpp::Clock &dbgClock()
{
  static rclcpp::Clock c(RCL_STEADY_TIME);
  return c;
}

// Throttled INFO/WARN. Every DBG line is its own throttle slot, so you get one
// block of lines every debug_period_ms.
#define DBG(...)                                                                      \
  do {                                                                                \
    if (debug_log)                                                                    \
      RCLCPP_INFO_THROTTLE(dbgLog(), dbgClock(), debug_period_ms, __VA_ARGS__);       \
  } while (0)
#define DBG_WARN(...)                                                                 \
  do {                                                                                \
    if (debug_log)                                                                    \
      RCLCPP_WARN_THROTTLE(dbgLog(), dbgClock(), debug_period_ms, __VA_ARGS__);       \
  } while (0)

constexpr float rad2deg = 180.0f / static_cast<float>(M_PI);

// Which branch drove the car this scan. Logged when it changes.
static const char *const MODE_NONE = "NONE";
static const char *const MODE_DRIVE = "DRIVE";
static const char *const MODE_STOP_AHEAD = "STOP (ahead < stop_distance)";
static const char *const MODE_STOP_ALLROUND = "STOP (all-round < 0.3 m)";
static const char *const MODE_NO_SCAN = "STOP (bad/empty scan)";
static const char *mode = MODE_NONE;

// Raw reading of the beam nearest `angle` (rad). -1 if the scan doesn't reach it.
static float rawAt(const std::vector<float> &ranges, float angle)
{
  const int n = static_cast<int>(ranges.size());
  if (n < 2)
    return -1.0f;
  const float inc = ind2angle(1) - ind2angle(0);
  if (inc <= 0.0f)
    return -1.0f;
  const int i = static_cast<int>(std::lround((angle - ind2angle(0)) / inc));
  return (i >= 0 && i < n) ? ranges[i] : -1.0f;
}

// Everything about the raw scan worth knowing: beam counts, how many are
// no-returns, closest/farthest real hits, the front cone, and a handful of
// sample beams. One pass over the scan, no allocation.
static void debugScan(const std::vector<float> &ranges)
{
  if (!debug_log)
    return;

  // scan rate, smoothed
  static auto last = std::chrono::steady_clock::now();
  static float hz = 0.0f;
  const auto now = std::chrono::steady_clock::now();
  const float dt = std::chrono::duration<float>(now - last).count();
  last = now;
  if (dt > 0.0f)
    hz = hz <= 0.0f ? 1.0f / dt : 0.9f * hz + 0.1f / dt;

  const int n = static_cast<int>(ranges.size());
  if (n < 2)
  {
    DBG_WARN("scan: only %d beams - nothing to plan on", n);
    return;
  }

  int valid = 0, zero = 0, n_inf = 0, n_nan = 0;
  int min_i = -1, max_i = -1;
  int ahead_beams = 0, ahead_noret = 0, ahead_i = -1;
  for (int i = 0; i < n; i++)
  {
    const float r = ranges[i];
    const bool in_front = std::abs(ind2angle(i)) <= ahead_half_angle;
    if (in_front)
      ahead_beams++;

    if (std::isnan(r))
      n_nan++;
    else if (std::isinf(r))
      n_inf++;
    else if (r <= 0.0f)
      zero++;
    else
    {
      valid++;
      if (min_i < 0 || r < ranges[min_i])
        min_i = i;
      if (max_i < 0 || r > ranges[max_i])
        max_i = i;
      if (in_front && (ahead_i < 0 || r < ranges[ahead_i]))
        ahead_i = i;
      continue;
    }
    if (in_front)
      ahead_noret++;
  }

  DBG("scan: %d beams %+.0f..%+.0f deg (%.2f deg step) @ %.1f Hz | valid %d, zero %d, inf %d, nan %d",
      n, ind2angle(0) * rad2deg, ind2angle(n - 1) * rad2deg,
      (ind2angle(1) - ind2angle(0)) * rad2deg, hz, valid, zero, n_inf, n_nan);

  if (min_i >= 0)
    DBG("scan: closest %.3f m @ %+.1f deg (beam %d), farthest %.3f m @ %+.1f deg",
        ranges[min_i], ind2angle(min_i) * rad2deg, min_i,
        ranges[max_i], ind2angle(max_i) * rad2deg);
  else
    DBG_WARN("scan: NO valid returns at all (every beam is 0/inf/nan)");

  DBG("ahead +-%.0f deg: %d beams, %d no-return, closest real hit %.3f m @ %+.1f deg",
      ahead_half_angle * rad2deg, ahead_beams, ahead_noret,
      ahead_i >= 0 ? ranges[ahead_i] : INFINITY,
      ahead_i >= 0 ? ind2angle(ahead_i) * rad2deg : 0.0f);
  // the creep suspect: no-returns get skipped, so a wall too close to read
  // looks like an empty road
  if (ahead_beams > 0 && ahead_noret == ahead_beams)
    DBG_WARN("ahead: EVERY front beam is a no-return - brake sees inf and thinks it's clear");

  // sample beams, raw (nan/inf print as-is, -1 = outside the scan)
  static char buf[256];
  int len = 0;
  for (int deg = -90; deg <= 90; deg += 15)
  {
    const int w = std::snprintf(buf + len, sizeof(buf) - len, " %+d:%.2f", deg,
                                rawAt(ranges, deg / rad2deg));
    if (w < 0 || w >= static_cast<int>(sizeof(buf)) - len)
      break;
    len += w;
  }
  DBG("raw deg:m%s", buf);
}

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
  mode = MODE_NO_SCAN;
  const int n = static_cast<int>(ranges.size());
  if (n < 2)
    return cmd;
  const float inc = ind2angle(1) - ind2angle(0);
  if (inc <= 0.0f)
  {
    DBG_WARN("gap: angle_increment %.5f <= 0, stopping", inc);
    return cmd;
  }

  // 1. Clean.
  proc.resize(n);
  int cleaned = 0; // beams the clean step turned into "open at max_range"
  for (int i = 0; i < n; i++)
  {
    proc[i] = cleanRange(ranges[i]);
    if (proc[i] != ranges[i])
      cleaned++;
  }
  DBG("clean: %d/%d beams rewritten to %.1f m (no-return or past max_range)",
      cleaned, n, max_range);

  // 2. Bubble. Find the closest point ahead and blank out every beam that
  //    falls within bubble_radius of it, so the car can't aim anywhere it would
  //    clip that obstacle.
  int closest_i = -1;
  for (int i = 0; i < n; i++)
    if (std::abs(ind2angle(i)) <= look_half_angle && (closest_i < 0 || proc[i] < proc[closest_i]))
      closest_i = i;
  if (closest_i < 0)
  {
    DBG_WARN("gap: no beams within +-%.0f deg, stopping", look_half_angle * rad2deg);
    return cmd; // no forward beams at all
  }

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
  int target_i = -1;
  if (best_start >= 0)
  {
    target_i = best_start;
    for (int i = best_start; i < best_start + best_len; i++)
      if (proc[i] > proc[target_i] ||
          (proc[i] == proc[target_i] && std::abs(ind2angle(i)) < std::abs(ind2angle(target_i))))
        target_i = i;
    target_angle = ind2angle(target_i);
  }
  cmd.steering = std::clamp(target_angle / full_lock_angle, -1.0f, 1.0f);

  DBG("bubble: closest (cleaned) %.3f m @ %+.1f deg, blanks +-%.1f deg (%d beams each side)",
      closest, ind2angle(closest_i) * rad2deg, bubble_angle * rad2deg, cover);
  if (best_start >= 0)
    DBG("gap: widest %+.1f..%+.1f deg (%d beams) | aim %+.1f deg, %.2f m deep -> steer %+.2f",
        ind2angle(best_start) * rad2deg, ind2angle(best_start + best_len - 1) * rad2deg,
        best_len, target_angle * rad2deg, proc[target_i], cmd.steering);
  else
    DBG_WARN("gap: NO gap >= %.1f m within +-%.0f deg, holding straight",
             min_gap_dist, look_half_angle * rad2deg);

  // 4. Speed. Emergency brake first; steering still follows the gap so the car
  //    is pointed somewhere open.
  const float ahead = closestAhead(ranges);
  if (ahead <= stop_distance)
  {
    cmd.throttle = brake_throttle;
    mode = MODE_STOP_AHEAD;
    DBG_WARN("speed: STOP, ahead %.3f m <= stop_distance %.2f -> throttle %+.2f, steer %+.2f",
             ahead, stop_distance, cmd.throttle, cmd.steering);
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
  mode = MODE_DRIVE;

  // ahead = inf means no real return in front, NOT "measured far away"
  DBG("speed: ahead %.3f m%s | wall_t %.2f turn_t %.2f -> t %.2f -> throttle %+.2f",
      ahead, std::isinf(ahead) ? " (no return!)" : "", wall_t, turn_t, t, cmd.throttle);

  return cmd;
}

// Emergency brake if ANY beam (all the way round, not just in front) sees
// something closer than all_round_stop_distance. Otherwise follow the gap.
constexpr float all_round_stop_distance = 0.3f; // m, measured from the lidar
DriveCommand driveForward(const std::vector<float> &ranges)
{
  for (int i = 0; i < static_cast<int>(ranges.size()); i++)
  {
    const float r = ranges[i];
    // Skip no-returns (0, inf, nan): the real SICK reads 0 for "nothing there",
    // so counting 0 as close would brake forever.
    if (std::isfinite(r) && r > 0.0f && r < all_round_stop_distance)
    {
      mode = MODE_STOP_ALLROUND;
      DBG_WARN("all-round brake: beam %d @ %+.1f deg reads %.3f m < %.2f m -> throttle 0, steer 0",
               i, ind2angle(i) * rad2deg, r, all_round_stop_distance);
      return moveForward(0.0f); // emergency brake
    }
  }
  return followGap(ranges); // checked every beam, nothing too close
}

// Called once per scan. Pick which algorithm drives the car.
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  debugScan(ranges);
  const char *prev_mode = mode;

  //no emergency brake, just follow the gap
  //return followGap(ranges);


  //uses emergency brake if anything is too close, otherwise follow the gap
  const DriveCommand cmd = driveForward(ranges);

  // not throttled: you always see the moment it starts or stops braking
  if (debug_log && mode != prev_mode)
    RCLCPP_WARN(dbgLog(), "mode: %s -> %s (throttle %+.2f, steer %+.2f)",
                prev_mode, mode, cmd.throttle, cmd.steering);
  DBG("cmd: throttle %+.3f steer %+.3f [%s]", cmd.throttle, cmd.steering, mode);
  return cmd;
}
