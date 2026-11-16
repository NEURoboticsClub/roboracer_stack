#include "algorithm.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>

// ===========================================================================
// Gap follow (disparity extender) + adaptive corner speed.
//
// Steering: aim at the farthest open point the car can actually fit through.
// Speed:    slow down BEFORE a corner (as soon as the open space swings to one
//           side), and learn from every corner. A corner with lots of room
//           means the next one can be a bit faster; a corner that got too close
//           to a wall means slow down and remember that speed as a ceiling.
// ===========================================================================
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  // --- Speed settings ---
  const float STRAIGHT_THROTTLE = 0.70f; // throttle on clear straights
  const float SLOW_THROTTLE     = 0.35f; // slowest moving throttle (real car needs >= ~0.33)
  const float BRAKE_THROTTLE    = -0.5f; // braking strength when about to hit something
  const float SLOW_DOWN_DIST    = 3.0f;  // meters: start slowing when something is this close ahead
  const float MIN_SPEED_DIST    = 0.8f;  // meters: be at slowest speed by here
  const float STOP_DIST         = 0.3f;  // meters: brake if closer than this
  const float PATH_HALF_WIDTH   = 0.18f; // meters: half the car's width plus a small margin

  // --- Corner speed learning ---
  const float CORNER_START      = 0.40f; // first-lap corner throttle (safe)
  const float CORNER_MIN        = 0.35f; // never go slower than this in a corner
  const float CORNER_MAX        = 0.65f; // never go faster than this in a corner
  const float CORNER_STEP_UP    = 0.03f; // speed added after a clean corner
  const float CORNER_STEP_DOWN  = 0.08f; // speed removed after a close call
  const float SAFE_CLEARANCE    = 0.40f; // meters: closer than this is not "clean"
  const float DANGER_CLEARANCE  = 0.25f; // meters: closer than this is a close call
  const float HIT_CLEARANCE     = 0.15f; // meters: basically touching the wall
  const float CORNER_ANGLE      = 0.15f; // radians: target this far off-center = corner coming

  // --- Steering settings ---
  const float CAR_HALF_WIDTH   = 0.45f; // meters: how far to widen corner edges (bigger = wider turns)
  const float DISPARITY_THRESH = 0.30f; // meters: jump between neighbor beams that counts as a corner edge
  const float FOV_HALF         = 1.57f; // radians: only aim within +/-90 deg of straight ahead
  const float MAX_RANGE        = 10.0f; // meters: treat "no return" as this far
  const float MAX_STEER        = 0.52f; // radians: car's steering limit (~30 deg)
  const float STEER_GAIN       = 1.2f;  // how hard to turn toward the target
  const float STEER_SIGN       = 1.0f;  // set to -1.0f if the car turns the WRONG way
  const float STEER_SMOOTH     = 0.7f;  // 0..1: lower = smoother but reacts later
  const float ESCAPE_CONE      = 0.10f; // radians: width of the "straight at target" check
  const float ESCAPE_CLEAR     = 1.0f;  // meters: if the way to the target is this clear, keep moving

  // Remembered between scans.
  static float lastDist = INFINITY;
  static float lastSteer = 0.0f;
  static int scanCount = 0;
  static float cornerThrottle = CORNER_START;
  static float cornerCeiling = CORNER_MAX;
  static bool cornerActive = false;
  static bool cornerPunished = false;
  static int cornerFrames = 0;
  static int exitFrames = 0;
  static float cornerMinClear = 10.0f;
  static int cornerCount = 0;

  const int n = static_cast<int>(ranges.size());
  if (n < 2)
  {
    DriveCommand stop;
    stop.steering = 0.0f;
    stop.throttle = 0.0f;
    return stop;
  }

  // 1. Clean the scan: invalid readings (0, inf, nan, too close) count as open space.
  std::vector<float> clean(n);
  for (int i = 0; i < n; i++)
  {
    float r = ranges[i];
    if (!std::isfinite(r) || r < 0.10f)
      r = MAX_RANGE;
    clean[i] = std::min(r, MAX_RANGE);
  }

  // Closest wall around the front of the car (used to judge each corner).
  float nearest = MAX_RANGE;
  for (int i = 0; i < n; i++)
    if (std::fabs(ind2angle(i)) <= 1.75f)
      nearest = std::min(nearest, clean[i]);

  // 2. Disparity extender: widen every corner edge by the car's half width so
  //    the car doesn't aim at a gap right next to a corner and clip it.
  const float angleStep = std::fabs(ind2angle(1) - ind2angle(0));
  std::vector<float> ext = clean;
  for (int i = 0; i < n - 1; i++)
  {
    float a = clean[i];
    float b = clean[i + 1];
    if (std::fabs(a - b) < DISPARITY_THRESH)
      continue;
    float closer = std::min(a, b);
    int beams = static_cast<int>(std::ceil(std::atan2(CAR_HALF_WIDTH, closer) / angleStep));
    if (a < b)
      for (int j = i + 1; j <= std::min(n - 1, i + beams); j++)
        ext[j] = std::min(ext[j], closer);
    else
      for (int j = i; j >= std::max(0, i + 1 - beams); j--)
        ext[j] = std::min(ext[j], closer);
  }

  // 3. Aim at the farthest reachable point in front.
  int best = -1;
  float bestDist = -1.0f;
  for (int i = 0; i < n; i++)
  {
    if (std::fabs(ind2angle(i)) > FOV_HALF)
      continue;
    if (ext[i] > bestDist)
    {
      bestDist = ext[i];
      best = i;
    }
  }
  float targetAngle = (best >= 0) ? ind2angle(best) : 0.0f;

  float steer = STEER_SIGN * STEER_GAIN * targetAngle;
  steer = std::max(-MAX_STEER, std::min(MAX_STEER, steer));
  steer = lastSteer + STEER_SMOOTH * (steer - lastSteer);
  lastSteer = steer;

  // 4. Closest thing in the lane the car is actually heading into.
  const float heading = STEER_SIGN * steer;
  float pathDist = INFINITY;
  for (int i = 0; i < n; i++)
  {
    float r = ranges[i];
    if (!std::isfinite(r) || r < 0.10f)
      continue;
    float rel = ind2angle(i) - heading;
    float ahead = r * std::cos(rel);
    float sideways = r * std::sin(rel);
    if (ahead <= 0.0f || std::fabs(sideways) > PATH_HALF_WIDTH)
      continue;
    pathDist = std::min(pathDist, ahead);
  }

  float targetClear = INFINITY;
  for (int i = 0; i < n; i++)
    if (std::fabs(ind2angle(i) - targetAngle) <= ESCAPE_CONE)
      targetClear = std::min(targetClear, clean[i]);

  // 5. Corner learning. A corner = the open space has swung to one side.
  bool cornering = std::fabs(targetAngle) > CORNER_ANGLE || std::fabs(steer) > CORNER_ANGLE;
  if (cornering)
  {
    if (!cornerActive)
    {
      cornerActive = true;
      cornerPunished = false;
      cornerFrames = 0;
      cornerMinClear = 10.0f;
    }
    cornerFrames++;
    exitFrames = 0;
    cornerMinClear = std::min(cornerMinClear, nearest);

    // Basically touching the wall: slow down right now, don't wait for the corner to end.
    if (!cornerPunished && nearest < HIT_CLEARANCE)
    {
      cornerPunished = true;
      cornerCeiling = std::max(CORNER_MIN, cornerThrottle - 0.02f);
      cornerThrottle = std::max(CORNER_MIN, cornerThrottle - CORNER_STEP_DOWN);
      std::fprintf(stderr, "[corner] nearly hit the wall (%.2f m) - corner speed down to %.2f (cap %.2f)\n",
                   nearest, cornerThrottle, cornerCeiling);
    }
  }
  else if (cornerActive && ++exitFrames >= 8) // 8 straight scans in a row = corner is over
  {
    cornerActive = false;
    if (cornerFrames >= 10 && !cornerPunished)
    {
      cornerCount++;
      if (cornerMinClear < DANGER_CLEARANCE)
      {
        cornerCeiling = std::max(CORNER_MIN, cornerThrottle - 0.02f);
        cornerThrottle = std::max(CORNER_MIN, cornerThrottle - CORNER_STEP_DOWN);
        std::fprintf(stderr, "[corner %d] too close (%.2f m) - corner speed down to %.2f (cap %.2f)\n",
                     cornerCount, cornerMinClear, cornerThrottle, cornerCeiling);
      }
      else if (cornerMinClear > SAFE_CLEARANCE)
      {
        cornerThrottle = std::min(cornerThrottle + CORNER_STEP_UP, cornerCeiling);
        std::fprintf(stderr, "[corner %d] clean (%.2f m) - corner speed now %.2f\n",
                     cornerCount, cornerMinClear, cornerThrottle);
      }
      else
      {
        std::fprintf(stderr, "[corner %d] ok (%.2f m) - holding corner speed %.2f\n",
                     cornerCount, cornerMinClear, cornerThrottle);
      }
    }
  }

  // 6. Throttle from distance ahead.
  float throttle;
  if (pathDist >= SLOW_DOWN_DIST)
  {
    throttle = STRAIGHT_THROTTLE;
  }
  else if (pathDist > MIN_SPEED_DIST)
  {
    float t = (pathDist - MIN_SPEED_DIST) / (SLOW_DOWN_DIST - MIN_SPEED_DIST);
    throttle = SLOW_THROTTLE + t * (STRAIGHT_THROTTLE - SLOW_THROTTLE);
  }
  else if (pathDist > STOP_DIST)
  {
    throttle = SLOW_THROTTLE;
  }
  else
  {
    bool stillClosing = pathDist < lastDist - 0.005f;
    if (stillClosing)
      throttle = BRAKE_THROTTLE;
    else if (targetClear > ESCAPE_CLEAR)
      throttle = SLOW_THROTTLE; // stopped but the way out is open: creep out
    else
      throttle = 0.0f;
  }
  lastDist = pathDist;

  // 7. In or approaching a corner: never faster than the learned corner speed.
  if (cornering && throttle > 0.0f)
    throttle = std::min(throttle, cornerThrottle);

  scanCount++;
  if (scanCount % 20 == 0)
  {
    std::fprintf(stderr,
                 "[algorithm] target=%.2f steer=%.2f path=%.2f m nearest=%.2f m %s corner_speed=%.2f throttle=%.2f\n",
                 targetAngle, steer, pathDist, nearest, cornering ? "CORNER" : "straight",
                 cornerThrottle, throttle);
  }

  DriveCommand cmd;
  cmd.steering = steer;
  cmd.throttle = throttle;
  return cmd;
}