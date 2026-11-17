#include "algorithm.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"


namespace
{
constexpr float kDisparity = 0.4f;        // m jump between neighbouring beams = an edge. Lower = bumpy walls sprout bubbles
constexpr float kMaxBubbleAngle = 0.35f;  // rad cap so a close edge can't blank out the world
constexpr float kForwardLimit = 1.5708f;  // only consider gaps within +-90 deg
constexpr float kDepthTie = 0.10f;        // gap depths this close are a tie, broken by width
constexpr float kClosingSpeedGain = 0.4f; // low-pass filter gain on closing speed
constexpr float kMaxClosingSpeed = 10.0f; // m/s; faster than this is a new obstacle popping in, not motion
constexpr float kBrakeDeadband = 0.3f;    // m/s over the cap the VESC's own speed loop gets to fix before we brake
constexpr float kBrakeStep = 0.2f;        // max brake added per scan (15 Hz): full in ~1/3 s, no jolt
constexpr float kBrakeRelease = 0.5f;     // let go once the decel we need drops under this x brake_decel
constexpr float kSlideMargin = 1.0f;      // m/s the lidar has to beat the wheels by to call it a skid
constexpr float kDegToRad = 3.14159265f / 180.0f;


// ---- emergency brake ----
constexpr float close_stop_distance = 0.3f; // m from the lidar: closer than this = stop
constexpr float ahead_half_angle = 0.3f;   // rad (~20 deg) each side of straight ahead
// 0 = cut throttle and roll to a stop. In the sim negative is REVERSE, so keep 0.
constexpr float stop_throttle = 0.0f;


struct Tuning
{
 // Gap selection
 float bubble_radius;  // m padded around every near edge
 float min_gap_depth;  // m an opening must see to count
 float min_gap_width;  // m chord the car must fit through
 float deep_band;      // aim at the deepest band within this of the gap's max


 // Steering
 float max_steer_angle;      // rad of heading that maps to full lock
 float corner_lock_fraction; // steering past this fraction forces corner speed
 float side_clearance;       // m; > 0 vetoes turning into a wall alongside the car


 // Clearance ahead
 float front_cone_half_angle; // rad
 float corridor_half_width;   // m; car half-width (0.15) + margin, never below 0.15
 float corner_distance;       // m clearance at or under which we hold corner speed
 float slowdown_distance;     // m; > corner_distance ramps cruise down toward corner speed


 // Closing-speed brake, with hysteresis between engage and release
 float brake_activation_distance; // m
 float brake_engage_speed;        // m/s toward the obstacle
 float brake_release_speed;       // m/s toward the obstacle
 // Speed governor (real car): cap speed at what can still slow to creep_speed
 // by brake_margin, and only brake if we're over that cap anyway.
 float brake_decel;      // m/s^2 the cap slows us at. Lower = eases off earlier and gentler. 0 = off
 float brake_latency;    // s from the scan to the VESC actually reacting
 float brake_margin;     // m from the lidar where we're down to creep_speed
 float creep_speed;      // m/s we can always crawl at to steer out. VESC won't hold much under ~1.1
 float brake_full_decel; // m/s^2 the car does at full brake. Lower = brakes harder for the same need


 // Normalized throttle (see DriveCommand)
 float cruise_throttle;
 float corner_throttle;
 float brake_throttle; // braking state. Real car: negative = brake current. Sim: negative is reverse
};


// Real car. Throttle 1.0 = irl_max_throttle_erpm (30000) in irl_launch.py.
constexpr float kIrlMaxErpm = 50000.0f;
constexpr Tuning kIrl{
 0.15f, 0.8f, 0.40f, 0.20f,             // bubble, gap depth, gap width, deep band
 0.80f, 0.30f, 0.0f,                    // max steer, corner lock, side clearance
 20.0f * kDegToRad, 0.18f, 2.0f, 2.0f,  // cone, corridor, corner dist, slowdown dist
 3.0f, 5.0f, 0.2f,                      // brake activation, engage, release
 3.0f, 0.15f, 0.5f, 1.1f, 5.0f,         // governor: decel, latency, margin, creep, full-brake decel
 50000.0f / kIrlMaxErpm, 10000.0f / kIrlMaxErpm, // cruise, corner
 -0.2f,                                 // brake: full irl_max_brake_amps
};


// AutoDRIVE sim. Throttle 1.0 = sim_max_throttle (0.25) in sim_launch.py.
// Bigger bubble and side check stop corner cutting; long slowdown ramp and a
// late, high brake because the sim car only sheds speed by drag.
constexpr float kSimMaxThrottle = 0.25f;
constexpr Tuning kSim{
 0.35f, 0.8f, 0.40f, 0.20f,             // bubble, gap depth, gap width, deep band
 0.80f, 0.30f, 0.25f,                   // max steer, corner lock, side clearance
 20.0f * kDegToRad, 0.18f, 2.0f, 6.0f,  // cone, corridor, corner dist, slowdown dist
 2.0f, 2.0f, 1.5f,                      // brake activation, engage, release
 0.0f, 0.15f, 0.5f, 0.0f, 5.0f,         // governor: off in the sim
 0.06f / kSimMaxThrottle, 0.048f / kSimMaxThrottle, 0.05f / kSimMaxThrottle, // cruise, corner, brake
};


const Tuning *tune = &kIrl;


struct Gap
{
 int first, last;
 float depth; // deepest reading inside
 float width; // m chord between the edge beams
};


DebugInfo debug;
std::vector<float> clean;                   // cleaned scan
std::vector<float> &padded = debug.ranges;  // clean scan with disparity bubbles applied
std::vector<char> &blocked = debug.blocked; // beam lies inside a bubble
std::vector<Gap> gaps;


void cleanScan(const std::vector<float> &ranges)
{
 clean.resize(ranges.size());
 for (size_t i = 0; i < ranges.size(); i++)
 {
   const float v = ranges[i];
   clean[i] = (std::isfinite(v) && v > 0.0f && v <= kMaxRange) ? v : kMaxRange;
 }
}


// Edges are detected on the clean scan so bubbles don't cascade into each other.
void extendDisparities()
{
 const int n = static_cast<int>(clean.size());
 const float inc = ind2angle(1) - ind2angle(0);
 const int max_cover = static_cast<int>(kMaxBubbleAngle / inc);
 padded = clean;
 blocked.assign(n, 0);
 debug.edges.clear();


 for (int i = 0; i + 1 < n; i++)
 {
   const float a = clean[i], b = clean[i + 1];
   if (std::abs(a - b) <= kDisparity)
     continue;
   debug.edges.push_back({i, a, b});


   const float near = std::min(a, b);
   const int cover = std::min(
     max_cover,
     static_cast<int>(std::ceil(std::atan2(tune->bubble_radius, std::max(near, 0.01f)) / inc)));


   // Extend the near side over the far side.
   const int from = a < b ? i + 1 : i - cover + 1;
   const int to = a < b ? i + cover : i;
   for (int j = std::max(from, 0); j <= std::min(to, n - 1); j++)
   {
     padded[j] = std::min(padded[j], near);
     blocked[j] = 1;
   }
 }
}


float chordWidth(int i0, int i1)
{
 const float a = padded[i0], b = padded[i1];
 const float sweep = ind2angle(i1) - ind2angle(i0);
 return std::sqrt(std::max(0.0f, a * a + b * b - 2.0f * a * b * std::cos(sweep)));
}


// Every un-bubbled run of beams in the forward cone that sees at least min_depth.
void collectGaps(float min_depth)
{
 gaps.clear();
 const int n = static_cast<int>(padded.size());
 int start = -1;
 float depth = 0.0f;


 for (int i = 0; i <= n; i++)
 {
   const bool open = i < n && std::abs(ind2angle(i)) < kForwardLimit && !blocked[i] &&
                     padded[i] >= min_depth;
   if (open)
   {
     depth = start < 0 ? padded[i] : std::max(depth, padded[i]);
     if (start < 0)
       start = i;
   }
   else if (start >= 0)
   {
     gaps.push_back({start, i - 1, depth, chordWidth(start, i - 1)});
     start = -1;
   }
 }
}


// Deepest gap at least min_width across. Depth is quantized so near-equal gaps
// (e.g. one opening split by noise at a corner) are decided by width instead of flickering.
const Gap *pickGap(float min_width)
{
 const Gap *best = nullptr;
 for (const Gap &g : gaps)
 {
   if (g.width < min_width)
     continue;
   if (!best)
   {
     best = &g;
     continue;
   }
   const float qg = std::floor(g.depth / kDepthTie);
   const float qb = std::floor(best->depth / kDepthTie);
   if (qg > qb || (qg == qb && g.width > best->width))
     best = &g;
 }
 return best;
}


// Middle of the longest band within deep_band of the gap's deepest reading.
// Aiming at the deep end rather than the gap midpoint is what makes the car turn in at wide corners.
int aimIndex(const Gap &gap)
{
 const float floor = gap.depth - tune->deep_band;
 int best_start = -1, best_end = -1, best_len = 0, run = -1;


 for (int i = gap.first; i <= gap.last + 1; i++)
 {
   if (i <= gap.last && padded[i] >= floor)
   {
     if (run < 0)
       run = i;
   }
   else if (run >= 0)
   {
     if (i - run > best_len)
     {
       best_len = i - run;
       best_start = run;
       best_end = i - 1;
     }
     run = -1;
   }
 }
 return best_start >= 0 ? (best_start + best_end) / 2 : (gap.first + gap.last) / 2;
}


float chooseHeading()
{
 collectGaps(tune->min_gap_depth);
 const Gap *gap = pickGap(tune->min_gap_width);
 if (!gap)
   gap = pickGap(0.0f); // nothing we fit through: take the most open direction anyway
 if (!gap)
 {
   collectGaps(0.0f); // everything is shallow: drop the depth floor
   gap = pickGap(0.0f);
 }
 return ind2angle(gap ? aimIndex(*gap) : static_cast<int>(padded.size()) / 2);
}


// The body pivots behind the lidar, so turning toward a wall that's already
// alongside drags the flank into it. Hold straight until it's cleared.
float applySideCheck(float heading)
{
 if (tune->side_clearance <= 0.0f)
   return heading;


 bool left_close = false, right_close = false;
 for (size_t i = 0; i < clean.size(); i++)
 {
   const float ang = ind2angle(static_cast<int>(i));
   const float forward = clean[i] * std::cos(ang);
   const float lateral = clean[i] * std::sin(ang);
   if (forward < -0.40f || forward > 0.15f || std::abs(lateral) > tune->side_clearance)
     continue;
   (lateral > 0.0f ? left_close : right_close) = true;
 }


 if ((heading > 0.0f && left_close) || (heading < 0.0f && right_close))
   return 0.0f;
 return heading;
}


// Nearest obstacle inside the car's straight-ahead corridor. Walls off to the
// side don't count. Uses the clean scan since bubbles would fake obstructions.
float frontClearance(bool &has_reading)
{
 float clearance = kMaxRange;
 has_reading = false;
 for (size_t i = 0; i < clean.size(); i++)
 {
   const float ang = ind2angle(static_cast<int>(i));
   if (std::abs(ang) > tune->front_cone_half_angle)
     continue;
   has_reading = true;


   const float forward = clean[i] * std::cos(ang);
   const float lateral = clean[i] * std::sin(ang);
   if (forward > 0.0f && std::abs(lateral) <= tune->corridor_half_width)
     clearance = std::min(clearance, forward);
 }
 return clearance;
}


struct ClosingBrake
{
 bool have_prev = false;
 float prev_clearance = 0.0f;
 double prev_time = 0.0;
 float speed = 0.0f; // filtered m/s toward the obstacle, from the lidar
 float v = 0.0f;     // speed the brakes go off: wheels if we have them, else lidar
 bool active = false;


 bool update(float clearance, double now, float wheel_speed)
 {
   const double dt = now - prev_time;
   if (!have_prev || dt > 0.001) // skip duplicate stamps, keep the old baseline
   {
     if (have_prev && dt < 1.0)
     {
       // A jump this big is the nearest point in the corridor changing (new
       // obstacle, or one left), not the car moving. Just rebaseline.
       const float raw = (prev_clearance - clearance) / static_cast<float>(dt);
       if (std::abs(raw) <= kMaxClosingSpeed)
         speed += kClosingSpeedGain * (raw - speed);
     }
     else if (have_prev)
     {
       speed = 0.0f; // stale baseline
       active = false;
     }
     prev_clearance = clearance;
     prev_time = now;
     have_prev = true;
   }


   // Wheels are instant, the filtered lidar speed lags (reads high while slowing,
   // which used to fake an overspeed and kick the brake on). So wheels first;
   // the lidar only wins if it's way faster, i.e. the wheels locked and we're sliding.
   v = wheel_speed < 0.0f ? speed : (speed > wheel_speed + kSlideMargin ? speed : wheel_speed);


   if (clearance <= tune->brake_activation_distance && v >= tune->brake_engage_speed)
     active = true;
   else if (v <= tune->brake_release_speed || clearance > tune->brake_activation_distance)
     active = false;
   return active;
 }
} closing_brake;


// Real car speed governor.
//  - Cap speed at v_safe, the most we can still bleed down to creep_speed by
//    brake_margin at a gentle brake_decel. The cap drops smoothly as a wall gets
//    closer, so the VESC's speed loop just eases off. That's the gentle part.
//  - If the car's over the cap anyway, brake with whatever decel it now takes
//    to make the margin, as a fraction of full brake. Ramped in so it doesn't
//    jolt, unless it needs everything right now. Once on it rides out the
//    whole slowdown instead of pulsing on and off.
struct Governor
{
 bool on = false;     // braking (vs just capping speed)
 float brake = 0.0f;  // 0..1, last scan's brake
 float v_safe = 0.0f; // for the log


 float apply(float throttle, float clearance, float v)
 {
   const Tuning &t = *tune;
   const float c = t.creep_speed;
   const float room = clearance - t.brake_margin - v * t.brake_latency;
   v_safe = std::sqrt(c * c + 2.0f * t.brake_decel * std::max(0.0f, room));


   // decel that still gets us down to creep speed by the margin
   const float need = v <= c ? 0.0f
     : room > 0.05f ? (v * v - c * c) / (2.0f * room)
     : INFINITY;


   if (!on)
     on = v > v_safe + kBrakeDeadband;
   else if (need < kBrakeRelease * t.brake_decel)
     on = false; // steered clear, or slowed right down


   if (!on)
   {
     brake = 0.0f;
     return std::min(throttle, speedToThrottle(v_safe));
   }
   const float target = std::min(1.0f, need / t.brake_full_decel);
   brake = target >= 1.0f ? 1.0f : std::min(target, brake + kBrakeStep);
   return std::min(throttle, -brake * std::abs(t.brake_throttle));
 }
} governor;


// Time from the scan stamp so bursty delivery (the sim bridge) doesn't fake closing speed.
double scanTimeOrNow()
{
 const double stamp = scanTime();
 if (stamp > 0.0)
   return stamp;
 return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}


float throttleFor(bool has_reading, bool braking, float clearance, float heading)
{
 const Tuning &t = *tune;
 if (!has_reading)
   return 0.0f; // coast
 if (braking)
   return t.brake_throttle;


 const bool hard_steer = std::abs(heading) / t.max_steer_angle >= t.corner_lock_fraction;
 if (clearance <= t.corner_distance || hard_steer)
   return t.corner_throttle;


 if (t.slowdown_distance <= t.corner_distance)
   return t.cruise_throttle;
 const float ramp = std::clamp(
   (clearance - t.corner_distance) / (t.slowdown_distance - t.corner_distance), 0.0f, 1.0f);
 return t.corner_throttle + ramp * (t.cruise_throttle - t.corner_throttle);
}
} // namespace


DriveCommand computeDrive(const std::vector<float> &ranges, const vesc_msgs::msg::VescImuStamped *imu)
{
 (void)imu;
 DriveCommand cmd;
 if (ranges.empty())
   return cmd;


 tune = simMode() ? &kSim : &kIrl;


 cleanScan(ranges);
 extendDisparities();
 const float heading = applySideCheck(chooseHeading());
 debug.heading = heading;


 bool has_reading = false;
 const float clearance = frontClearance(has_reading);
 bool braking = false;
 if (has_reading)
   braking = closing_brake.update(clearance, scanTimeOrNow(), wheelSpeed());
 else
 {
   closing_brake = ClosingBrake{};
   governor = Governor{};
 }


 cmd.steering = std::clamp(heading / tune->max_steer_angle, -1.0f, 1.0f);
 cmd.throttle = throttleFor(has_reading, braking, clearance, heading);
 if (has_reading && tune->brake_decel > 0.0f)
   cmd.throttle = governor.apply(cmd.throttle, clearance, closing_brake.v);


 // Emergency brake: any real return within +-ahead_half_angle closer than
 // close_stop_distance cuts the throttle. Uses the raw scan, not the cleaned one.
 bool estop = false;
 for (int i = 0; i < static_cast<int>(ranges.size()) && !estop; i++)
 {
   if (std::abs(ind2angle(i)) > ahead_half_angle)
     continue;
   const float r = ranges[i];
   estop = std::isfinite(r) && r > 0.0f && r < close_stop_distance;
 }
 if (estop)
   cmd.throttle = -1.0f; // steering still follows the gap


 // One line when a brake starts and one when it ends, with how hard it got.
 static bool was_braking = false;
 static float peak = 0.0f;
 static double started = 0.0;
 static const rclcpp::Logger log = rclcpp::get_logger("planner.algo");
 const bool now_braking = cmd.throttle < 0.0f;
 if (now_braking && !was_braking)
 {
   started = scanTimeOrNow();
   peak = 0.0f;
   RCLCPP_WARN(log, "BRAKE %.0f%%%s: %.2f m/s (lidar %.2f, wheels %.2f), safe %.2f m/s, clearance %.2f m",
               -cmd.throttle * 100.0f, estop ? " (e-stop)" : "", closing_brake.v, closing_brake.speed,
               wheelSpeed(), governor.v_safe, clearance);
 }
 if (now_braking)
   peak = std::max(peak, -cmd.throttle);
 else if (was_braking)
   RCLCPP_INFO(log, "brake off after %.2f s, peak %.0f%%: %.2f m/s, clearance %.2f m",
               scanTimeOrNow() - started, peak * 100.0f, closing_brake.v, clearance);
 was_braking = now_braking;
 return cmd;
}


const DebugInfo &debugInfo()
{
 return debug;
}



