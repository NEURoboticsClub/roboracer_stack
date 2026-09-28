#include "planner_gap_follow.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

// ---------------------------------------------------------------------------
// Output target. Selected at RUNTIME by the `output_mode` parameter:
//
//   "vesc" (default)  the real car - /commands/motor/{speed,brake} and
//                     /commands/servo/position, e-stop enforced.
//   "sim"             the AutoDRIVE simulator - /autodrive/roboracer_1/
//                     {throttle,steering}_command, no e-stop console.
//
// This used to be a compile-time #define, which meant one binary could only
// ever drive one stack and a launch file could not pick: running the sim needed
// a source edit plus a rebuild, and forgetting either left the planner
// publishing VESC topics that nothing in the sim subscribes to. A parameter
// makes it a launch-file line.
//
// The mode switches BOTH the throttle and the steering output. Both stacks are
// driven from one set of decisions through outputThrottleCommand() /
// outputSteeringCommand(), so they cannot drift apart the way they did when
// each had its own command path and only the VESC side saw the steering gate.
// ---------------------------------------------------------------------------

float angleIncrement = 0.0f;
float angleMin = -2.35619f; // updated from the scan each callback
std::string scanFrame = "cloud"; // updated from the scan each callback

// Corner-detection geometry. Set once from node parameters at startup so they
// can be tuned trackside; see the comment at the clearance check for why the
// defaults are what they are.
float front_cone_half_angle = 20.0f * static_cast<float>(M_PI) / 180.0f;
float corner_distance = 2.0f;
// Half-width of the lane the car needs clear to keep driving straight. Anything
// further off the centerline than this goes past the car rather than into it.
// 0.18 m is the car's 0.15 m half-width plus a thin margin: 0.15 is a hard
// floor, since below it the corridor is narrower than the car and the check
// would wave through walls the car actually hits.
float corridor_half_width = 0.18f;

// Gap-selection geometry, also set from node parameters at startup. See the
// follow-the-gap step for what each one does and how the defaults were picked.
float min_gap_depth = 0.8f;
float min_gap_width = 0.40f;
float deep_band = 0.20f;

// What the planner wants the drivetrain to do this frame. This is the ONLY
// throttle decision either stack sees: the node turns it into a single
// normalized command (see throttleForState) which the output layer then renders
// as either an AutoDRIVE throttle or a VESC speed/brake request.
enum class DriveState
{
  Coast, // no usable forward reading: let the motor freewheel
  Brake, // closing on a wall fast: wind the speed down
  Corner, // tight clearance or hard steering: hold a reduced speed
  Cruise // clear ahead and running straight: hold the full cruise speed
};

DriveState drive_state = DriveState::Coast;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

// ---------------------------------------------------------------------------
// Reusable state. Everything below persists across callbacks so the hot path
// allocates nothing per frame: buffers keep their capacity, the marker objects
// are configured once and merely refilled, and the trig tables are cached.
// ---------------------------------------------------------------------------
namespace
{
std::vector<float> clean_ranges; // ranges with inf/nan replaced (read-only source)
std::vector<float> proc_ranges;  // clean_ranges + safety-bubble extension (written)
std::vector<char> blocked;       // 1 => index sits inside a safety bubble (avoided)

// cos/sin lookup so we never call trig per-point in the hot loop
std::vector<float> cos_table;
std::vector<float> sin_table;
size_t trig_size = 0;
float trig_min = 0.0f;
float trig_inc = 0.0f;

rclcpp::Clock g_clock(RCL_ROS_TIME);

// A fixed handful of markers (vs. one marker per lidar point in the old code).
// Each is an aggregate primitive: thousands of points ride inside a single
// message, which is dramatically cheaper to build, serialize, and render.
visualization_msgs::msg::MarkerArray g_array;
bool markers_ready = false;

enum MarkerSlot
{
  SCAN = 0,  // full scan, dim — context
  AVOID = 1, // the sections the car is actively steering away from
  DISP = 2,  // disparity edges that triggered the avoidance
  HEAD = 3,  // chosen heading
  SLOT_COUNT = 4
};

void configureMarkers()
{
  g_array.markers.resize(SLOT_COUNT);

  auto base = [](visualization_msgs::msg::Marker &m, const char *ns) {
    m.header.frame_id = "map";
    m.ns = ns;
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);
  };

  // Context: every valid return as small, faint points.
  auto &scan = g_array.markers[SCAN];
  base(scan, "scan");
  scan.type = visualization_msgs::msg::Marker::POINTS;
  scan.scale.x = 0.04;
  scan.scale.y = 0.04;
  scan.color.r = 0.25f;
  scan.color.g = 0.6f;
  scan.color.b = 0.85f;
  scan.color.a = 0.45f;

  // Avoided sections: a sphere list, colored per-point by proximity
  // (deep red = very close, orange = farther but still bubbled out).
  auto &avoid = g_array.markers[AVOID];
  base(avoid, "avoided");
  avoid.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  avoid.scale.x = 0.16;
  avoid.scale.y = 0.16;
  avoid.scale.z = 0.16;
  avoid.color.a = 1.0f; // overridden per-point

  // Disparity edges: one LINE_LIST holding every jump segment.
  auto &disp = g_array.markers[DISP];
  base(disp, "disparities");
  disp.type = visualization_msgs::msg::Marker::LINE_LIST;
  disp.scale.x = 0.05;
  disp.color.r = 1.0f;
  disp.color.g = 1.0f;
  disp.color.b = 0.0f;
  disp.color.a = 1.0f;

  // Chosen heading.
  auto &head = g_array.markers[HEAD];
  base(head, "heading");
  head.type = visualization_msgs::msg::Marker::ARROW;
  head.scale.x = 0.05;
  head.scale.y = 0.1;
  head.scale.z = 0.1;
  head.color.r = 0.0f;
  head.color.g = 1.0f;
  head.color.b = 0.0f;
  head.color.a = 1.0f;
  head.points.resize(2);

  markers_ready = true;
}

// Rebuild the cos/sin tables only when the scan geometry actually changes.
void rebuildTrig(size_t n)
{
  if (trig_size == n && trig_min == angleMin && trig_inc == angleIncrement)
    return;

  cos_table.resize(n);
  sin_table.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    float a = angleMin + static_cast<float>(i) * angleIncrement;
    cos_table[i] = std::cos(a);
    sin_table[i] = std::sin(a);
  }
  trig_size = n;
  trig_min = angleMin;
  trig_inc = angleIncrement;
}

inline geometry_msgs::msg::Point pointAt(size_t i, float dist)
{
  geometry_msgs::msg::Point p;
  p.x = dist * cos_table[i];
  p.y = dist * sin_table[i];
  p.z = 0.0;
  return p;
}

// A contiguous run of beams the car could drive into: not bubbled out by the
// disparity extender, and seeing at least min_gap_depth. Depth and width are
// what the selection ranks on.
struct Gap
{
  int i0 = -1;        // first beam in the gap
  int i1 = -1;        // last beam in the gap
  float depth = 0.0f; // deepest reading inside it
  float width = 0.0f; // metric chord between its two edges
};

std::vector<Gap> gaps; // reused across frames; clear() keeps the capacity

// Straight-line distance between the endpoints of the two edge beams: how wide
// the opening actually is in meters, which is what decides whether the car
// fits. Angular width does not - the same angle is a doorway up close and a
// barn door at range.
float chordWidth(int i0, int i1)
{
  const float a = proc_ranges[static_cast<size_t>(i0)];
  const float b = proc_ranges[static_cast<size_t>(i1)];
  const float sweep = static_cast<float>(i1 - i0) * angleIncrement;
  const float sq = a * a + b * b - 2.0f * a * b * std::cos(sweep);
  return sq > 0.0f ? std::sqrt(sq) : 0.0f;
}

// Fill `gaps` with every opening in the forward cone that sees at least
// min_depth.
void collectGaps(float min_depth, float forward_lim, size_t n)
{
  gaps.clear();
  int start = -1;
  float depth = 0.0f;

  // One past the end so a run still open at the last beam gets closed too.
  for (size_t p = 0; p <= n; p++)
  {
    bool open = false;
    if (p < n)
    {
      const float angle = angleMin + static_cast<float>(p) * angleIncrement;
      open = angle > -forward_lim && angle < forward_lim && !blocked[p] &&
             proc_ranges[p] >= min_depth;
    }

    if (open)
    {
      if (start < 0)
      {
        start = static_cast<int>(p);
        depth = proc_ranges[p];
      }
      else if (proc_ranges[p] > depth)
      {
        depth = proc_ranges[p];
      }
      continue;
    }

    if (start >= 0)
    {
      const int end = static_cast<int>(p) - 1;
      gaps.push_back({start, end, depth, chordWidth(start, end)});
      start = -1;
    }
  }
}

// Deepest gap wins, among those at least min_width across.
//
// Depth is quantized before comparing so two gaps within a scan's worth of
// noise of each other are separated by width instead. That matters at a corner:
// a wall seen at grazing incidence throws spurious disparities that can split
// one real opening into near-equal halves, and a bare > would flip between them
// frame to frame. An exact tie keeps the earlier gap, so the choice is at least
// deterministic rather than dependent on iteration order.
const Gap *pickGap(float min_width)
{
  constexpr float depth_tie = 0.10f; // meters; finer than this is noise
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
    const float qg = std::floor(g.depth / depth_tie);
    const float qb = std::floor(best->depth / depth_tie);
    if (qg > qb || (qg == qb && g.width > best->width))
      best = &g;
  }
  return best;
}
} // namespace

std_msgs::msg::Float32 findDisparities(float threshold, const std::vector<float> *ranges)
{
  const size_t n = ranges->size();
  if (!markers_ready)
    configureMarkers();
  rebuildTrig(n);

  // 1. Preprocess. The SICK encodes a no-return as 0.0 (not inf), and reports
  //    distances out to a 100 m range_max. A no-return means "nothing out to
  //    max range" => OPEN space, NOT a zero-distance wall. Treating it as a
  //    wall (the old behavior) floods the disparity extender and blocks the
  //    whole forward cone. So clamp non-finite / non-positive / over-range
  //    readings to a sane open value, and cap everything to max_scan_range.
  clean_ranges.resize(n);
  blocked.assign(n, 0);
  const float max_scan_range = 10.0f;
  for (size_t i = 0; i < n; i++)
  {
    float v = (*ranges)[i];
    if (!std::isfinite(v) || v <= 0.0f || v > max_scan_range)
      clean_ranges[i] = max_scan_range; // no return => open
    else
      clean_ranges[i] = v;
  }
  proc_ranges = clean_ranges; // working copy we extend bubbles into

  // 2. Disparity extender. Jumps are detected against the untouched
  //    clean_ranges so chained bubbles don't cascade, exactly as before.
  auto &disp = g_array.markers[DISP];
  disp.points.clear();
  // ~car half-width (0.15 m) plus margin. The extra margin over a bare
  // half-width pushes the center of the chosen gap further off the inner wall,
  // which is what stops the car from clipping the apex on a tight turn.
  const float bubble_radius = 0.22f;
  // A close obstacle subtends a huge half-angle (atan2 -> ~50 deg at 0.3 m),
  // which would smear the bubble across open space. Cap how wide a single
  // disparity may block.
  const float max_bubble_angle = 0.35f; // ~20 deg
  const int max_idx_cover = static_cast<int>(max_bubble_angle / angleIncrement);

  if (n > 1)
  {
    for (size_t i = 0; i < n - 1; i++)
    {
      float curr = clean_ranges[i];
      float next = clean_ranges[i + 1];
      float diff = curr - next;

      if (std::abs(diff) <= threshold)
        continue;

      // Disparity found: bubble out the closer side.
      float dist = std::min(curr, next);
      float angle_needed = std::atan2(bubble_radius, std::max(dist, 0.01f));
      int idx_cover = static_cast<int>(std::ceil(angle_needed / angleIncrement));
      if (idx_cover > max_idx_cover)
        idx_cover = max_idx_cover; // don't let close obstacles block the world

      // Record the jump edge as a SHORT segment (a LINE_LIST consumes points
      // in pairs). The far side is capped to near + disp_span so a wall->open
      // (no-return) disparity doesn't draw a spoke all the way out to max
      // range; we only show the step right at the obstacle.
      {
        const float disp_span = 0.6f; // max drawn length of the jump, meters
        float near_d = std::min(curr, next);
        float far_d = std::min(std::max(curr, next), near_d + disp_span);
        disp.points.push_back(pointAt(i, curr < next ? near_d : far_d));
        disp.points.push_back(pointAt(i + 1, curr < next ? far_d : near_d));
      }

      if (curr < next)
      {
        // Jump up: 'curr' is the close obstacle, extend it to the right.
        for (int k = 1; k <= idx_cover; k++)
        {
          size_t j = i + static_cast<size_t>(k);
          if (j < n)
          {
            proc_ranges[j] = std::min(proc_ranges[j], curr);
            blocked[j] = 1;
          }
        }
      }
      else
      {
        // Jump down: 'next' is the close obstacle, extend it to the left.
        for (int k = 0; k < idx_cover; k++)
        {
          if (static_cast<int>(i) - k >= 0)
          {
            size_t j = i - static_cast<size_t>(k);
            proc_ranges[j] = std::min(proc_ranges[j], next);
            blocked[j] = 1;
          }
        }
      }
    }
  }

  // 3a. Build scan/avoided geometry.
  auto &scan = g_array.markers[SCAN];
  auto &avoid = g_array.markers[AVOID];
  scan.points.clear();
  avoid.points.clear();
  avoid.colors.clear();

  for (size_t p = 0; p < n; p++)
  {
    float val = proc_ranges[p];

    if (blocked[p])
    {
      // Avoided section: closer => deeper red, farther => orange.
      avoid.points.push_back(pointAt(p, val));
      std_msgs::msg::ColorRGBA c;
      float t = std::min(val / 3.0f, 1.0f);
      c.r = 1.0f;
      c.g = 0.55f * t;
      c.b = 0.0f;
      c.a = 1.0f;
      avoid.colors.push_back(c);
    }
    else if (val > 0.0f && val < max_scan_range)
    {
      scan.points.push_back(pointAt(p, val));
    }
  }

  // 3b. Follow-the-gap. Two separate decisions live here, and at a wide corner
  //     BOTH of them used to go wrong.
  //
  //     WHICH opening. Candidates are every un-bubbled run that sees at least
  //     min_gap_depth - an ABSOLUTE floor. The old gate was 0.7 x the deepest
  //     forward reading, so an opening shallower than that could not even be
  //     considered whenever some other direction happened to read long.
  //
  //     WHERE to aim inside it. The old code aimed at the gap's angular
  //     MIDPOINT, and that is what made the car run wide at a wide corner. The
  //     corner's open outside and the exit corridor fall inside ONE gap tens of
  //     degrees across, so its midpoint points at the outer wall, while the way
  //     onward is the deep sliver at the far edge of that gap - hard up against
  //     the disparity thrown by the inner apex. Aiming at the DEEPEST part of
  //     the gap rather than its middle is what turns the car into the sliver.
  //
  //     Raycast offline against a wide 90 deg corner (apex 1 m off the
  //     centerline, outer wall 5 m out, exit corridor bearing +45 deg): the
  //     midpoint rule asked for +11 deg where this asks for +32 deg, and the
  //     heading now tracks the exit bearing to within a few degrees through the
  //     whole approach. A plain corridor still reads 0 deg, and a deeper room
  //     off to one side is still ignored, because both openings' deep ends sit
  //     where their midpoints did.
  const float forward_lim = static_cast<float>(M_PI) / 2.0f; // +-90 deg cone

  collectGaps(min_gap_depth, forward_lim, n);
  const Gap *chosen = pickGap(min_gap_width);
  if (!chosen)
  {
    // Nothing the car fits through. Take the most open direction there is
    // rather than freezing the wheels, and let the clearance check below decide
    // how hard to slow down.
    chosen = pickGap(0.0f);
  }
  if (!chosen)
  {
    // Everything in the cone reads shorter than min_gap_depth (squeezed into a
    // corner, say). Drop the depth floor so the heading still tracks the most
    // open direction, which is what the old relative threshold gave us for free.
    collectGaps(0.0f, forward_lim, n);
    chosen = pickGap(0.0f);
  }

  int target_idx = static_cast<int>(n / 2); // everything blocked: aim straight
  if (chosen)
  {
    // Aim at the middle of the deepest contiguous band inside the gap. A band,
    // not simply the deepest beam: one long return - an edge reflection, or
    // plain noise - would otherwise yank the steering. Measured offline against
    // 2 cm of scan noise, bands under ~0.15 m start chasing single beams and
    // heading jitter blows up from 0.2 deg to 25 deg, while bands over ~0.25 m
    // visibly delay turn-in. Taking the LONGEST such band rather than the first
    // is what keeps a far-off secondary lobe from stealing the aim point.
    const float band_floor = chosen->depth - deep_band;
    int band_start = -1, band_end = -1, band_len = 0, run = -1;

    for (int p = chosen->i0; p <= chosen->i1 + 1; p++)
    {
      const bool inside =
        p <= chosen->i1 && proc_ranges[static_cast<size_t>(p)] >= band_floor;
      if (inside)
      {
        if (run < 0)
          run = p;
        continue;
      }
      if (run >= 0)
      {
        const int len = p - run;
        if (len > band_len)
        {
          band_len = len;
          band_start = run;
          band_end = p - 1;
        }
        run = -1;
      }
    }

    target_idx = band_start >= 0 ? (band_start + band_end) / 2
                                 : (chosen->i0 + chosen->i1) / 2;
  }

  float target_angle = ind2angle(target_idx);
  auto &head = g_array.markers[HEAD];
  float len = proc_ranges[target_idx] > 0.0f ? proc_ranges[target_idx] : 4.0f;
  head.points[0] = geometry_msgs::msg::Point();
  head.points[1] = pointAt(static_cast<size_t>(target_idx), len);

  // Stamp and publish the whole array in one shot. Reusing marker ids with
  // ADD overwrites the previous frame, so no DELETEALL churn is needed.
  auto stamp = g_clock.now();
  for (auto &m : g_array.markers)
  {
    m.header.stamp = stamp;
    m.header.frame_id = scanFrame; // align markers with the scan's own frame
  }
  marker_pub->publish(g_array);

  // The chosen bearing in the SCAN's own frame, so REP-103 applies: positive is
  // LEFT. Each stack's output function applies its own polarity below - this
  // car's servo runs backwards from REP-103, AutoDRIVE does not - rather than
  // baking one car's wiring into the shared decision, which is what a bare
  // negation here used to do and why the sim steered into the wall it saw.
  std_msgs::msg::Float32 steering;
  steering.data = target_angle;

  // 4. Measure clearance in a cone straight ahead of the car and pick a drive
  //    state. This function no longer emits any actuator value: it only decides
  //    WHAT the car should do, and the node's output layer decides how to say it
  //    to whichever stack is built in.
  //
  //    Cone width and trigger distance come from node parameters so they can be
  //    tuned trackside without a rebuild. The old hardcoded values (+-5 deg at
  //    1.0 m) were both too narrow and too late: at 1 m that cone spans 0.17 m,
  //    narrower than the car, so a wall approached at an angle slipped past it
  //    entirely, and the measured 0.965 m turning radius needs ~1.5 m of arc to
  //    come round 90 deg - by 1 m the car is already committed.
  const float brake_activation_distance = 3.0f;
  const float fast_closing_speed = 0.8f; // m/s toward the wall
  const float slow_closing_speed = 0.2f; // m/s release threshold

  // Clearance is the distance to the nearest thing in the car's CORRIDOR, not
  // the nearest thing anywhere in the cone. A wall running alongside the car
  // does not block driving straight, but a plain min-over-the-cone treats it
  // like it does: in a narrow section the side walls enter the cone at short
  // range - a wall 0.5 m to the side is only 1.46 m out along the +-20 deg edge,
  // inside the 2.0 m corner_distance - so the car held corner speed down
  // straights with the track ahead wide open. Projecting each beam to
  // (forward, lateral) and dropping whatever clears the car's flanks fixes that
  // without going blind: anything that genuinely blocks the path subtends a
  // small angle at range, so the beams near straight ahead still see it at full
  // distance (a wall 3 m ahead is caught by everything inside +-4.8 deg).
  //
  // The geometry is taken from clean_ranges rather than proc_ranges because a
  // bubbled beam is shortened along its own ray, which drags its lateral offset
  // in toward the centerline and would manufacture an in-corridor obstruction
  // out of open space.
  float front_clearance = max_scan_range;
  bool has_front_reading = false;
  for (size_t p = 0; p < n; p++)
  {
    const float angle = angleMin + static_cast<float>(p) * angleIncrement;
    if (std::abs(angle) > front_cone_half_angle)
      continue;

    // Set on any forward beam, not just obstructing ones, so "corridor is
    // empty" reads as clear rather than as the no-data Coast case.
    has_front_reading = true;

    const float forward = clean_ranges[p] * cos_table[p];
    const float lateral = clean_ranges[p] * sin_table[p];
    if (forward <= 0.0f || std::abs(lateral) > corridor_half_width)
      continue; // passes the car down one side: not in the way

    front_clearance = std::min(front_clearance, forward);
  }

  static bool have_previous_clearance = false;
  static float previous_front_clearance = max_scan_range;
  static float filtered_closing_speed = 0.0f;
  static bool closing_brake_active = false;
  static auto previous_clearance_time = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now();

  // Positive closing speed means the measured wall distance is shrinking.
  // Filter the derivative, engage braking at a high threshold, and release it
  // at a lower threshold so scan noise cannot rapidly toggle the brake.
  if (has_front_reading)
  {
    if (have_previous_clearance)
    {
      const float elapsed_sec = std::chrono::duration<float>(
        now - previous_clearance_time).count();
      if (elapsed_sec > 0.001f && elapsed_sec < 1.0f)
      {
        const float raw_closing_speed =
          (previous_front_clearance - front_clearance) / elapsed_sec;
        const float filter_gain = 0.4f;
        filtered_closing_speed +=
          filter_gain * (raw_closing_speed - filtered_closing_speed);
      }
      else
      {
        filtered_closing_speed = 0.0f;
        closing_brake_active = false;
      }
    }

    previous_front_clearance = front_clearance;
    previous_clearance_time = now;
    have_previous_clearance = true;

    if (front_clearance <= brake_activation_distance &&
        filtered_closing_speed >= fast_closing_speed)
    {
      closing_brake_active = true;
    }
    else if (filtered_closing_speed <= slow_closing_speed ||
             front_clearance > brake_activation_distance)
    {
      closing_brake_active = false;
    }
  }
  else
  {
    have_previous_clearance = false;
    filtered_closing_speed = 0.0f;
    closing_brake_active = false;
  }

  if (!has_front_reading)
    drive_state = DriveState::Coast;
  else if (closing_brake_active)
    drive_state = DriveState::Brake;
  else if (front_clearance <= corner_distance)
    drive_state = DriveState::Corner;
  else
    drive_state = DriveState::Cruise;

  return steering;
}

class GapFollowerNode : public rclcpp::Node
{
public:
  GapFollowerNode() : Node("gap_follower_node")
  {
    // Which stack this run drives. See the block at the top of the file.
    // Defaults to "vesc" so an un-parameterized launch is still the real car,
    // which is what every existing launch file expects.
    output_mode_ = declare_parameter<std::string>("output_mode", "vesc");
    if (output_mode_ != "vesc" && output_mode_ != "sim")
    {
      RCLCPP_ERROR(
        get_logger(), "output_mode '%s' is not 'vesc' or 'sim' - falling back to 'vesc'.",
        output_mode_.c_str());
      output_mode_ = "vesc";
    }
    sim_mode_ = (output_mode_ == "sim");

    // Cruise/corner speeds are in ELECTRICAL RPM, which is what the VESC speed
    // command takes: mechanical RPM x the motor's pole pairs.
    //
    // Measured on this car: 2000 eRPM is silently ignored (the motor never
    // starts - reported speed/current/duty all stay at 0), while 5000 eRPM
    // holds cleanly at ~5020 eRPM drawing ~4.4 A. The VESC speed PID has a
    // minimum ERPM below which it will not start the motor, so keep cruise
    // above it; the threshold sits somewhere between 2000 and 5000 here.
    // 5000 is the anchor: it is the one value measured good on this car, so the
    // corner speed sits exactly there, the straight-line max reaches above it,
    // and braking drops below it but stays clear of the 2000 dead zone.
    cruise_erpm_ = declare_parameter<double>("cruise_erpm", 6000.0);
    corner_erpm_ = declare_parameter<double>("corner_erpm", 5000.0);
    // Slowing down is done by asking for a low speed rather than by dumping
    // braking current: the Brake state commands brake_erpm_ so the car eases
    // off instead of locking up. brake_current_ is now only the hard stop
    // (e-stop console and Ctrl+C shutdown).
    brake_erpm_ = declare_parameter<double>("brake_erpm", 4000.0);
    brake_current_ = declare_parameter<double>("brake_current", 4.0);

    // Target heading (radians) at which the servo reaches full lock. This is a
    // GAIN, not a physical limit: lowering it does not buy more wheel angle, it
    // just reaches the existing stop at a smaller requested heading, which
    // sharpens turn-in. 0.80 rad (~46 deg) halves the gain that 0.40 gave,
    // because at 0.40 a small heading correction already threw the wheels most
    // of the way to the stop.
    // The car's real full lock was measured at 18.6 deg of wheel angle, giving a
    // 0.965 m minimum turning radius - that is the hard limit and no gain change
    // moves it.
    max_steer_angle_ = declare_parameter<double>("max_steer_angle", 0.80);

    // Steering-gated speed. Cruise is only safe when the car is going more or
    // less straight; past this fraction of full lock it drops to corner speed
    // however much room the lidar sees ahead. Expressed as a fraction of
    // max_steer_angle_, so halving the steering gain has to be paid for here:
    // 0.30 of 0.80 rad keeps the gate at the same ~0.24 rad of requested heading
    // it fired at when the gain was 0.40 x 0.6.
    corner_lock_fraction_ = declare_parameter<double>("corner_lock_fraction", 0.30);

    // Corner-detection cone (globals: findDisparities reads them directly).
    front_cone_half_angle = static_cast<float>(
      declare_parameter<double>("front_cone_half_angle_deg", 20.0) * M_PI / 180.0);
    corner_distance = static_cast<float>(declare_parameter<double>("corner_distance", 2.0));
    // Car half-width (0.15 m) plus a thin margin. Raise it to make the car treat
    // a narrowing section as a corner sooner; lower it to hold cruise speed
    // through tighter gaps. Do not go below 0.15 - see the global's comment.
    corridor_half_width = static_cast<float>(
      declare_parameter<double>("corridor_half_width", 0.18));

    // Gap selection (globals: findDisparities reads them directly).
    //
    // How far an opening must see before it counts as somewhere to drive. This
    // is absolute, replacing a threshold relative to the deepest forward
    // reading which hid every shallower opening whenever one direction read
    // long.
    min_gap_depth = static_cast<float>(declare_parameter<double>("min_gap_depth", 0.8));
    // How wide the opening has to be in METERS for the car to fit through it.
    // The car is 0.30 m across and the disparity extender has already inset
    // both edges by bubble_radius, so this is that 0.30 plus a thin margin.
    // Raise it to make the car refuse tighter gaps and look elsewhere.
    min_gap_width = static_cast<float>(declare_parameter<double>("min_gap_width", 0.40));
    // How far back from the chosen gap's deepest reading still counts as "the
    // deep end" that the car aims at. Lower turns in earlier but starts chasing
    // individual beams; higher is smoother but drifts back toward the old
    // aim-at-the-midpoint behavior. See the measurements at the aim-point code.
    deep_band = static_cast<float>(declare_parameter<double>("deep_band", 0.20));

    invert_steering_ = declare_parameter<bool>("invert_steering", false);
    // Servo range in the VESC's normalized 0..1 units. Defaults match
    // servo_min/servo_max in vesc_driver's vesc_config.yaml so the driver never
    // has to clip what we publish.
    servo_center_ = declare_parameter<double>("servo_center", 0.5);
    servo_min_ = declare_parameter<double>("servo_min", 0.117);
    servo_max_ = declare_parameter<double>("servo_max", 0.895);

    // Simulator scaling. The normalized command is stack-agnostic, so these
    // convert it into what AutoDRIVE's open-loop throttle expects. Only read in
    // output_mode:=sim.
    sim_cruise_throttle_ = declare_parameter<double>("sim_cruise_throttle", 0.1);
    // Below this the open-loop ESC/sim simply ignores the request, so anything
    // smaller has to be pulsed rather than sent directly. The simulator itself
    // has no such dead zone, so a sim launch should set this to 0.0 and get a
    // smooth corner throttle instead of a 2 Hz stutter.
    sim_throttle_floor_ = declare_parameter<double>("sim_throttle_floor", 0.1);
    sim_brake_throttle_ = declare_parameter<double>("sim_brake_throttle", -0.05);
    // AutoDRIVE's steering_command is NORMALIZED [-1, 1], not radians, and
    // positive is LEFT - same sense as the LaserScan the heading came from (the
    // bridge turns a positive command into a positive front-wheel yaw). So the
    // heading maps straight through with +1, using max_steer_angle_ as the
    // gain, which also keeps |command| equal to the lock_fraction the corner
    // gate in throttleForState() computes.
    //
    // Set this to -1.0 if the car in the sim mirrors every turn - that is the
    // one thing here that cannot be checked without watching it drive.
    sim_steer_sign_ = declare_parameter<double>("sim_steer_sign", 1.0);

    auto topic_callback = [this](sensor_msgs::msg::LaserScan::UniquePtr scan) -> void
    {
      angleIncrement = scan->angle_increment;
      angleMin = scan->angle_min;
      scanFrame = scan->header.frame_id;

      // findDisparities only decides; it updates the global drive_state and
      // returns the steering angle. Everything actuator-facing happens below.
      const std_msgs::msg::Float32 steering = findDisparities(0.2f, &scan->ranges);

      // Held stopped (real car only) => the stop has already been published.
      if (holdStopped())
        return;

      // Publishing straight from the scan callback means the scan rate is also
      // the command rate, so a stalled lidar lets the VESC's own command
      // timeout stop the motor.
      outputSteeringCommand(steering.data);
      outputThrottleCommand(throttleForState(drive_state, steering.data));
    };

    // The scan topic is /scan on the car. The AutoDRIVE bridge publishes its
    // lidar on /autodrive/roboracer_1/lidar instead, so the sim launch file
    // remaps this rather than the node knowing about either name. Default QoS
    // at depth 10 is reliable/volatile, which the bridge's reliable depth-1
    // publisher matches.
    subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, topic_callback);
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 1);

    // Only the stack actually being driven gets publishers, so `ros2 topic
    // list` shows what this run commands instead of advertising both and
    // moving neither.
    if (sim_mode_)
    {
      steering_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/steering_command", 1);
      throttle_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/throttle_command", 1);
    }
    else
    {
      speed_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 1);
      brake_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/brake", 1);
      servo_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 1);
    }

    // How long the estop heartbeat may go missing before we brake. The console
    // sends it at 100 Hz, so this is ~30 missed beats.
    estop_timeout_sec_ = declare_parameter<double>("estop_timeout_sec", 0.3);

    // No e-stop console is part of the sim stack, so subscribing there would
    // only log stops the sim build does not act on. See holdStopped().
    if (!sim_mode_)
    {
      // Latched to match the estop console's publisher, so starting the planner
      // second still picks up an already-engaged stop instead of driving off.
      rclcpp::QoS estop_qos(1);
      estop_qos.transient_local().reliable();
      estop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/estop", estop_qos,
        [this](std_msgs::msg::Bool::UniquePtr msg)
        {
          if (msg->data != estop_engaged_)
            RCLCPP_WARN(
              get_logger(), "E-STOP %s", msg->data ? "ENGAGED - braking" : "released - driving");
          estop_engaged_ = msg->data;
          last_estop_msg_ = std::chrono::steady_clock::now();
          estop_seen_ = true;
        });
    }

    if (sim_mode_)
      RCLCPP_INFO(
        get_logger(),
        "SIM output: throttle %.2f cruise / %.2f brake on "
        "/autodrive/roboracer_1/throttle_command, steering on "
        "/autodrive/roboracer_1/steering_command (normalized, full lock at "
        "%.2f rad, sign %+.0f). NO E-STOP - the car drives as soon as scans arrive.",
        sim_cruise_throttle_, sim_brake_throttle_, max_steer_angle_, sim_steer_sign_);
    else
      RCLCPP_INFO(
        get_logger(),
        "VESC output: cruise %.0f eRPM, corner %.0f eRPM, brake %.0f eRPM "
        "(hard stop %.1f A). "
        "STOPPED BY DEFAULT - run 'ros2 run planner_node estop' and press r to drive.",
        cruise_erpm_, corner_erpm_, brake_erpm_, brake_current_);
  }

private:
  // Map a REP-103 steering angle in radians (positive = LEFT) onto the VESC's
  // 0..1 servo output. Each side is scaled against its own limit so an
  // off-center trim still reaches full lock in both directions.
  double steeringToServo(float angle_rad) const
  {
    if (!std::isfinite(angle_rad))
      return servo_center_;

    // This car's servo runs OPPOSITE to REP-103: 0.117 is full LEFT and 0.895
    // full RIGHT, so a positive (left) angle has to land BELOW servo_center_.
    // That fixed wiring polarity lives here; invert_steering_ stays as the
    // trackside override on top of it, for a car wired the other way round.
    constexpr double servo_polarity = -1.0;
    const double angle = servo_polarity *
      (invert_steering_ ? -static_cast<double>(angle_rad)
                        : static_cast<double>(angle_rad));
    const double normalized = std::clamp(angle / max_steer_angle_, -1.0, 1.0);
    const double servo = normalized >= 0.0
      ? servo_center_ + normalized * (servo_max_ - servo_center_)
      : servo_center_ + normalized * (servo_center_ - servo_min_);
    return std::clamp(servo, servo_min_, servo_max_);
  }

  // The car is stopped unless the estop console is alive AND has released it.
  // Both halves matter: estop_engaged_ starts true so nothing moves before the
  // console appears, and the heartbeat check means a closed or killed console
  // stops the car instead of leaving it running on a stale value.
  bool stopRequested()
  {
    if (estop_engaged_)
      return true;

    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_estop_msg_).count();

    if (!estop_seen_ || age_sec > estop_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        estop_seen_
          ? "E-STOP heartbeat lost (%.2fs) - braking. Restart: ros2 run planner_node estop"
          : "Waiting for e-stop console - braking. Run 'ros2 run planner_node estop', press r.",
        age_sec);
      return true;
    }

    return false;
  }

  // Hold the car stopped if the e-stop says so, publishing the stop itself.
  // Returns true when the caller should emit nothing further this frame.
  //
  // Only the real car has an e-stop console, so output_mode:=sim never holds -
  // it would otherwise sit motionless forever waiting on a heartbeat from a
  // console that is not part of the sim stack at all.
  bool holdStopped()
  {
    if (sim_mode_)
      return false;

    // The e-stop console also brakes the VESC directly at 100 Hz, so this is the
    // cooperative half of the stop: hold the brake and straighten the wheels
    // instead of steering blind.
    if (!stopRequested())
      return false;

    std_msgs::msg::Float64 msg;
    msg.data = servo_center_;
    servo_pub->publish(msg);
    msg.data = brake_current_;
    brake_pub->publish(msg);
    return true;
  }

  // Collapse a drive state into ONE normalized throttle, shared by both stacks:
  //
  //     +1.0  full cruise
  //      0.0  coast / freewheel
  //     -1.0  brake (wind the speed down)
  //
  // The steering gate lives here so it applies everywhere. The clearance logic
  // in findDisparities only looks straight ahead, so it happily holds cruise
  // into a gap that is wide open but too tight to actually arc into at speed;
  // downgrading Cruise to Corner on steering alone is what stops the car running
  // wide. Only Cruise is downgraded - Brake and Coast are already slower.
  float throttleForState(DriveState state, float steering_angle) const
  {
    if (state == DriveState::Cruise && max_steer_angle_ > 0.0)
    {
      const double lock_fraction =
        std::abs(static_cast<double>(steering_angle)) / max_steer_angle_;
      if (lock_fraction >= corner_lock_fraction_)
        state = DriveState::Corner;
    }

    switch (state)
    {
      case DriveState::Cruise:
        return 1.0f;
      case DriveState::Corner:
        // Corner speed as a fraction of cruise, so the round trip through the
        // VESC branch below lands exactly back on corner_erpm_.
        return cruise_erpm_ > 0.0
          ? static_cast<float>(corner_erpm_ / cruise_erpm_) : 0.0f;
      case DriveState::Brake:
        return -1.0f;
      case DriveState::Coast:
      default:
        return 0.0f;
    }
  }

  // The single point where a throttle decision becomes an actuator command.
  // The `output_mode` parameter (see the top of this file) picks the stack.
  void outputThrottleCommand(float throttle)
  {
    if (sim_mode_)
    {
      std_msgs::msg::Float32 msg;

      if (throttle < 0.0f)
      {
        msg.data = static_cast<float>(sim_brake_throttle_);
      }
      else if (throttle == 0.0f)
      {
        msg.data = 0.0f;
      }
      else
      {
        const float scaled = throttle * static_cast<float>(sim_cruise_throttle_);
        const float floor_v = static_cast<float>(sim_throttle_floor_);

        if (scaled >= floor_v || floor_v <= 0.0f)
        {
          msg.data = scaled;
        }
        else
        {
          // An open-loop ESC ignores anything below the floor outright, so a
          // low request has to be pulsed AT the floor with the duty cycle that
          // averages out to what was asked for. The simulator has no such dead
          // zone: set sim_throttle_floor to 0.0 there and this never runs.
          const long long period_ms = 500;
          const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          const float duty = scaled / floor_v;
          const long long on_ms = static_cast<long long>(duty * period_ms);
          msg.data = (elapsed_ms % period_ms) < on_ms ? floor_v : 0.0f;
        }
      }

      throttle_pub->publish(msg);
      return;
    }

    // The VESC acts on whichever command arrived last, so exactly one of the
    // speed/brake topics gets a message per frame.
    std_msgs::msg::Float64 msg;

    if (throttle > 0.0f)
    {
      msg.data = static_cast<double>(throttle) * cruise_erpm_;
      speed_pub->publish(msg);
    }
    else if (throttle < 0.0f)
    {
      // Braking is a speed request, not a current request: the PID winds the
      // motor down to brake_erpm_ instead of slamming the brakes on.
      msg.data = brake_erpm_;
      speed_pub->publish(msg);
    }
    else
    {
      // Zero braking current lets the motor freewheel.
      msg.data = 0.0;
      brake_pub->publish(msg);
    }
  }

  // Steering's counterpart to the above, switched by the same flag so one build
  // never drives half of each stack.
  void outputSteeringCommand(float steering_angle)
  {
    if (sim_mode_)
    {
      // AutoDRIVE wants a NORMALIZED command in [-1, 1], not the radians the
      // planner works in - publishing radians straight through asked for full
      // lock at anything past ~57 deg of heading and saturated for most of a
      // corner. Same normalization the servo path uses, so the corner gate in
      // throttleForState() and this command agree on what "full lock" means.
      std_msgs::msg::Float32 msg;
      msg.data = std::isfinite(steering_angle) && max_steer_angle_ > 0.0
        ? static_cast<float>(std::clamp(
            sim_steer_sign_ * static_cast<double>(steering_angle) / max_steer_angle_,
            -1.0, 1.0))
        : 0.0f;
      steering_pub->publish(msg);
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = steeringToServo(steering_angle);
    servo_pub->publish(msg);
  }

public:
  // Brake burst used on shutdown (Ctrl+C) so the car stops immediately instead
  // of coasting until the VESC's own command timeout expires.
  void brakeOnShutdown()
  {
    if (!brake_pub)
      return;

    std_msgs::msg::Float64 brake_msg;
    brake_msg.data = brake_current_;
    for (int i = 0; i < 5; i++)
      brake_pub->publish(brake_msg);
  }

private:

  std::string output_mode_ = "vesc";
  bool sim_mode_ = false;
  double cruise_erpm_ = 6000.0;
  double corner_erpm_ = 5000.0;
  double brake_erpm_ = 4000.0;
  double brake_current_ = 4.0;
  double max_steer_angle_ = 0.80;
  double corner_lock_fraction_ = 0.30;
  bool invert_steering_ = false;
  double servo_center_ = 0.5;
  double servo_min_ = 0.117;
  double servo_max_ = 0.895;
  double sim_cruise_throttle_ = 0.1;
  double sim_throttle_floor_ = 0.1;
  double sim_brake_throttle_ = -0.05;
  double sim_steer_sign_ = 1.0;
  // Stopped by default: nothing moves until the estop console releases it.
  // Only consulted in output_mode:=vesc - the sim has no e-stop console.
  bool estop_engaged_ = true;
  bool estop_seen_ = false;
  double estop_timeout_sec_ = 0.3;
  std::chrono::steady_clock::time_point last_estop_msg_{};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GapFollowerNode>();

  // Ctrl+C path: brake before the context tears down, so the car does not coast
  // for the duration of the VESC's command timeout.
  rclcpp::on_shutdown([node]() { node->brakeOnShutdown(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

float ind2angle(int index)
{
  return (index * angleIncrement) + angleMin;
}
