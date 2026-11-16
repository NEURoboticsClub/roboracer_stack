#include "markers.hpp"
#include <algorithm>
#include <cmath>

namespace
{
using visualization_msgs::msg::Marker;

enum Slot { kScan, kAvoided, kDisparities, kHeading, kSlotCount };

constexpr float kMaxEdgeLength = 0.6f; // m; keeps edges into open space from drawing to max range

visualization_msgs::msg::MarkerArray markers; // reused every scan

Marker makeMarker(const char *ns, int type, double scale, float r, float g, float b, float a)
{
  Marker m;
  m.ns = ns;
  m.id = 0;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  return m;
}

void initMarkers()
{
  markers.markers.resize(kSlotCount);
  markers.markers[kScan] = makeMarker("scan", Marker::POINTS, 0.04, 0.25f, 0.6f, 0.85f, 0.45f);
  markers.markers[kAvoided] = makeMarker("avoided", Marker::SPHERE_LIST, 0.16, 1.0f, 0.0f, 0.0f, 1.0f);
  markers.markers[kDisparities] = makeMarker("disparities", Marker::LINE_LIST, 0.05, 1.0f, 1.0f, 0.0f, 1.0f);
  markers.markers[kHeading] = makeMarker("heading", Marker::ARROW, 0.1, 0.0f, 1.0f, 0.0f, 1.0f);
  markers.markers[kHeading].scale.x = 0.05; // shaft diameter
  markers.markers[kHeading].points.resize(2);
}

geometry_msgs::msg::Point pointAt(int index, float dist)
{
  const float ang = ind2angle(index);
  geometry_msgs::msg::Point p;
  p.x = dist * std::cos(ang);
  p.y = dist * std::sin(ang);
  return p;
}

// Deep red up close, fading to orange by 3 m.
std_msgs::msg::ColorRGBA proximityColor(float dist)
{
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0f;
  c.g = 0.55f * std::min(dist / 3.0f, 1.0f);
  c.b = 0.0f;
  c.a = 1.0f;
  return c;
}
} // namespace

const visualization_msgs::msg::MarkerArray &buildMarkers(
  const DebugInfo &info, const std::string &frame, const rclcpp::Time &stamp)
{
  if (markers.markers.empty())
    initMarkers();

  auto &scan = markers.markers[kScan];
  auto &avoided = markers.markers[kAvoided];
  auto &disparities = markers.markers[kDisparities];
  auto &heading = markers.markers[kHeading];
  scan.points.clear();
  avoided.points.clear();
  avoided.colors.clear();
  disparities.points.clear();

  const int n = static_cast<int>(info.ranges.size());
  for (int i = 0; i < n; i++)
  {
    const float r = info.ranges[i];
    if (info.blocked[i])
    {
      avoided.points.push_back(pointAt(i, r));
      avoided.colors.push_back(proximityColor(r));
    }
    else if (r > 0.0f && r < kMaxRange)
    {
      scan.points.push_back(pointAt(i, r));
    }
  }

  for (const auto &e : info.edges)
  {
    const float cap = std::min(e.a, e.b) + kMaxEdgeLength;
    disparities.points.push_back(pointAt(e.index, std::min(e.a, cap)));
    disparities.points.push_back(pointAt(e.index + 1, std::min(e.b, cap)));
  }

  heading.points[0] = geometry_msgs::msg::Point();
  if (n > 0)
  {
    const float inc = ind2angle(1) - ind2angle(0);
    const int idx = std::clamp(
      static_cast<int>(std::lround((info.heading - ind2angle(0)) / inc)), 0, n - 1);
    heading.points[1] = pointAt(idx, info.ranges[idx]);
  }

  for (auto &m : markers.markers)
  {
    m.header.frame_id = frame;
    m.header.stamp = stamp;
  }
  return markers;
}
