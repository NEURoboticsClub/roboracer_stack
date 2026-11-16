#pragma once

#include <string>
#include "algorithm.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Scan, avoided beams, disparity edges and chosen heading, drawn in the scan's frame.
const visualization_msgs::msg::MarkerArray &buildMarkers(
  const DebugInfo &info, const std::string &frame, const rclcpp::Time &stamp);
