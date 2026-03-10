#pragma once
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace trajectory_controller
{

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct TrajectoryPoint {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double velocity{0.0};
  double time{0.0};
};

struct DetectedObstacle {
  double x{0.0};
  double y{0.0};
  double radius{0.0};
};

// Call this from any node to get waypoints
// Reads from params if available, falls back to defaults
inline std::vector<Point2D> getWaypoints(rclcpp::Node* node)
{
  // Default waypoints as flat list [x0,y0, x1,y1, ...]
  const std::vector<double> defaults = {
    0.0, 0.0,
    1.0, 0.5,
    2.0, 2.0,
    3.0, 2.5,
    4.0, 1.5,
    5.0, 0.5,
    6.0, 1.0
  };

  // Declare parameter if not already declared
  if (!node->has_parameter("waypoints")) {
    node->declare_parameter("waypoints", defaults);
  }

  auto flat = node->get_parameter("waypoints").as_double_array();

  std::vector<Point2D> waypoints;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    waypoints.push_back({flat[i], flat[i+1]});
  }
  return waypoints;
}

} // namespace trajectory_controller