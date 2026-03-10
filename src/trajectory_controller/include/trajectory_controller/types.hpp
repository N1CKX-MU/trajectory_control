#pragma once
#include <vector>

namespace trajectory_controller
{

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct TrajectoryPoint {
    double x;
    double y;
    double theta;
    double velocity;
    double time;
};

struct DetectedObstacle {
  double x{0.0};
  double y{0.0};
  double radius{0.0};
};

// Waypoints used by all nodes — single source of truth
inline std::vector<Point2D> getDefaultWaypoints()
{
  return {
    {0.0, 0.0},
    {1.0, 0.5},
    {2.0, 2.0},
    {3.0, 2.5},
    {4.0, 1.5},
    {5.0, 0.5},
    {6.0, 1.0}
  };
}

} // namespace trajectory_controller
