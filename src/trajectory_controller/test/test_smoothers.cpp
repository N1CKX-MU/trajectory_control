#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "trajectory_controller/types.hpp"

// ─── Copy algorithm functions here for testing ───────────────────────────────
// (same functions from your nodes, pulled into test scope)

namespace trajectory_controller
{

// Catmull-Rom
std::vector<Point2D> catmullRomSmooth(const std::vector<Point2D>& pts, int samples)
{
  std::vector<Point2D> result;
  for (size_t i = 0; i < pts.size() - 1; i++)
  {
    Point2D p0 = pts[i > 0 ? i-1 : i];
    Point2D p1 = pts[i];
    Point2D p2 = pts[i+1];
    Point2D p3 = pts[i+2 < pts.size() ? i+2 : i+1];
    for (int s = 0; s < samples; s++)
    {
      double t  = (double)s / samples;
      double t2 = t*t, t3 = t2*t;
      result.push_back({
        0.5*((2*p1.x)+(-p0.x+p2.x)*t+(2*p0.x-5*p1.x+4*p2.x-p3.x)*t2+(-p0.x+3*p1.x-3*p2.x+p3.x)*t3),
        0.5*((2*p1.y)+(-p0.y+p2.y)*t+(2*p0.y-5*p1.y+4*p2.y-p3.y)*t2+(-p0.y+3*p1.y-3*p2.y+p3.y)*t3)
      });
    }
  }
  result.push_back(pts.back());
  return result;
}

// Gradient Descent
std::vector<Point2D> gradientDescentSmooth(
  const std::vector<Point2D>& waypoints, int num_points,
  double alpha, double learning_rate, int iterations)
{
  std::vector<Point2D> path;
  for (size_t i = 0; i < waypoints.size() - 1; i++)
  {
    int n = num_points / (int)(waypoints.size() - 1);
    for (int j = 0; j < n; j++)
    {
      double t = (double)j / n;
      path.push_back({
        waypoints[i].x + t * (waypoints[i+1].x - waypoints[i].x),
        waypoints[i].y + t * (waypoints[i+1].y - waypoints[i].y)
      });
    }
  }
  path.push_back(waypoints.back());
  std::vector<Point2D> original = path;
  for (int iter = 0; iter < iterations; iter++)
    for (size_t i = 1; i < path.size() - 1; i++)
    {
      path[i].x += learning_rate * (
        alpha * (path[i-1].x - 2*path[i].x + path[i+1].x) +
        (1-alpha) * (original[i].x - path[i].x));
      path[i].y += learning_rate * (
        alpha * (path[i-1].y - 2*path[i].y + path[i+1].y) +
        (1-alpha) * (original[i].y - path[i].y));
    }
  return path;
}

// Trajectory Generator
std::vector<TrajectoryPoint> generateTrajectory(
  const std::vector<Point2D>& path, double v_max, double acceleration)
{
  std::vector<double> arc(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); i++) {
    double dx = path[i].x - path[i-1].x;
    double dy = path[i].y - path[i-1].y;
    arc[i] = arc[i-1] + std::sqrt(dx*dx + dy*dy);
  }
  double L = arc.back();
  std::vector<TrajectoryPoint> traj;
  double t_acc = 0.0;
  for (size_t i = 0; i < path.size(); i++) {
    TrajectoryPoint tp;
    tp.x = path[i].x; tp.y = path[i].y;
    tp.theta = (i < path.size()-1)
      ? std::atan2(path[i+1].y-path[i].y, path[i+1].x-path[i].x)
      : traj.back().theta;
    double v_up   = std::sqrt(2.0 * acceleration * arc[i]);
    double v_down = std::sqrt(2.0 * acceleration * (L - arc[i]));
    tp.velocity = std::max(std::min({v_max, v_up, v_down}), 0.01);
    if (i == 0) { tp.time = 0.0; }
    else {
      double ds = arc[i] - arc[i-1];
      double va = 0.5 * (traj.back().velocity + tp.velocity);
      t_acc += ds / std::max(va, 0.01);
      tp.time = t_acc;
    }
    traj.push_back(tp);
  }
  return traj;
}

} // namespace trajectory_controller

// ─── Helper ──────────────────────────────────────────────────────────────────

double dist(const trajectory_controller::Point2D& a,
            const trajectory_controller::Point2D& b)
{
  return std::sqrt(std::pow(a.x-b.x,2) + std::pow(a.y-b.y,2));
}

const std::vector<trajectory_controller::Point2D> TEST_WAYPOINTS = {
  {0.0,0.0},{1.0,0.5},{2.0,2.0},{3.0,2.5},{4.0,1.5},{5.0,0.5},{6.0,1.0}
};

// ─── Catmull-Rom Tests ────────────────────────────────────────────────────────

TEST(CatmullRomTest, OutputSizeIsCorrect)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  // 6 segments * 20 samples + 1 endpoint
  EXPECT_EQ(path.size(), (size_t)121);
}

TEST(CatmullRomTest, StartsAtFirstWaypoint)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  EXPECT_NEAR(path.front().x, TEST_WAYPOINTS.front().x, 1e-6);
  EXPECT_NEAR(path.front().y, TEST_WAYPOINTS.front().y, 1e-6);
}

TEST(CatmullRomTest, EndsAtLastWaypoint)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  EXPECT_NEAR(path.back().x, TEST_WAYPOINTS.back().x, 1e-6);
  EXPECT_NEAR(path.back().y, TEST_WAYPOINTS.back().y, 1e-6);
}

TEST(CatmullRomTest, NoLargeJumps)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  for (size_t i = 1; i < path.size(); i++)
    EXPECT_LT(dist(path[i], path[i-1]), 0.5)
      << "Large jump at index " << i;
}

TEST(CatmullRomTest, PathStaysWithinBounds)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  for (const auto& p : path) {
    EXPECT_GE(p.x, -0.5);
    EXPECT_LE(p.x,  7.0);
    EXPECT_GE(p.y, -0.5);
    EXPECT_LE(p.y,  3.5);
  }
}

TEST(CatmullRomTest, SingleSegmentWorks)
{
  std::vector<trajectory_controller::Point2D> two = {{0,0},{1,1}};
  auto path = trajectory_controller::catmullRomSmooth(two, 10);
  EXPECT_GT(path.size(), (size_t)1);
  EXPECT_NEAR(path.back().x, 1.0, 1e-6);
  EXPECT_NEAR(path.back().y, 1.0, 1e-6);
}

// ─── Gradient Descent Tests ───────────────────────────────────────────────────

TEST(GradientDescentTest, OutputSizeIsCorrect)
{
  auto path = trajectory_controller::gradientDescentSmooth(
    TEST_WAYPOINTS, 60, 0.8, 0.1, 100);
  EXPECT_EQ(path.size(), (size_t)61);
}

TEST(GradientDescentTest, EndpointsAreFixed)
{
  auto path = trajectory_controller::gradientDescentSmooth(
    TEST_WAYPOINTS, 60, 0.8, 0.1, 500);
  EXPECT_NEAR(path.front().x, TEST_WAYPOINTS.front().x, 1e-3);
  EXPECT_NEAR(path.front().y, TEST_WAYPOINTS.front().y, 1e-3);
  EXPECT_NEAR(path.back().x,  TEST_WAYPOINTS.back().x,  1e-3);
  EXPECT_NEAR(path.back().y,  TEST_WAYPOINTS.back().y,  1e-3);
}

TEST(GradientDescentTest, HighAlphaProducesSmootherPath)
{
  // Higher alpha = more smoothing = lower total curvature
  auto path_low  = trajectory_controller::gradientDescentSmooth(
    TEST_WAYPOINTS, 60, 0.3, 0.1, 300);
  auto path_high = trajectory_controller::gradientDescentSmooth(
    TEST_WAYPOINTS, 60, 0.9, 0.1, 300);

  // Measure total curvature (sum of direction changes)
  auto curvature = [](const std::vector<trajectory_controller::Point2D>& p) {
    double total = 0.0;
    for (size_t i = 1; i < p.size()-1; i++) {
      double ax = p[i].x - p[i-1].x, ay = p[i].y - p[i-1].y;
      double bx = p[i+1].x - p[i].x, by = p[i+1].y - p[i].y;
      double cross = std::abs(ax*by - ay*bx);
      total += cross;
    }
    return total;
  };

  EXPECT_LT(curvature(path_high), curvature(path_low));
}

TEST(GradientDescentTest, PathDoesNotDriftFarFromWaypoints)
{
  auto path = trajectory_controller::gradientDescentSmooth(
    TEST_WAYPOINTS, 60, 0.5, 0.1, 300);

  // Every path point should be within 2m of the nearest waypoint
  for (const auto& p : path) {
    double min_d = std::numeric_limits<double>::max();
    for (const auto& wp : TEST_WAYPOINTS)
      min_d = std::min(min_d, dist(p, wp));
    EXPECT_LT(min_d, 2.0) << "Path drifted far from waypoints at ("
                           << p.x << ", " << p.y << ")";
  }
}

// ─── Trajectory Generator Tests ───────────────────────────────────────────────

TEST(TrajectoryGeneratorTest, OutputSizeMatchesInput)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  EXPECT_EQ(traj.size(), path.size());
}

TEST(TrajectoryGeneratorTest, TimestampsAreMonotonic)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  for (size_t i = 1; i < traj.size(); i++)
    EXPECT_GE(traj[i].time, traj[i-1].time)
      << "Time went backwards at index " << i;
}

TEST(TrajectoryGeneratorTest, StartsAtTimeZero)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  EXPECT_NEAR(traj.front().time, 0.0, 1e-6);
}

TEST(TrajectoryGeneratorTest, VelocityNeverExceedsMax)
{
  double v_max = 0.18;
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, v_max, 0.05);
  for (const auto& tp : traj)
    EXPECT_LE(tp.velocity, v_max + 1e-6)
      << "Velocity exceeded max at time " << tp.time;
}

TEST(TrajectoryGeneratorTest, VelocityIsAlwaysPositive)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  for (const auto& tp : traj)
    EXPECT_GT(tp.velocity, 0.0);
}

TEST(TrajectoryGeneratorTest, StartsAndEndsSlowly)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  // Trapezoidal profile — start and end velocity should be lower than max
  EXPECT_LT(traj.front().velocity, 0.18);
  EXPECT_LT(traj.back().velocity,  0.18);
}

TEST(TrajectoryGeneratorTest, PositionsMatchInputPath)
{
  auto path = trajectory_controller::catmullRomSmooth(TEST_WAYPOINTS, 20);
  auto traj = trajectory_controller::generateTrajectory(path, 0.18, 0.05);
  for (size_t i = 0; i < path.size(); i++) {
    EXPECT_NEAR(traj[i].x, path[i].x, 1e-6);
    EXPECT_NEAR(traj[i].y, path[i].y, 1e-6);
  }
}

// ─── Waypoint Tests ───────────────────────────────────────────────────────────

TEST(WaypointTest, DefaultWaypointsAreValid)
{
  auto wps = TEST_WAYPOINTS;
  EXPECT_EQ(wps.size(), (size_t)7);
  EXPECT_NEAR(wps.front().x, 0.0, 1e-6);
  EXPECT_NEAR(wps.front().y, 0.0, 1e-6);
  EXPECT_NEAR(wps.back().x,  6.0, 1e-6);
  EXPECT_NEAR(wps.back().y,  1.0, 1e-6);
}

TEST(WaypointTest, AllWaypointsAreFinite)
{
  for (const auto& wp : TEST_WAYPOINTS) {
    EXPECT_TRUE(std::isfinite(wp.x));
    EXPECT_TRUE(std::isfinite(wp.y));
  }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}