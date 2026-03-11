#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_controller/types.hpp"
#include <cmath>
#include <vector>
#include "trajectory_controller/msg/segment.hpp"
#include <std_msgs/msg/bool.hpp>

namespace trajectory_controller
{
// Catmull-Rom — same as before
std::vector<Point2D> catmullRomSmooth(
  const std::vector<Point2D>& pts, int samples)
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

std::vector<Point2D> avoidObstacles(
    const std::vector<Point2D>& path,
    const std::vector<DetectedObstacle>& obstacles,
    double alpha = 0.5,
    double learning_rate = 0.1,
    int iterations = 50,
    double safe_margin = 0.5 // Minimum distance to maintain from obstacles
    )
    {

        std::vector<Point2D> result = path;
        std::vector<Point2D> original = path;

        for( int iter = 0; iter < iterations; iter++)
        {
            for(size_t i = 1 ; i < result.size() - 1; i++)
            {
                //Smoothness gradient
                double smooth_x = result[i-1].x - 2*result[i].x + result[i+1].x;
                double smooth_y = result[i-1].y - 2*result[i].y + result[i+1].y;
                
                //Distance from original path

                double dist_x = original[i].x - result[i].x;
                double dist_y = original[i].y - result[i].y;

                //Obstacle repulsion

                double repulse_x = 0.0;
                double repulse_y = 0.0;

                for(const auto& obs : obstacles)
                {
                    double dx   = result[i].x - obs.x;
                    double dy   = result[i].y - obs.y;
                    double d    = std::sqrt(dx*dx + dy*dy);
                    double safe_r = obs.radius + safe_margin;

                    if(d < safe_r && d > 1e-6)
                    {
                        // Determine which side to go around — use cross product
                        // of path direction vs obstacle direction
                        // This is computed once from original path so it never flips
                        double path_dx = 0.0, path_dy = 0.0;
                        if (i < original.size() - 1) {
                            path_dx = original[i+1].x - original[i].x;
                            path_dy = original[i+1].y - original[i].y;
                        }
                        // Cross product z-component: positive = obstacle on right, negative = left
                        double cross = path_dx * (obs.y - original[i].y) 
                                     - path_dy * (obs.x - original[i].x);

                        // Always push perpendicular to path in consistent direction
                        double perp_x = -path_dy;
                        double perp_y =  path_dx;
                        double perp_len = std::sqrt(perp_x*perp_x + perp_y*perp_y);
                        if (perp_len > 1e-6) {
                            perp_x /= perp_len;
                            perp_y /= perp_len;
                        }

                        // If obstacle is on the right (cross > 0), push left (negative perp)
                        // If obstacle is on the left (cross < 0), push right (positive perp)
                        double sign = (cross > 0) ? -1.0 : 1.0;
                        double strength = (safe_r - d);

                        repulse_x += sign * strength * perp_x;
                        repulse_y += sign * strength * perp_y;
                    }
                }
                //Combine gradients
                result[i].x += learning_rate * (alpha * smooth_x + (1.0 - alpha) * dist_x + 2.0 * repulse_x);
                result[i].y += learning_rate * (alpha * smooth_y + (1.0 - alpha) * dist_y + 2.0 * repulse_y);                
            }
        }
        return result;
    }



} // namespace trajectory_controller


class ObstacleAvoiderNode : public rclcpp::Node
{
public:
  ObstacleAvoiderNode() : Node("obstacle_avoider_node")
  {
    waypoints_ = trajectory_controller::getWaypoints(this);

    // Publisher — the safe avoided path
    path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_avoiding", 10);

    // Subscriber — detected obstacles from obstacle_detector_node
    obstacle_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/detected_obstacles", 10,
      [this](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        obstacleCallback(msg); });

    // Publish at 1Hz
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish(); });

    segment_sub_ = this->create_subscription<trajectory_controller::msg::Segment>(
      "/current_segment", 10,
      [this](trajectory_controller::msg::Segment::SharedPtr msg) {
        current_segment_ = *msg;
        segment_received_ = true;
        path_computed_ = false;  // recompute path for new segment
        RCLCPP_INFO(this->get_logger(),
          "Received segment %d: (%.2f,%.2f) -> (%.2f,%.2f)",
          msg->segment_index,
          msg->start_x, msg->start_y,
          msg->goal_x, msg->goal_y);
      });

    
    samples_ = this->declare_parameter<int>(
        "smoother.samples_per_segment", 20);

    alpha_ = this->declare_parameter<double>(
        "obstacle_avoider.alpha", 0.6);

    learning_rate_ = this->declare_parameter<double>(
        "obstacle_avoider.learning_rate", 0.05);

    iterations_ = this->declare_parameter<int>(
        "obstacle_avoider.iterations", 800);

    safe_margin_ = this->declare_parameter<double>(
        "obstacle_avoider.safe_margin", 0.45);

    RCLCPP_INFO(this->get_logger(),
        "Obstacle Avoider params: alpha=%.2f lr=%.3f iter=%d margin=%.2f",
        alpha_, learning_rate_, iterations_, safe_margin_);
  }

private:
    std::vector<trajectory_controller::Point2D>         waypoints_;
    std::vector<trajectory_controller::DetectedObstacle> obstacles_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool path_computed_{false};
    trajectory_controller::msg::Segment current_segment_;
    bool segment_received_{false};
    rclcpp::Subscription<trajectory_controller::msg::Segment>::SharedPtr segment_sub_;

    double alpha_;
    double learning_rate_;
    int iterations_;
    double safe_margin_;
    int samples_;

  void obstacleCallback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    obstacles_.clear();
    for (const auto& marker : msg->markers)
    {
      // Skip the DELETEALL marker
      if (marker.action == visualization_msgs::msg::Marker::DELETEALL) continue;

      trajectory_controller::DetectedObstacle obs;
      obs.x      = marker.pose.position.x;
      obs.y      = marker.pose.position.y;
      obs.radius = marker.scale.x / 2.0;
      obstacles_.push_back(obs);
    }
  }

  void publish()
  {


    if (obstacles_.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(),
        *this->get_clock(), 2000, "Waiting for obstacle detections...");
      return;
    }

    if (!segment_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(),
        *this->get_clock(), 2000, "Waiting for segment...");
      return;
    }

    if (path_computed_) {
      return;
    }

    // Build two-point path for this segment
    // Build segment path with intermediate points for smoother avoidance
    std::vector<trajectory_controller::Point2D> segment_path;
    int n_interp = 5;
    for (int i = 0; i <= n_interp; i++) {
      double t = static_cast<double>(i) / n_interp;
      segment_path.push_back({
        current_segment_.start_x + t * (current_segment_.goal_x - current_segment_.start_x),
        current_segment_.start_y + t * (current_segment_.goal_y - current_segment_.start_y)
      });
    }

    auto smooth = segment_path;  // already interpolated, no need for catmull on 2 points

    // Deform around detected obstacles
    auto safe = trajectory_controller::avoidObstacles(
        smooth,
        obstacles_,
        alpha_,
        learning_rate_,
        iterations_,
        safe_margin_);

    // Publish as purple line
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker line;

    line.header.frame_id    = "odom";
    line.header.stamp       = this->now();
    line.ns                 = "avoiding";
    line.id                 = 0;
    line.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action             = visualization_msgs::msg::Marker::ADD;
    line.scale.x            = 0.04;
    line.color.r            = 0.5;
    line.color.g            = 0.0;
    line.color.b            = 1.0;
    line.color.a            = 1.0;
    line.pose.orientation.w = 1.0;

    for (const auto& p : safe) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      line.points.push_back(pt);
    }

    arr.markers.push_back(line);
    path_pub_->publish(arr);
    path_computed_ = true;
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoiderNode>());
  rclcpp::shutdown();
  return 0;
}