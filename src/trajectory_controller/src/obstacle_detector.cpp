#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "trajectory_controller/types.hpp"
#include <vector>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>


namespace trajectory_controller
{

    struct Cluster{
        std::vector<Point2D> points;
        Point2D centre;
        double radius{0.0};
    };

    std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<Point2D> points;

        for(size_t i = 0 ; i < scan->ranges.size(); i++)
        {
            double r = scan->ranges[i];
            if(std::isinf(r) || std::isnan(r)) continue;
            if( r < scan->range_min || r > scan->range_max) continue;

            double angle = scan->angle_min + i * scan->angle_increment;
            points.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
        return points;
    }


    std::vector<Cluster> clusterPoints(
        const std::vector<Point2D>& points,
        double cluster_threshold = 0.2)
    {
        std::vector<Cluster> clusters;
        std::vector<bool> visited(points.size(),false);

        for(size_t i = 0 ; i < points.size();i++){
            if(visited[i]) continue;

            Cluster cluster;
            cluster.points.push_back(points[i]);
            visited[i] = true;

            //Find all points within threshold and add to cluster

            for(size_t j = i + 1;j < points.size(); j++){
                if(visited[j]) continue;

                double dx = points[j].x - points[i].x;
                double dy = points[j].y - points[i].y;
                double d = std::sqrt(dx*dx + dy*dy);

                if(d < cluster_threshold){
                    cluster.points.push_back(points[j]);
                    visited[j] = true;
                }
            }
            clusters.push_back(cluster);
        }
        
        return clusters;
    }


    // Compute centre and radius of each cluster
void computeClusterProperties(std::vector<Cluster>& clusters)
{
  for (auto& c : clusters)
  {
    // Centre = mean of all points
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : c.points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    c.centre.x = sum_x / c.points.size();
    c.centre.y = sum_y / c.points.size();

    // Radius = max distance from centre to any point
    double max_r = 0.0;
    for (const auto& p : c.points) {
      double dx = p.x - c.centre.x;
      double dy = p.y - c.centre.y;
      max_r = std::max(max_r, std::sqrt(dx*dx + dy*dy));
    }
    c.radius = max_r + 0.05; // small buffer
  }
}

} //namespace trajectory_controller


class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode() : Node("obstacle_detector_node")
  {
    // TF buffer for transforming from base_scan to odom frame
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_obstacles", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scanCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Obstacle detector node started");
  }

private:
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;

  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert scan to points in robot frame
    auto points = trajectory_controller::scanToPoints(msg);

    // Cluster points
    auto clusters = trajectory_controller::clusterPoints(points, 0.25);

    // Discard clusters with too few points (noise)
    clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
        [](const trajectory_controller::Cluster& c) {
          return c.points.size() < 9;// Adjust threshold as needed
        }),
      clusters.end());

    // Compute centre and radius
    trajectory_controller::computeClusterProperties(clusters);

    // Transform cluster centres from base_scan to odom frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        "odom", "base_scan",
        tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
      return;
    }

    // Publish as markers
    visualization_msgs::msg::MarkerArray arr;

    // First clear old markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    for (size_t i = 0; i < clusters.size(); i++)
    {
      // Transform centre point to odom frame
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header.frame_id = "base_scan";
      pt_in.header.stamp    = msg->header.stamp;
      pt_in.point.x         = clusters[i].centre.x;
      pt_in.point.y         = clusters[i].centre.y;
      pt_in.point.z         = 0.0;

      tf2::doTransform(pt_in, pt_out, transform);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp    = this->now();
      m.ns              = "obstacles";
      m.id              = i + 1;
      m.type            = visualization_msgs::msg::Marker::CYLINDER;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pt_out.point.x;
      m.pose.position.y = pt_out.point.y;
      m.pose.position.z = 0.25;
      m.pose.orientation.w = 1.0;
      m.scale.x         = clusters[i].radius * 2.0;
      m.scale.y         = clusters[i].radius * 2.0;
      m.scale.z         = 0.5;
      m.color.r         = 1.0;
      m.color.g         = 0.0;
      m.color.b         = 0.0;
      m.color.a         = 0.7;

      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);

   RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Detected %zu obstacles", clusters.size());

   }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

