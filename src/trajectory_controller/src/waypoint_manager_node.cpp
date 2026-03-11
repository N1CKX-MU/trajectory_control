#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "trajectory_controller/types.hpp"
#include "trajectory_controller/msg/segment.hpp"
#include <vector>


// Owns the global waypoint list and drives the segment-by-segment replanning loop
// Instead of handing all the waypoints to theavoider at once , this node publishes
// one segment at a time - start to next waypoint iteratively
// this ensures robot always plans over short enough segments that all obstacles are within
class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode() : Node("waypoint_manager_node")
  {
    waypoints_       = trajectory_controller::getWaypoints(this);
    current_segment_ = 0;

    segment_pub_ = this->create_publisher<trajectory_controller::msg::Segment>(
      "/current_segment", 10);

    done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/mission_complete", 10);

    // Controller publishes the indec of the segment it just completed
    waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/waypoint_reached", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        waypointReachedCallback(msg);
      });

    // Publish first segment after short delay to let other nodes start
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        publishCurrentSegment();
        timer_->cancel();
        timer_.reset();
      });

    RCLCPP_INFO(this->get_logger(),
      "Waypoint manager started with %zu waypoints", waypoints_.size());
  }

private:
  std::vector<trajectory_controller::Point2D> waypoints_;
  int current_segment_{0};

  rclcpp::Publisher<trajectory_controller::msg::Segment>::SharedPtr  segment_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  done_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              waypoint_reached_sub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;

  void publishCurrentSegment()
  {
    int total = static_cast<int>(waypoints_.size()) - 1;

    if (current_segment_ >= total) {
      RCLCPP_INFO(this->get_logger(), "Mission complete — all waypoints reached");
      std_msgs::msg::Bool done;
      done.data = true;
      done_pub_->publish(done);
      return;
    }

    auto & start = waypoints_[current_segment_];
    auto & goal  = waypoints_[current_segment_ + 1];

    trajectory_controller::msg::Segment msg;
    msg.header.frame_id    = "odom";
    msg.header.stamp       = this->now();
    msg.start_x            = start.x;
    msg.start_y            = start.y;
    msg.goal_x             = goal.x;
    msg.goal_y             = goal.y;
    msg.segment_index      = current_segment_;
    msg.total_segments     = total;

    segment_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "Segment %d/%d: (%.2f, %.2f) -> (%.2f, %.2f)",
      current_segment_ + 1, total,
      start.x, start.y, goal.x, goal.y);
  }

  void waypointReachedCallback(std_msgs::msg::Int32::SharedPtr msg)
  {
    //Ignore stale confirmations from prev segments
    if (msg->data != current_segment_) {
      return;
    }

    current_segment_++;
    RCLCPP_INFO(this->get_logger(),
      "Waypoint %d reached, advancing to segment %d",
      msg->data, current_segment_);


    // Brief pause before publishing the next segment, giving time to clear prev path
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        publishCurrentSegment();
        timer_->cancel();
        timer_.reset();
      });
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManagerNode>());
  rclcpp::shutdown();
  return 0;
}