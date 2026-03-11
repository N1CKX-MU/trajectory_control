#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "trajectory_controller/types.hpp"
#include "trajectory_controller/msg/segment.hpp"
#include <std_msgs/msg/int32.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <memory>

#include <visualization_msgs/msg/marker_array.hpp>



namespace trajectory_controller
{
// Extract yaw (heading) from an odometry message
    double getYaw(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            return tf2::getYaw(q);
    }
// Find the closest trajectory point to the robot's current position
size_t findClosestPoint(
        const std::vector<TrajectoryPoint>& trajectory,
        double rx, double ry)
    {
        size_t closest = 0; 
        double min_dist = std::numeric_limits<double>::max();

        for(size_t i = 0 ; i < trajectory.size();i++)
        {
            double dx = trajectory[i].x - rx;
            double dy = trajectory[i].y - ry;
            double d = std::sqrt(dx*dx + dy*dy);

            if(d < min_dist)
            {
                min_dist = d;
                closest = i;
            }
        }
        return closest;

    }

// Find the first trajectory point that is at least distance L ahead of the robot
size_t findLookaheadPoint(
        const std::vector<TrajectoryPoint>& trajectory,
        double rx, double ry,
        double L,
        size_t start_idx
    )
    {
        for(size_t i = start_idx; i < trajectory.size();i++)
        {
            double dx = trajectory[i].x - rx;
            double dy = trajectory[i].y - ry;
            double d = std::sqrt(dx*dx + dy*dy);

            if(d >= L)
            {
                return i;
            }
        }
        return trajectory.size() - 1;
    }
}



class ControllerObstacleNode : public rclcpp::Node
{


public: 
    ControllerObstacleNode() : Node("controller_node")
    {
    k_lookahead_    = this->declare_parameter("controller.lookahead_gain",  1.5);
    min_lookahead_  = this->declare_parameter("controller.min_lookahead",   0.15);
    max_lookahead_  = this->declare_parameter("controller.max_lookahead",   0.60);
    v_max_          = this->declare_parameter("controller.max_linear_vel",  0.18);
    v_max_acc_       = this->declare_parameter("trajectory.acceleration"  ,  0.15);
    goal_tolerance_ = this->declare_parameter("controller.goal_tolerance",  0.10);

        waypoints_ = trajectory_controller::getWaypoints(this);

        buildTrajectory();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",10);

        error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "tracking_error",10);

        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/actual_path", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){odomCallback(msg); });


        path_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/path_avoiding", 10,
      [this](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        pathCallback(msg); });

        // Fallback — if no avoided path received within 3s use Catmull-Rom directly
        fallback_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() {
            if (!path_received_) {
              RCLCPP_INFO(this->get_logger(),
                "No avoider running — using Catmull-Rom path directly");
              path_received_ = true;
            }
            fallback_timer_->cancel();
          });

        waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/waypoint_reached", 10);

        segment_sub_ = this->create_subscription<trajectory_controller::msg::Segment>(
        "/current_segment", 10,
        [this](trajectory_controller::msg::Segment::SharedPtr msg) {
            current_segment_index_ = msg->segment_index;
            segment_goal_x_        = msg->goal_x;
            segment_goal_y_        = msg->goal_y;
            segment_received_      = true;
            goal_reached_          = false;
            path_received_         = false;  // wait for avoider to replan
            closest_idx_           = 0;
            RCLCPP_INFO(this->get_logger(),
                "New segment %d received, waiting for avoided path...",
                current_segment_index_);
        });


        RCLCPP_INFO(this->get_logger(),
            "Controller node started, Trajectory has %zu points", trajectory_.size());
    }

private: 

    double k_lookahead_;
    double min_lookahead_;
    double max_lookahead_;
    double v_max_;
    double goal_tolerance_;
    double v_max_acc_;

    //State
    std::vector<trajectory_controller::Point2D> waypoints_;
    std::vector<trajectory_controller::TrajectoryPoint> trajectory_;
    bool goal_reached_{false};
    size_t closest_idx_{0};


    //ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     error_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  actual_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr                                       fallback_timer_;
    bool path_received_{false};

    rclcpp::Subscription<trajectory_controller::msg::Segment>::SharedPtr segment_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_reached_pub_;
    int current_segment_index_{0};
    double segment_goal_x_{0.0};
    double segment_goal_y_{0.0};
    bool segment_received_{false};

    nav_msgs::msg::Path actual_path_msg_;

    //  Generate a smooth trajectory from waypoints using Catmull-Rom and trapezoidal velocity
    void buildTrajectory()
    {
        std::vector<trajectory_controller::Point2D> smooth;
        const auto& pts = waypoints_;

        for(size_t i = 0; i < pts.size() - 1;i++)
        {
            trajectory_controller::Point2D p0 = pts[i > 0 ? i - 1 : i];
            trajectory_controller::Point2D p1 = pts[i];
            trajectory_controller::Point2D p2 = pts[i + 1];
            trajectory_controller::Point2D p3 = pts[i + 2 < pts.size() ? i + 2 : i + 1];

            for(int s = 0; s< 20 ; s++)
            {
                double t = (double)s / 20;
                double t2 = t * t, t3 = t2 * t;
                smooth.push_back({
                    0.5*((2*p1.x)+(-p0.x+p2.x)*t+(2*p0.x-5*p1.x+4*p2.x-p3.x)*t2+(-p0.x+3*p1.x-3*p2.x+p3.x)*t3),
                    0.5*((2*p1.y)+(-p0.y+p2.y)*t+(2*p0.y-5*p1.y+4*p2.y-p3.y)*t2+(-p0.y+3*p1.y-3*p2.y+p3.y)*t3)
                });
            }
        }
        smooth.push_back(pts.back());

        std::vector<double> arc(smooth.size(),0.0);
        for(size_t i = 1; i < smooth.size();i++)
        {
            double dx = smooth[i].x - smooth[i-1].x;
            double dy = smooth[i].y - smooth[i-1].y;
            arc[i] = arc[i-1] + std::sqrt(dx*dx + dy*dy);
        }

        double L = arc.back();
        double t_acc = 0.0;

        for(size_t i = 0 ; i < smooth.size();i++)
        {
            trajectory_controller::TrajectoryPoint tp;
            tp.x = smooth[i].x;
            tp.y = smooth[i].y;

            if(i < smooth.size() - 1){
                tp.theta = std::atan2(
                    smooth[i+1].y - smooth[i].y,
                    smooth[i+1].x - smooth[i].x
                );
            }else{
                tp.theta = trajectory_.empty() ? 0.0 : trajectory_.back().theta; // Keep last orientation
            }

            double v_up = std::sqrt(2.0 * v_max_acc_ * arc[i]);
            double v_down = std::sqrt(2.0 * v_max_acc_ * (L - arc[i]));
            tp.velocity = std::max(std::min({v_max_, v_up, v_down}),0.01);

            if(i == 0){
                tp.time = 0.0;
            }else{
                double ds = arc[i] - arc[i-1];
                double v_avg = (trajectory_.back().velocity + tp.velocity) / 2.0;
                v_avg = std::max(v_avg,0.01);
                t_acc += ds / v_avg;
                tp.time = t_acc;
            }

            trajectory_.push_back(tp);
        }
    }
    
    // Callback to receive avoided path from planner
    void pathCallback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto& marker : msg->markers)
    {
        if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP) continue;

        std::vector<trajectory_controller::Point2D> path_points;
        for (const auto& pt : marker.points) {
            path_points.push_back({pt.x, pt.y});
        }

        if (path_points.size() < 2) continue;

        trajectory_.clear();
        closest_idx_ = 0;
        goal_reached_ = false;

        // Arc lengths
        std::vector<double> arc(path_points.size(), 0.0);
        for (size_t i = 1; i < path_points.size(); i++) {
            double dx = path_points[i].x - path_points[i-1].x;
            double dy = path_points[i].y - path_points[i-1].y;
            arc[i] = arc[i-1] + std::sqrt(dx*dx + dy*dy);
        }
        double L = arc.back();

        double t_acc = 0.0;
        for (size_t i = 0; i < path_points.size(); i++)
        {
            trajectory_controller::TrajectoryPoint tp;
            tp.x = path_points[i].x;
            tp.y = path_points[i].y;
            tp.theta = (i < path_points.size()-1)
                ? std::atan2(path_points[i+1].y - path_points[i].y,
                             path_points[i+1].x - path_points[i].x)
                : (trajectory_.empty() ? 0.0 : trajectory_.back().theta);

            
            double v_up   = std::sqrt(2.0 * v_max_acc_ * arc[i]);
            double v_down = std::sqrt(2.0 * v_max_acc_ * (L - arc[i]));
            double v_trap = std::min({v_max_, v_up, v_down});
            // Floor at 60% of max so short segments don't crawl
            tp.velocity   = std::max(v_trap, v_max_ * 0.6);

            if (i == 0) {
                tp.time = 0.0;
            } else {
                double ds = arc[i] - arc[i-1];
                double va = 0.5 * (trajectory_.back().velocity + tp.velocity);
                t_acc += ds / va;
                tp.time = t_acc;
            }
            trajectory_.push_back(tp);
        }

        path_received_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Received new avoided path with %zu points", trajectory_.size());
        }
    }

    // Compute control commands and publish

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(goal_reached_){
            publishZeroVelocity();
            return;
        }

        if (!path_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(),
                *this->get_clock(), 2000, "Waiting for avoided path...");
            return;
        }

        // Robot State
        double rx = msg->pose.pose.position.x;
        double ry = msg->pose.pose.position.y;
        double ryaw = trajectory_controller::getYaw(msg);
        double rv = msg->twist.twist.linear.x;

        // check is reached 
        // Check segment goal reached (not full trajectory end)
        double dist_to_segment_goal = std::sqrt(
          std::pow(segment_goal_x_ - rx, 2) +
          std::pow(segment_goal_y_ - ry, 2));

        

        if (dist_to_segment_goal < goal_tolerance_ * 4.0) {
          RCLCPP_INFO(this->get_logger(),
            "Waypoint %d reached!", current_segment_index_);
          std_msgs::msg::Int32 reached_msg;
          reached_msg.data = current_segment_index_;
          waypoint_reached_pub_->publish(reached_msg);
          goal_reached_ = true;
          publishZeroVelocity();
          return;
        }

        // find closest points and dont search backwards
        closest_idx_ = trajectory_controller::findClosestPoint(trajectory_, rx, ry);

        double L = std::clamp(k_lookahead_ * std::abs(rv), min_lookahead_, max_lookahead_);


        size_t la_idx = trajectory_controller::findLookaheadPoint(
            trajectory_ , rx, ry, L , closest_idx_);

        auto& la = trajectory_[la_idx];


        // Angle to lookahead point in robot frame 
        double angle_to_la = std::atan2(la.y - ry , la.x - rx);
        double alpha       = angle_to_la - ryaw;


        // Normalize alpha to [-pi] to [pi]
        while(alpha > M_PI) alpha -= 2.0* M_PI;
        while(alpha < - M_PI) alpha += 2.0 * M_PI;

        // Pure pursuit curvature 
        double chord   = std::sqrt(std::pow(la.x - rx,2) + std::pow(la.y - ry,2));
        double curvature = (chord > 1e-6) ? 2.0 * std::sin(alpha) / chord : 0.0;

        // Velocity Commands
        double v_cmd = trajectory_[closest_idx_].velocity;

        // Slow down when approaching segment goal
        if (dist_to_segment_goal < 0.1) {
          v_cmd *= (dist_to_segment_goal / 0.5);
          v_cmd = std::max(v_cmd, 0.18);
         }

        double omega_cmd = v_cmd * curvature;


        // Safety clamp 
        v_cmd = std::clamp(v_cmd, 0.0 , v_max_);
        omega_cmd = std::clamp(omega_cmd,-2.84,2.84);

        //Publish cmd_vel

        geometry_msgs::msg::Twist twist;
        twist.linear.x = v_cmd;
        twist.angular.z = omega_cmd;
        cmd_vel_pub_->publish(twist);

        //Publish tracking error 
        // Cross track error -> perpendicular distance from robot to closest point
        auto& cp = trajectory_[closest_idx_];
        double cte = -(std::sin(cp.theta) * (rx - cp.x) - 
                        std::cos(cp.theta) * (ry - cp.y));
        
        double heading_error = cp.theta -ryaw;
        while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

        std_msgs::msg::Float64MultiArray err_msg;
        err_msg.data = {cte, heading_error, v_cmd, omega_cmd};
        error_pub_->publish(err_msg);


        // Append current position to actual path
        actual_path_msg_.header.frame_id = "odom";
        actual_path_msg_.header.stamp    = this->now();

        geometry_msgs::msg::PoseStamped ps;
        ps.header = actual_path_msg_.header;
        ps.pose.position.x = rx;
        ps.pose.position.y = ry;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = msg->pose.pose.orientation;
        actual_path_msg_.poses.push_back(ps);

        actual_path_pub_->publish(actual_path_msg_);
    }
    // Publish zero velocity to stop the robot
    void publishZeroVelocity()
    {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerObstacleNode>());
    rclcpp::shutdown();
    return 0;
}


