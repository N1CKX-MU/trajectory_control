#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "trajectory_controller/types.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <memory>

#include <visualization_msgs/msg/marker_array.hpp>



namespace trajectory_controller
{


//Extracts yaw from an odometry message quaternion

double getYaw(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            return tf2::getYaw(q);
    }

// Finds the trajectory point closest to the robot's current position
// Used to anchor the lookahead search and compute cross-track error
// Pure pursuit technique used to control the robot
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

// Finds the first trajectory point atleast distance L ahead of the robot.
// Searching forward from start_idx avoids jumping back to already-passed points. 
size_t findLookaheadPoint(
        const std::vector<TrajectoryPoint>& trajectory,
        double rx, double ry,
        double L,
        size_t start_idx
    )
    {
        for(size_t i = start_idx; i < trajectory.size();i++) // starts from start index
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
} // namespace trajectory_controller



class ControllerNode : public rclcpp::Node
{


public: 
    ControllerNode() : Node("controller_node")
    {
    k_lookahead_    = this->declare_parameter("controller.lookahead_gain",  1.5);
    min_lookahead_  = this->declare_parameter("controller.min_lookahead",   0.15);
    max_lookahead_  = this->declare_parameter("controller.max_lookahead",   0.60);
    v_max_          = this->declare_parameter("controller.max_linear_vel",  0.18);
    goal_tolerance_ = this->declare_parameter("controller.goal_tolerance",  0.10);

    waypoints_ = trajectory_controller::getWaypoints(this);
    buildTrajectory();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tracking_error",10);
    actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){odomCallback(msg); });

    // Listen for an avoided path from obstacles from obstacle_avoider_node
    // if None arrives the fallback timer releases the controller to use the catmull Rom trajectory
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


        RCLCPP_INFO(this->get_logger(),
            "Controller node started, Trajectory has %zu points", trajectory_.size());
    }

private: 

    double k_lookahead_;
    double min_lookahead_;
    double max_lookahead_;
    double v_max_;
    double goal_tolerance_;

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

    nav_msgs::msg::Path actual_path_msg_;


    // Builds the internal Catmull-Rom trajectory when no avoider is running
    // Velocity is assigned with trapezoidal profile (ramp up , ramp down , cruise)

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

            double v_up = std::sqrt(2.0 * 0.05 * arc[i]);
            double v_down = std::sqrt(2.0 * 0.05 * (L - arc[i]));
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


    // Rebuilds the trajectory when the obstacle avoider publishes a new safe path 
    // this replaces the internal Catmull-Rom path entirely if it recieves a new path

    void pathCallback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // Extract points from the LINE_STRIP marker
    for (const auto& marker : msg->markers)
    {
      if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP) continue;

      std::vector<trajectory_controller::Point2D> path_points;
      for (const auto& pt : marker.points) {
        path_points.push_back({pt.x, pt.y});
      }

      if (path_points.size() < 2) continue;

      // Rebuild trajectory from new path
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
        double v_up   = std::sqrt(2.0 * 0.05 * arc[i]);
        double v_down = std::sqrt(2.0 * 0.05 * (L - arc[i]));
        tp.velocity   = std::max(std::min({v_max_, v_up, v_down}), 0.01);
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
        auto& goal = trajectory_.back();
        double dist_to_goal = std::sqrt(std::pow(goal.x - rx,2) + std::pow(goal.y - ry,2)); // general euclidean distance formula
        
        if( dist_to_goal < goal_tolerance_){
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            goal_reached_ = true;
            publishZeroVelocity();
            return;
        }

        // find closest points and dont search backwards
        closest_idx_ = trajectory_controller::findClosestPoint(trajectory_, rx, ry);

        // Lookahead distance scaled with speed
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

    // To stop the bot once it reaches goal
    void publishZeroVelocity()
    {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}


