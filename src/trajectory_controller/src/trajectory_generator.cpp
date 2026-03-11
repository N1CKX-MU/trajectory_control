#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include "trajectory_controller/types.hpp"


typedef struct{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double velocity{0.0};
    double time{0.0};
}TrajectoryPoint;


namespace trajectory_controller
{

// Catmull Rom Spline - suplicated here so this is self contained 
// and doesnt depend oon catmull.cpp

    static std::vector<Point2D> catmullRomSpline( 
        const std::vector<Point2D>& pts,  
        int samples_per_segment = 10)
    {
            std::vector<Point2D> result;
            
            for( size_t i = 0 ; i < pts.size() -1;  i ++)
            {
                Point2D p0 = pts[i > 0 ? i - 1 : i];
                Point2D p1 = pts[i];
                Point2D p2 = pts[i + 1];
                Point2D p3 = pts[i + 2 < pts.size() ? i + 2 : i + 1];


                for(int s = 0 ; s < samples_per_segment;s++){

                    double t = double(s) / samples_per_segment;
                    double t2 = t * t;
                    double t3 = t2 * t;

                    double x = 0.5 * ((2 * p1.x) + 
                                       (-p0.x + p2.x) * t + 
                                       (2*p0.x - 5*p1.x + 4*p2.x - p3.x) * t2 + 
                                       (-p0.x + 3*p1.x - 3*p2.x + p3.x) * t3);

                    double y = 0.5 * ((2 * p1.y) + 
                                       (-p0.y + p2.y) * t + 
                                       (2*p0.y - 5*p1.y + 4*p2.y - p3.y) * t2 + 
                                       (-p0.y + 3*p1.y - 3*p2.y + p3.y) * t3);

                    result.push_back({x, y});

                }

            }
            

            result.push_back(pts.back()); // Ensure the last point is included

            return result;

    }

// Generates a time-parametrized trajectory from a smooth path
// Velocity is assigned using trapezoidal profile
// it ramps up and then cruises and then slows down to rest 
// Each point gets a timestamp based on the average velocity over each segment

    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<Point2D>& path,
        double max_velocity,
        double acceleration)
    {

        // Compute cumulative arc length along the path
        std::vector<double> arc_length(path.size(),0.0);
        for(size_t i = 1 ; i < path.size();i++){
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;

            arc_length[i] = arc_length[i-1] + std::sqrt(dx*dx + dy*dy);

        }

        double total_length = arc_length.back();

        std::vector<TrajectoryPoint> trajectory;
        double time_acc = 0.0;

        for(size_t i = 0 ; i < path.size();i++){

            TrajectoryPoint tp;
            tp.x = path[i].x;
            tp.y = path[i].y;

            // Heading points towards the next point, last point keeps prev heading

            if(i < path.size() - 1){
                tp.theta = std::atan2(
                    path[i+1].y - path[i].y,
                    path[i+1].x - path[i].x
                );
            }else if(!trajectory.empty()){
                tp.theta = trajectory.back().theta;
            }else{
                tp.theta = 0.0;
            }

            // Trapezoidal profile 
            
            double s = arc_length[i];
            double v_ramp_up = std::sqrt(2.0 * acceleration * s);
            double v_ramp_down = std::sqrt(2.0 * acceleration * (total_length - s));

            tp.velocity = std::min({max_velocity,v_ramp_up,v_ramp_down});
            tp.velocity = std::max(tp.velocity,0.01);

            if ( i == 0 ){
                tp.time = 0.0;
            }else {
                double ds = arc_length[i] - arc_length[i-1];
                double v_avg = trajectory.back().velocity + tp.velocity;
                v_avg = std::max(v_avg,0.01);
                time_acc += ds / v_avg;
                tp.time = time_acc;
            }
            trajectory.push_back(tp);
        }
        return trajectory;
    }
    
}// namespace trajectory_controller


class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_points", 10);
        vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/trajectory_velocities", 10); 

        samples_per_segment_ =
            this->declare_parameter<int>("smoother.samples_per_segment", 20);
        max_velocity_ =
            this->declare_parameter<double>("trajectory.max_velocity", 0.18);
        acceleration_ =
            this->declare_parameter<double>("trajectory.acceleration", 0.05);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publish(); });

        RCLCPP_INFO(this->get_logger(),
        "Params loaded: samples=%d vel=%.2f accel=%.2f",
        samples_per_segment_,
        max_velocity_,
        acceleration_);
    }
private: 

    std::vector<trajectory_controller::Point2D>                    waypoints_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

    int samples_per_segment_;
    double max_velocity_;
    double acceleration_;

    void publish()
    {
        auto smooth = trajectory_controller::catmullRomSpline(
            waypoints_,
            samples_per_segment_);

        auto trajectory = trajectory_controller::generateTrajectory(
            smooth,
            max_velocity_,
            acceleration_);

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = this->now();

        std_msgs::msg::Float64MultiArray vel_msg;

        for( const auto& tp : trajectory)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = tp.x;
            pose.pose.position.y = tp.y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, tp.theta);
            pose.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(pose);
            vel_msg.data.push_back(tp.velocity);
        }
        publisher_->publish(path_msg);
        vel_pub_->publish(vel_msg);

        RCLCPP_INFO(this->get_logger(),"Published trajectory: %zu points, total time: %.2f s", trajectory.size(), trajectory.back().time);


    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}