#include <rclcpp/rclcpp.hpp>
#include "trajectory_controller/types.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>


namespace trajectory_controller {

    std::vector<Point2D> gradientDescentSmoothing(
        const std::vector<Point2D>& waypoints,
        int num_points,
        double alpha = 0.1,
        double learning_rate = 0.01,
        int iterations = 100)
    {
        std::vector<Point2D> path;

        for(size_t i = 0 ; i < waypoints.size() - 1; i++)
        {
            int points_in_segment = num_points / (waypoints.size() - 1);
            for(int j = 0 ; j < points_in_segment; j++)
            {
                double t = double(j) / points_in_segment;
                path.push_back({
        waypoints[i].x + t * (waypoints[i+1].x - waypoints[i].x),
        waypoints[i].y + t * (waypoints[i+1].y - waypoints[i].y)
                });
            }
        }
        path.push_back(waypoints.back());

        std::vector<Point2D> original = path;

        for(int iter = 0 ; iter < iterations; iter++)
        {
            std::vector<Point2D> new_path = path;

            for(size_t i = 1 ; i < path.size() - 1; i++)
            {
                double smooth_x = path[i-1].x - 2*path[i].x + path[i+1].x;
                double smooth_y = path[i-1].y - 2*path[i].y + path[i+1].y;

                // Distance gradient — pulls point back towards original position
                double dist_x = original[i].x - path[i].x;
                double dist_y = original[i].y - path[i].y;

                // Update point — move in direction that reduces cost
                path[i].x += learning_rate * (alpha * smooth_x + (1.0 - alpha) * dist_x);
                path[i].y += learning_rate * (alpha * smooth_y + (1.0 - alpha) * dist_y);
            }

        }
        return path;
    }

} // namespace trajectory_controller



class GradientSmoothing : public rclcpp::Node
{

public: 
    GradientSmoothing() : Node("gradient_smoothing_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_gradient", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publish(); });

        RCLCPP_INFO(this->get_logger(), "Gradient Smoothing node started ");
      

    }

private:
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish(){
        auto smooth = trajectory_controller::gradientDescentSmoothing(waypoints_, 100, 0.9, 0.01, 500); // Function call 

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "odom";
        line.header.stamp = this->now();
        line.ns = "gradient_path";
        line.id = 100;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05;
        line.color.a = 1.0; // Alpha
        line.color.r = 0.0; // Red
        line.color.g = 1.0; // Green
        line.color.b = 1.0; // Blue
        line.pose.orientation.w = 1.0;

        for (const auto& p : smooth) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      line.points.push_back(pt);
        }

        marker_array.markers.push_back(line);
        publisher_->publish(marker_array);
    }

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GradientSmoothing>());
  rclcpp::shutdown();
  return 0;
}