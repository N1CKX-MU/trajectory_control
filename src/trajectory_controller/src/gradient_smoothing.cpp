#include <rclcpp/rclcpp.hpp>
#include "trajectory_controller/types.hpp"
#include "trajectory_controller/viz_utils.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>


namespace trajectory_controller {

    // Gradient descent path smoothing
    // iteratively adjusts each interior point by balancing 2 thing:
    // - Smoothness: pulls towards avg of neighbours
    // - Fitness : how well it fits the original path 

std::vector<Point2D> gradientDescentSmoothing(
        const std::vector<Point2D>& waypoints,
        int num_points,
        double alpha = 0.1,
        double learning_rate = 0.01,
        int iterations = 100)
    {

        // Build an initial path by lineary interpolating b/w waypoint pair
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

        std::vector<Point2D> original = path; // Keep a copy to compute fitness

        for(int iter = 0 ; iter < iterations; iter++)
        {
            std::vector<Point2D> new_path = path;

            for(size_t i = 1 ; i < path.size() - 1; i++)
            {
                // Smoothness gradient - second order finite difference 
                double smooth_x = path[i-1].x - 2*path[i].x + path[i+1].x;
                double smooth_y = path[i-1].y - 2*path[i].y + path[i+1].y;

                // Fitness gradient — pulls point back towards original position
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
       
        num_points_    = this->declare_parameter<int>("smoother.samples_per_segment"      , 100);
        alpha_         = this->declare_parameter<double>("smoother.gradient_alpha"        , 0.8);
        learning_rate_ = this->declare_parameter<double>("smoother.gradient_learning_rate", 0.1);
        iterations_    = this->declare_parameter<int>("smoother.gradient_iterations"      , 500);

        RCLCPP_INFO(this->get_logger(), "Gradient Smoothing node started ");

    }

private:
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int num_points_;
    double alpha_;
    double learning_rate_;
    int iterations_;

    void publish()
  {
    auto smooth = trajectory_controller::gradientDescentSmoothing(
      waypoints_, num_points_, alpha_, learning_rate_, iterations_);

    // Cyan — visually distinct from Catmull-Rom (blue), Bezier (red)
    auto arr = trajectory_controller::pathToMarkerArray(
      smooth, "gradient_path", 0, 0.0f, 1.0f, 1.0f);

    arr.markers[0].header.stamp = this->now();
    publisher_->publish(arr);
  }

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GradientSmoothing>());
  rclcpp::shutdown();
  return 0;
}