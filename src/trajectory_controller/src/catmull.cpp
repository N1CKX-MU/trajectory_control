#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_controller/types.hpp"
#include "trajectory_controller/viz_utils.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <vector>


namespace trajectory_controller
{

// Catmull-Rom spline interpolation between waypoints.
// For each segment [p1,p2], we use the neighbouring points p0 and p3
// to compute a smooth curve that passes through every waypoint 

    std::vector<Point2D> catmullRomSpline( 
        const std::vector<Point2D>& pts,  
        int samples_per_segment = 10)
        {
            std::vector<Point2D> result;
            
            for( size_t i = 0 ; i < pts.size() -1;  i ++)
            {
                // Clamp neighbours at the endpoints instead of wrapping
                Point2D p0 = pts[i > 0 ? i - 1 : i];
                Point2D p1 = pts[i];
                Point2D p2 = pts[i + 1];
                Point2D p3 = pts[i + 2 < pts.size() ? i + 2 : i + 1];

                for(int s = 0 ; s < samples_per_segment;s++){

                    double t = double(s) / samples_per_segment;
                    double t2 = t * t;
                    double t3 = t2 * t;

                    // Standard Catmull-Rom matrix with alpha = 0.5
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
} // namespace trajectory_controller


class CatmullNode : public rclcpp::Node
{
public:
    CatmullNode() : Node("catmull_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_catmullrom", 10);

        samples_ = this->declare_parameter<int>("smoother.samples_per_segment",20);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publish();});

        RCLCPP_INFO(this->get_logger(), "Catmull-Rom Spline node started ");    
    }        
private: 
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_; 
    int samples_;   

    void publish()
    {
        auto smooth = trajectory_controller::catmullRomSpline(waypoints_, samples_);

        // Light blue — visually distinct from Bezier (red) and gradient (orange)
        auto arr = trajectory_controller::pathToMarkerArray(
        smooth, "catmull_rom_path", 0, 0.0f, 0.5f, 1.0f);
           
        arr.markers[0].header.stamp = this->now();
        publisher_->publish(arr);
    }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CatmullNode>());
  rclcpp::shutdown();
  return 0;
}