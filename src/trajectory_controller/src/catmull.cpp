#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_controller/types.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <vector>


namespace trajectory_controller
{

    std::vector<Point2D> catmullRomSpline( 
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
} // namespace trajectory_controller


class CatmullNode : public rclcpp::Node
{
public:
    CatmullNode() : Node("catmull_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_catmullrom", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publish();});

        RCLCPP_INFO(this->get_logger(), "Catmull-Rom Spline node started ");    
    }        
private: 
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;    

    void publish()
    {
        auto smooth_path = trajectory_controller::catmullRomSpline(waypoints_,20);

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line;

        line.header.frame_id = "odom";
        line.header.stamp = this->now();
        line.ns = "catmull_rom_path";
        line.id = 0;        
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05; // Line width
        line.color.a = 1.0; // Alpha
        line.color.r = 0.0; // Red
        line.color.g = 0.5; // Green
        line.color.b = 1.0; // Blue
        line.pose.orientation.w = 1.0; // No rotation

        for(const auto& point : smooth_path)
        {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            line.points.push_back(p);
        }

        marker_array.markers.push_back(line);
        publisher_->publish(marker_array);
        
    }



};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CatmullNode>());
  rclcpp::shutdown();
  return 0;
}