#include <rclcpp/rclcpp.hpp>
#include "trajectory_controller/types.hpp"
#include "trajectory_controller/viz_utils.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>


namespace trajectory_controller
{

    // Helper to safely index outside the waypoint array - clamps first and last
    // instead of wrapping, keeps curve better at end points
    static Point2D getPoint(const std::vector<Point2D>& pts, int i)
    {
        if(i < 0) return pts.front();
        if(i >= static_cast<int>(pts.size())) return pts.back();
        return pts[i];

    }

    // Cubic Bezier spline with automatically computed control points
    // for each segment [p0,p3] control points p[p1,p2] are derived
    // from the neighbouring waypointsusing tension of 0.25
    std::vector<Point2D> bezierCurve(const std::vector<Point2D>& pts, int samples_per_segment = 20)
    {
        std::vector<Point2D> result;

        for( size_t i = 0 ; i < pts.size() - 1; i++)
        {
            Point2D p0 = pts[i];
            Point2D p3 = pts[i + 1];

            Point2D prev = getPoint(pts, (int)i - 1);
            Point2D next = getPoint(pts, (int)i + 2);
            
            // Control Points pulled towards the neighbouring segment direction
            Point2D p1 = {
                p0.x + 0.25 * (p3.x - prev.x),
                p0.y + 0.25 * (p3.y - prev.y)
                };

            Point2D p2 = {
                p3.x - 0.25 * (next.x - p0.x),
                p3.y - 0.25 * (next.y - p0.y)
                };
                
            for(int s = 0 ; s < samples_per_segment; s++)
            {
                double t = double(s) / samples_per_segment;
                double t2 = t * t;
                double t3 = t2 * t;
                
                double mt  = 1.0 - t;
                double mt2 = mt * mt;
                double mt3 = mt2 * mt;

                // Standard Cubic Bezier Formula

                double x = mt3 * p0.x +
                            3 * mt2 * t  * p1.x +
                            3 * mt  * t2 * p2.x +
                            t3            * p3.x;

                double y = mt3 * p0.y +
                            3 * mt2 * t  * p1.y +
                            3 * mt  * t2 * p2.y +
                            t3            * p3.y;

                result.push_back({x, y});
                }
            }   
        return result;
    }
} // namespace trjectory_controller

class BezierNode : public rclcpp::Node
{
public:
    BezierNode() : Node("bezier_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);
        samples_   = this->declare_parameter<int>("smoother.samples_per_segment", 20);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bezier_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publish(); });

        RCLCPP_INFO(this->get_logger(), "Bezier node started ");
    }

private:
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int samples_;

    void publish()
  {
    auto smooth = trajectory_controller::bezierCurve(waypoints_, samples_);

    // Red — visually distinct from Catmull-Rom (blue) and gradient (orange)
    auto arr = trajectory_controller::pathToMarkerArray(
      smooth, "bezier_path", 0, 1.0f, 0.0f, 0.0f);

    arr.markers[0].header.stamp = this->now();
    publisher_->publish(arr);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BezierNode>());
  rclcpp::shutdown();
  return 0;
}