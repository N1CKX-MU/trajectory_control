#include <rclcpp/rclcpp.hpp>
#include <trajectory_controller/types.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>


namespace trajectory_controller
{
    Point2D getPoint(const std::vector<Point2D>& pts, int i)
    {
        if(i < 0) return pts.front();
        if(i >= static_cast<int>(pts.size())) return pts.back();
        return pts[i];

    }

    std::vector<Point2D> bezierCurve(const std::vector<Point2D>& pts, int samples_per_segment = 20)
    {
        std::vector<Point2D> result;

        for( size_t i = 0 ; i < pts.size() - 1; i++)
        {
            Point2D p0 = pts[i];
            Point2D p3 = pts[i + 1];

            Point2D prev = getPoint(pts, (int)i - 1);
            Point2D next = getPoint(pts, (int)i + 2);
            

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

}


class BezierNode : public rclcpp::Node
{
public:
    BezierNode() : Node("bezier_node")
    {
        waypoints_ = trajectory_controller::getWaypoints(this);
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bezier_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this](){publishMarker(); });

        RCLCPP_INFO(this->get_logger(), "Bezier node started ");
    }

private:
    std::vector<trajectory_controller::Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishMarker()
    {
        auto bezier_points = trajectory_controller::bezierCurve(waypoints_);
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "odom";
        line.header.stamp = this->now();
        line.ns = "bezier_path";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05; // Line width
        line.color.a = 1.0; // Alpha
        line.color.r = 1.0; // Red
        line.color.g = 1.0; // Green
        line.color.b = 0.0; // Blue
        line.pose.orientation.w = 1.0;

        for(const auto& pt : bezier_points)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
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
  rclcpp::spin(std::make_shared<BezierNode>());
  rclcpp::shutdown();
  return 0;
}