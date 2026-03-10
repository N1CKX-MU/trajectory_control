#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>



typedef struct{
    double x;
    double y;
}Point2D;


class Waypoints : public rclcpp::Node
{
public:
    Waypoints() : Node("waypoints_node")
    {
    //     waypoints_ = {
    //   {0.0, 0.0},
    //   {1.0, 0.5},
    //   {2.0, 1.5},
    //   {3.0, 1.0},
    //   {4.0, 2.0},
    //   {5.0, 1.0},
    //   {6.0, 0.0}
    // };

     waypoints_ ={  // upside down V shaped trajectory 
        {0.0, 0.0},  //← start
        {1.0, 0.5},
        {2.0, 2.0},
        {3.0, 2.5},
        {4.0, 1.5},
        {5.0, 0.5},
        {6.0, 1.0}  //← end
    };

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this](){publishMarker(); });

    RCLCPP_INFO(this->get_logger(), "Waypoints node started with %zu waypoints",
      waypoints_.size());

    }

    

private:
    std::vector<Point2D> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


    void publishMarker()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0 ; i < waypoints_.size();i++){
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = waypoints_[i].x;
            marker.pose.position.y = waypoints_[i].y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0; // Alpha
            marker.color.r = 1.0; // Red
            marker.color.g = 0.0; // Green
            marker.color.b = 0.0; // Blue

            marker_array.markers.push_back(marker);
        }


        visualization_msgs::msg::Marker line;
        line.header.frame_id = "odom";
        line.header.stamp = this->now();
        line.ns = "waypoints_path";
        line.id = 100; 
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.02;
        line.color.a = 1.0; // Alpha
        line.color.r = 0.0; // Red  
        line.color.g = 1.0; // Green
        line.color.b = 0.0; // Blue
        line.pose.orientation.w = 1.0; // No rotation


        for(const auto& wp : waypoints_){
            geometry_msgs::msg::Point p;
            p.x = wp.x;
            p.y = wp.y;
            p.z = 0.0;
            line.points.push_back(p);
        }

        marker_array.markers.push_back(line);

        


        publisher_->publish(marker_array);
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Waypoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}