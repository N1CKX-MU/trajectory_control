#pragma once

#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_controller/types.hpp"

namespace trajectory_controller
{

// Converts a 2D path into a LINE_STRIP marker ready for RViz.
// Keeping this as a shared utility avoids duplicating the same
// 20-line marker setup block across every smoother node.
inline visualization_msgs::msg::Marker pathToLineMarker(
  const std::vector<Point2D> & path,
  const std::string & ns,
  int id,
  float r, float g, float b,
  float width = 0.05f)
{
  visualization_msgs::msg::Marker line;
  line.header.frame_id    = "odom";
  line.ns                 = ns;
  line.id                 = id;
  line.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action             = visualization_msgs::msg::Marker::ADD;
  line.scale.x            = width;
  line.color.r            = r;
  line.color.g            = g;
  line.color.b            = b;
  line.color.a            = 1.0f;
  line.pose.orientation.w = 1.0;

  for (const auto & pt : path) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
    line.points.push_back(p);
  }

  return line;
}

// Wraps a single marker into a MarkerArray — saves a few lines at every call site.
inline visualization_msgs::msg::MarkerArray toMarkerArray(
  visualization_msgs::msg::Marker marker)
{
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(std::move(marker));
  return arr;
}

// Convenience: build and wrap in one call.
inline visualization_msgs::msg::MarkerArray pathToMarkerArray(
  const std::vector<Point2D> & path,
  const std::string & ns,
  int id,
  float r, float g, float b,
  float width = 0.05f)
{
  return toMarkerArray(pathToLineMarker(path, ns, id, r, g, b, width));
}

}  // namespace trajectory_controller