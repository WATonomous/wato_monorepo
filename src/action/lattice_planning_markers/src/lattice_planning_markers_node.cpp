#include "lattice_planning_markers/lattice_planning_markers_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp_components/register_node_macro.hpp>

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace lattice_planning_markers
{

LatticePlanningMarkersNode::LatticePlanningMarkersNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lattice_planning_markers_node", options)
{
  // Subscription topics
  path_topic_ = this->declare_parameter<std::string>("path_topic", "path");
  available_paths_topic_ =
    this->declare_parameter<std::string>("available_paths_topic", "available_paths");

  // Publisher topics
  path_markers_topic_ =
    this->declare_parameter<std::string>("path_markers_topic", "path_markers");
  available_paths_markers_topic_ =
    this->declare_parameter<std::string>("available_paths_markers_topic", "available_paths_markers");

  // Appearance parameters.
  path_line_width_ = this->declare_parameter<double>("path_line_width", 0.05);
  available_path_line_width_ = this->declare_parameter<double>("available_path_line_width", 0.03);
  available_paths_alpha_ = this->declare_parameter<double>("available_paths_alpha", 0.85);
  max_available_paths_ = this->declare_parameter<int>("max_available_paths", 50);

  // If true, publish a DELETEALL marker before publishing updates, to avoid stale markers in RViz.
  publish_deleteall_each_update_ =
    this->declare_parameter<bool>("publish_deleteall_each_update", true);

  path_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    path_markers_topic_, rclcpp::QoS(10));
  available_paths_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    available_paths_markers_topic_, rclcpp::QoS(10));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(10),
    std::bind(&LatticePlanningMarkersNode::pathCallback, this, std::placeholders::_1));

  available_paths_sub_ = this->create_subscription<local_planning_msgs::msg::PathArray>(
    available_paths_topic_, rclcpp::QoS(10),
    std::bind(&LatticePlanningMarkersNode::availablePathsCallback, this, std::placeholders::_1));
}

visualization_msgs::msg::Marker LatticePlanningMarkersNode::makeDeleteAllMarker(
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "lattice_planning";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;  // type is ignored for DELETEALL
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

visualization_msgs::msg::MarkerArray LatticePlanningMarkersNode::pathToLineStripMarkers(
  const nav_msgs::msg::Path & path,
  const std::string & ns,
  int32_t id,
  float r, float g, float b, float a,
  float line_width) const
{
  visualization_msgs::msg::MarkerArray out;

  visualization_msgs::msg::Marker line;
  line.header = path.header;
  line.ns = ns;
  line.id = id;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;  // RViz uses "points" for line strips
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;  // identity
  line.scale.x = static_cast<float>(line_width);  // width for LINE_STRIP

  line.color.r = r;
  line.color.g = g;
  line.color.b = b;
  line.color.a = a;

  line.points.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = ps.pose.position.z;
    line.points.push_back(p);
  }

  out.markers.push_back(line);
  return out;
}

visualization_msgs::msg::MarkerArray LatticePlanningMarkersNode::pathArrayToMarkers(
  const local_planning_msgs::msg::PathArray & path_array) const
{
  visualization_msgs::msg::MarkerArray out;

  if (publish_deleteall_each_update_) {
    out.markers.push_back(makeDeleteAllMarker(path_array.header));
  }

  const size_t n_paths = path_array.paths.size();
  const size_t n_costs = path_array.costs.size();
  const size_t n = std::min(n_paths, n_costs);

  const size_t n_limited =
    static_cast<size_t>(std::max(0, max_available_paths_));
  const size_t n_use = std::min(n, n_limited);

  // Compute min/max costs (for color mapping). If costs are missing, we just use a constant color.
  double cmin = std::numeric_limits<double>::infinity();
  double cmax = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n_use; ++i) {
    cmin = std::min(cmin, path_array.costs[i]);
    cmax = std::max(cmax, path_array.costs[i]);
  }
  const bool have_cost_range = std::isfinite(cmin) && std::isfinite(cmax) && (cmax > cmin);

  for (size_t i = 0; i < n_use; ++i) {
    const auto & p = path_array.paths[i];
    const double cost = path_array.costs[i];

    // Map cost to a simple green->red gradient (lower cost = greener).
    float r = 1.0f, g = 0.0f, b = 0.2f;
    if (have_cost_range) {
      const double t = (cost - cmin) / (cmax - cmin);  // 0..1
      r = static_cast<float>(std::clamp(t, 0.0, 1.0));
      g = static_cast<float>(std::clamp(1.0 - t, 0.0, 1.0));
      b = 0.1f;
    } else {
      // No usable range; default to cyan-ish.
      r = 0.1f; g = 0.8f; b = 1.0f;
    }

    // Use path header if provided; otherwise fall back to PathArray header.
    nav_msgs::msg::Path tmp = p;
    if (tmp.header.frame_id.empty()) {
      tmp.header = path_array.header;
    }

    auto markers = pathToLineStripMarkers(
      tmp, "available_paths", static_cast<int32_t>(i),
      r, g, b, static_cast<float>(available_paths_alpha_),
      static_cast<float>(available_path_line_width_));

    // If we published DELETEALL above, it's okay to just append ADD markers.
    out.markers.insert(out.markers.end(), markers.markers.begin(), markers.markers.end());
  }

  return out;
}

void LatticePlanningMarkersNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray out;

  if (publish_deleteall_each_update_) {
    out.markers.push_back(makeDeleteAllMarker(msg->header));
  }

  auto markers = pathToLineStripMarkers(
    *msg, "path", 0,
    0.2f, 0.6f, 1.0f, 1.0f,
    static_cast<float>(path_line_width_));

  out.markers.insert(out.markers.end(), markers.markers.begin(), markers.markers.end());
  path_markers_pub_->publish(out);
}

void LatticePlanningMarkersNode::availablePathsCallback(
  const local_planning_msgs::msg::PathArray::SharedPtr msg)
{
  auto out = pathArrayToMarkers(*msg);
  available_paths_markers_pub_->publish(out);
}

}  // namespace lattice_planning_markers

RCLCPP_COMPONENTS_REGISTER_NODE(lattice_planning_markers::LatticePlanningMarkersNode)
