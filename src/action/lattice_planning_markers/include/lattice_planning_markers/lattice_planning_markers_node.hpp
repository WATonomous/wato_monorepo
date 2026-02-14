#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "local_planning_msgs/msg/path_array.hpp"

namespace lattice_planning_markers
{

class LatticePlanningMarkersNode : public rclcpp::Node
{
public:
  explicit LatticePlanningMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void availablePathsCallback(const local_planning_msgs::msg::PathArray::SharedPtr msg);

  visualization_msgs::msg::MarkerArray pathToLineStripMarkers(
    const nav_msgs::msg::Path & path,
    const std::string & ns,
    int32_t id,
    float r, float g, float b, float a,
    float line_width) const;

  visualization_msgs::msg::MarkerArray pathArrayToMarkers(
    const local_planning_msgs::msg::PathArray & path_array) const;

  visualization_msgs::msg::Marker makeDeleteAllMarker(const std_msgs::msg::Header & header) const;

  // Subscription topics names
  std::string path_topic_, available_paths_topic_;
  
  // Pulisher topic names
  std::string path_markers_topic_, available_paths_markers_topic_;

  // Parameters (appearance)
  double path_line_width_;
  double available_path_line_width_;
  double available_paths_alpha_;
  int max_available_paths_;
  bool publish_deleteall_each_update_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<local_planning_msgs::msg::PathArray>::SharedPtr available_paths_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr available_paths_markers_pub_;
};

}  // namespace lattice_planning_markers
