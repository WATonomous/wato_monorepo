#include "eidos/plugins/visualization/loop_closure_visualization.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

void LoopClosureVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/loop_closures");
  node_->declare_parameter(prefix + ".loop_closure_from",
                           "euclidean_distance_loop_closure_factor");
  node_->declare_parameter(prefix + ".line_width", 0.1);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".loop_closure_from", loop_closure_from_);
  node_->get_parameter(prefix + ".line_width", line_width_);

  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void LoopClosureVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void LoopClosureVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void LoopClosureVisualization::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_keyframes = map_manager.numKeyframes();

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL marker to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // Build a single LINE_LIST marker with all loop closure edges
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = map_frame_;
  line_marker.header.stamp = node_->now();
  line_marker.ns = "loop_closure_edges";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = line_width_;
  line_marker.color.r = 1.0f;
  line_marker.color.g = 0.0f;
  line_marker.color.b = 0.0f;
  line_marker.color.a = 1.0f;

  std::string key = loop_closure_from_ + "/loop_target";

  for (int i = 0; i < num_keyframes; i++) {
    auto target_data = map_manager.getKeyframeData(i, key);
    if (!target_data.has_value()) continue;

    int target_idx;
    try {
      target_idx = std::any_cast<int>(target_data.value());
    } catch (const std::bad_any_cast&) {
      continue;
    }

    if (target_idx < 0 || target_idx >= num_keyframes) continue;

    auto& source_pose = key_poses_6d->points[i];
    auto& target_pose = key_poses_6d->points[target_idx];

    geometry_msgs::msg::Point p1, p2;
    p1.x = source_pose.x;
    p1.y = source_pose.y;
    p1.z = source_pose.z;
    p2.x = target_pose.x;
    p2.y = target_pose.y;
    p2.z = target_pose.z;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
  }

  if (!line_marker.points.empty()) {
    marker_array.markers.push_back(line_marker);
  }

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LoopClosureVisualization, eidos::VisualizationPlugin)
