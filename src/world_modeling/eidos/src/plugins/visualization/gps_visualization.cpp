#include "eidos/plugins/visualization/gps_visualization.hpp"

#include <Eigen/Core>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

void GpsVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/gps");
  node_->declare_parameter(prefix + ".gps_from", "gps_factor");
  node_->declare_parameter(prefix + ".marker_scale", 1.0);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".gps_from", gps_from_);
  node_->get_parameter(prefix + ".marker_scale", marker_scale_);

  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void GpsVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void GpsVisualization::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  const auto& map_manager = core_->getMapManager();

  // Get the utm_to_map offset
  auto global_offset = map_manager.getGlobalData(gps_from_ + "/utm_to_map");
  if (!global_offset.has_value()) return;

  auto offset_vec = std::any_cast<Eigen::Vector4d>(global_offset.value());
  Eigen::Vector3d utm_to_map = offset_vec.head<3>();

  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_keyframes = map_manager.numKeyframes();

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL marker to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;
  auto stamp = node_->now();

  for (int i = 0; i < num_keyframes; i++) {
    auto utm_data = map_manager.getKeyframeData(i, gps_from_ + "/utm_position");
    if (!utm_data.has_value()) continue;

    auto utm_vec = std::any_cast<Eigen::Vector4d>(utm_data.value());
    Eigen::Vector3d utm_pos = utm_vec.head<3>();
    Eigen::Vector3d gps_map_pos = utm_pos - utm_to_map;

    auto& kf_pose = key_poses_6d->points[i];

    // Green sphere at GPS position
    visualization_msgs::msg::Marker gps_marker;
    gps_marker.header.frame_id = map_frame_;
    gps_marker.header.stamp = stamp;
    gps_marker.ns = "gps_positions";
    gps_marker.id = marker_id++;
    gps_marker.type = visualization_msgs::msg::Marker::SPHERE;
    gps_marker.action = visualization_msgs::msg::Marker::ADD;
    gps_marker.pose.position.x = gps_map_pos.x();
    gps_marker.pose.position.y = gps_map_pos.y();
    gps_marker.pose.position.z = gps_map_pos.z();
    gps_marker.pose.orientation.w = 1.0;
    gps_marker.scale.x = marker_scale_;
    gps_marker.scale.y = marker_scale_;
    gps_marker.scale.z = marker_scale_;
    gps_marker.color.r = 0.0f;
    gps_marker.color.g = 1.0f;
    gps_marker.color.b = 0.0f;
    gps_marker.color.a = 0.8f;
    marker_array.markers.push_back(gps_marker);

    // Blue sphere at optimized keyframe position
    visualization_msgs::msg::Marker kf_marker;
    kf_marker.header.frame_id = map_frame_;
    kf_marker.header.stamp = stamp;
    kf_marker.ns = "keyframe_positions";
    kf_marker.id = marker_id++;
    kf_marker.type = visualization_msgs::msg::Marker::SPHERE;
    kf_marker.action = visualization_msgs::msg::Marker::ADD;
    kf_marker.pose.position.x = kf_pose.x;
    kf_marker.pose.position.y = kf_pose.y;
    kf_marker.pose.position.z = kf_pose.z;
    kf_marker.pose.orientation.w = 1.0;
    kf_marker.scale.x = marker_scale_;
    kf_marker.scale.y = marker_scale_;
    kf_marker.scale.z = marker_scale_;
    kf_marker.color.r = 0.0f;
    kf_marker.color.g = 0.0f;
    kf_marker.color.b = 1.0f;
    kf_marker.color.a = 0.8f;
    marker_array.markers.push_back(kf_marker);

    // Line connecting GPS and keyframe positions
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = map_frame_;
    line_marker.header.stamp = stamp;
    line_marker.ns = "gps_kf_lines";
    line_marker.id = marker_id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.05 * marker_scale_;
    line_marker.color.r = 1.0f;
    line_marker.color.g = 1.0f;
    line_marker.color.b = 0.0f;
    line_marker.color.a = 0.6f;

    geometry_msgs::msg::Point p1, p2;
    p1.x = gps_map_pos.x();
    p1.y = gps_map_pos.y();
    p1.z = gps_map_pos.z();
    p2.x = kf_pose.x;
    p2.y = kf_pose.y;
    p2.z = kf_pose.z;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    marker_array.markers.push_back(line_marker);
  }

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsVisualization, eidos::VisualizationPlugin)
