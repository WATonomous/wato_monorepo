#include "eidos/plugins/visualization/gps_visualization.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/geometry/Point3.h>

#include "eidos/slam_core.hpp"

namespace eidos {

void GpsVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/gps");
  node_->declare_parameter(prefix + ".gps_from", "gps_factor");
  node_->declare_parameter(prefix + ".marker_scale", 1.0);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".gps_from", gps_from_);
  node_->get_parameter(prefix + ".marker_scale", marker_scale_);
  node_->get_parameter(prefix + ".publish_rate", publish_rate_);

  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void GpsVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  last_publish_time_ = node_->now();
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

  // Rate limit
  auto now = node_->now();
  if (publish_rate_ > 0.0 &&
      (now - last_publish_time_).seconds() < 1.0 / publish_rate_) {
    return;
  }
  last_publish_time_ = now;

  const auto& map_manager = core_->getMapManager();

  auto key_poses_6d = map_manager.getKeyPoses6D();
  auto key_list = map_manager.getKeyList();

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL marker to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  auto stamp = node_->now();

  // Batched SPHERE_LIST for GPS positions
  visualization_msgs::msg::Marker gps_spheres;
  gps_spheres.header.frame_id = map_frame_;
  gps_spheres.header.stamp = stamp;
  gps_spheres.ns = "gps_positions";
  gps_spheres.id = 0;
  gps_spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  gps_spheres.action = visualization_msgs::msg::Marker::ADD;
  gps_spheres.pose.orientation.w = 1.0;
  gps_spheres.scale.x = marker_scale_;
  gps_spheres.scale.y = marker_scale_;
  gps_spheres.scale.z = marker_scale_;
  gps_spheres.color.r = 0.0f;
  gps_spheres.color.g = 1.0f;
  gps_spheres.color.b = 0.0f;
  gps_spheres.color.a = 0.8f;

  // Batched SPHERE_LIST for keyframe positions
  visualization_msgs::msg::Marker kf_spheres;
  kf_spheres.header.frame_id = map_frame_;
  kf_spheres.header.stamp = stamp;
  kf_spheres.ns = "keyframe_positions";
  kf_spheres.id = 1;
  kf_spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  kf_spheres.action = visualization_msgs::msg::Marker::ADD;
  kf_spheres.pose.orientation.w = 1.0;
  kf_spheres.scale.x = marker_scale_;
  kf_spheres.scale.y = marker_scale_;
  kf_spheres.scale.z = marker_scale_;
  kf_spheres.color.r = 0.0f;
  kf_spheres.color.g = 0.0f;
  kf_spheres.color.b = 1.0f;
  kf_spheres.color.a = 0.8f;

  // Batched LINE_LIST for connecting lines
  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = map_frame_;
  lines.header.stamp = stamp;
  lines.ns = "gps_kf_lines";
  lines.id = 2;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.05 * marker_scale_;
  lines.color.r = 1.0f;
  lines.color.g = 1.0f;
  lines.color.b = 0.0f;
  lines.color.a = 0.6f;

  for (auto gtsam_key : key_list) {
    // Read stored map-frame GPS position
    auto gps_data = map_manager.getKeyframeData(gtsam_key, gps_from_ + "/position");
    if (!gps_data.has_value()) continue;

    auto gps_pos = std::any_cast<gtsam::Point3>(gps_data.value());

    int cloud_idx = map_manager.getCloudIndex(gtsam_key);
    if (cloud_idx < 0) continue;
    auto& kf_pose = key_poses_6d->points[cloud_idx];

    // If elevation is disabled, use keyframe z so markers are visually comparable
    double gps_z = gps_pos.z();
    if (std::abs(gps_z) < 1e-3) {
      gps_z = kf_pose.z;
    }

    geometry_msgs::msg::Point p1, p2;
    p1.x = gps_pos.x();
    p1.y = gps_pos.y();
    p1.z = gps_z;
    p2.x = kf_pose.x;
    p2.y = kf_pose.y;
    p2.z = kf_pose.z;

    gps_spheres.points.push_back(p1);
    kf_spheres.points.push_back(p2);
    lines.points.push_back(p1);
    lines.points.push_back(p2);
  }

  if (!gps_spheres.points.empty()) marker_array.markers.push_back(gps_spheres);
  if (!kf_spheres.points.empty()) marker_array.markers.push_back(kf_spheres);
  if (!lines.points.empty()) marker_array.markers.push_back(lines);

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsVisualization, eidos::VisualizationPlugin)
