#include "eidos/plugins/visualization/factor_graph_visualization.hpp"
#include "eidos/core/map_manager.hpp"

#include <functional>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/inference/Symbol.h>

namespace eidos {

// Deterministic color from plugin name
static std_msgs::msg::ColorRGBA pluginColor(const std::string& name, float alpha) {
  static const float palette[][3] = {
    {0.0f, 1.0f, 1.0f},   // cyan
    {1.0f, 0.0f, 1.0f},   // magenta
    {1.0f, 1.0f, 0.0f},   // yellow
    {1.0f, 0.5f, 0.0f},   // orange
    {0.5f, 0.0f, 1.0f},   // purple
    {0.0f, 1.0f, 0.5f},   // spring green
    {1.0f, 0.3f, 0.3f},   // salmon
    {0.3f, 0.7f, 1.0f},   // light blue
  };
  static constexpr int n = sizeof(palette) / sizeof(palette[0]);
  std_msgs::msg::ColorRGBA c;
  c.a = alpha;
  if (name.empty()) {
    c.r = 0.5f; c.g = 0.5f; c.b = 0.5f;
    return c;
  }
  size_t h = std::hash<std::string>{}(name);
  c.r = palette[h % n][0];
  c.g = palette[h % n][1];
  c.b = palette[h % n][2];
  return c;
}

void FactorGraphVisualization::onInitialize() {
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/factor_graph");
  node_->declare_parameter(prefix + ".state_scale", 1.0);
  node_->declare_parameter(prefix + ".line_width", 0.5);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);
  node_->declare_parameter(prefix + ".mode", std::string("full"));
  node_->declare_parameter(prefix + ".window_radius", 50.0);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".state_scale", state_scale_);
  node_->get_parameter(prefix + ".line_width", line_width_);
  node_->get_parameter(prefix + ".publish_rate", publish_rate_);
  node_->get_parameter(prefix + ".mode", mode_);
  node_->get_parameter(prefix + ".window_radius", window_radius_);
  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void FactorGraphVisualization::onActivate() {
  pub_->on_activate();
}

void FactorGraphVisualization::onDeactivate() {
  pub_->on_deactivate();
}

void FactorGraphVisualization::render(const gtsam::Values& optimized_values) {
  if (!pub_->is_activated()) return;
  if (pub_->get_subscription_count() == 0) return;

  auto now = node_->now();
  if (publish_rate_ > 0.0 && (now - last_publish_time_).seconds() < 1.0 / publish_rate_) return;
  last_publish_time_ = now;

  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  // DELETEALL first — clears stale markers from previous renders
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  auto key_list = map_manager_->getKeyList();

  // Render state spheres — colored by owner plugin
  for (const auto& kv : optimized_values) {
    gtsam::Symbol sym(kv.key);
    if (sym.chr() != 'x') continue;

    gtsam::Pose3 pose;
    try {
      pose = optimized_values.at<gtsam::Pose3>(kv.key);
    } catch (...) { continue; }

    std::string owner = map_manager_->getOwnerPlugin(kv.key);
    auto color = pluginColor(owner, 0.8f);

    visualization_msgs::msg::Marker sphere;
    sphere.header.stamp = now;
    sphere.header.frame_id = map_frame_;
    sphere.ns = "states";
    sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = pose.translation().x();
    sphere.pose.position.y = pose.translation().y();
    sphere.pose.position.z = pose.translation().z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = state_scale_;
    sphere.scale.y = state_scale_;
    sphere.scale.z = state_scale_;
    sphere.color = color;
    sphere.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(sphere);

    // Text label: owner initial + state index (e.g. "L0", "L1")
    visualization_msgs::msg::Marker label;
    label.header.stamp = now;
    label.header.frame_id = map_frame_;
    label.ns = "state_labels";
    label.id = id++;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = pose.translation().x();
    label.pose.position.y = pose.translation().y();
    label.pose.position.z = pose.translation().z() + state_scale_ * 1.2;
    label.pose.orientation.w = 1.0;
    label.scale.z = state_scale_ * 0.6;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;
    label.color.a = 1.0f;
    label.text = (owner.empty() ? "?" : std::string(1, std::toupper(owner[0])))
                 + std::to_string(sym.index());
    label.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(label);
  }

  // Render GPS factor markers — larger magenta spheres at GPS-corrected states
  for (gtsam::Key k : key_list) {
    if (!map_manager_->hasKeyframeData(k, "gps_factor/position")) continue;

    gtsam::Pose3 pose;
    try {
      pose = optimized_values.at<gtsam::Pose3>(k);
    } catch (...) { continue; }

    visualization_msgs::msg::Marker gps_sphere;
    gps_sphere.header.stamp = now;
    gps_sphere.header.frame_id = map_frame_;
    gps_sphere.ns = "gps_factors";
    gps_sphere.id = id++;
    gps_sphere.type = visualization_msgs::msg::Marker::SPHERE;
    gps_sphere.action = visualization_msgs::msg::Marker::ADD;
    gps_sphere.pose.position.x = pose.translation().x();
    gps_sphere.pose.position.y = pose.translation().y();
    gps_sphere.pose.position.z = pose.translation().z();
    gps_sphere.pose.orientation.w = 1.0;
    gps_sphere.scale.x = state_scale_ * 1.5;
    gps_sphere.scale.y = state_scale_ * 1.5;
    gps_sphere.scale.z = state_scale_ * 1.5;
    gps_sphere.color.r = 0.8f;
    gps_sphere.color.g = 0.2f;
    gps_sphere.color.b = 0.8f;
    gps_sphere.color.a = 0.8f;
    gps_sphere.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(gps_sphere);

    // GPS label
    gtsam::Symbol gsym(k);
    visualization_msgs::msg::Marker gps_label;
    gps_label.header.stamp = now;
    gps_label.header.frame_id = map_frame_;
    gps_label.ns = "gps_labels";
    gps_label.id = id++;
    gps_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    gps_label.action = visualization_msgs::msg::Marker::ADD;
    gps_label.pose.position.x = pose.translation().x();
    gps_label.pose.position.y = pose.translation().y();
    gps_label.pose.position.z = pose.translation().z() + state_scale_ * 2.0;
    gps_label.pose.orientation.w = 1.0;
    gps_label.scale.z = state_scale_ * 0.5;
    gps_label.color.r = 0.8f;
    gps_label.color.g = 0.2f;
    gps_label.color.b = 0.8f;
    gps_label.color.a = 1.0f;
    gps_label.text = "GPS";
    gps_label.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(gps_label);
  }

  // Render edges from adjacency graph — colored by factor type
  const auto& adjacency = map_manager_->getAdjacency();
  for (const auto& [key, neighbors] : adjacency) {
    gtsam::Pose3 from_pose;
    try {
      from_pose = optimized_values.at<gtsam::Pose3>(key);
    } catch (...) { continue; }

    for (gtsam::Key neighbor : neighbors) {
      if (neighbor <= key) continue;

      gtsam::Pose3 to_pose;
      try {
        to_pose = optimized_values.at<gtsam::Pose3>(neighbor);
      } catch (...) { continue; }

      std::string edge_owner = map_manager_->getEdgeOwner(key, neighbor);
      auto edge_color = pluginColor(edge_owner, 0.7f);

      visualization_msgs::msg::Marker line;
      line.header.stamp = now;
      line.header.frame_id = map_frame_;
      line.ns = "edges";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = line_width_;
      line.color = edge_color;
      line.lifetime = rclcpp::Duration(0, 0);

      geometry_msgs::msg::Point p1, p2;
      p1.x = from_pose.translation().x();
      p1.y = from_pose.translation().y();
      p1.z = from_pose.translation().z();
      p2.x = to_pose.translation().x();
      p2.y = to_pose.translation().y();
      p2.z = to_pose.translation().z();
      line.points.push_back(p1);
      line.points.push_back(p2);
      markers.markers.push_back(line);
    }
  }

  pub_->publish(markers);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::FactorGraphVisualization, eidos::VisualizationPlugin)
