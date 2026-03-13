#include "eidos/plugins/visualization/factor_graph_visualization.hpp"

#include <cmath>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>

#include "eidos/slam_core.hpp"

namespace eidos {

void FactorGraphVisualization::onInitialize() {
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/factor_graph");
  node_->declare_parameter(prefix + ".state_scale", 0.5);
  node_->declare_parameter(prefix + ".line_width", 0.05);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);
  node_->declare_parameter(prefix + ".mode", "full");
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

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (mode=%s)", name_.c_str(), mode_.c_str());
}

void FactorGraphVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  last_publish_time_ = node_->now();
}

void FactorGraphVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
}

// Helper to get a label prefix for a state based on its owner plugin
static std::string stateLabel(const std::string& owner, uint64_t idx) {
  std::string prefix;
  if (owner.empty()) {
    prefix = "X";
  } else if (owner.find("gps") != std::string::npos) {
    prefix = "G";
  } else if (owner.find("liso") != std::string::npos) {
    prefix = "L";
  } else if (!owner.empty()) {
    prefix = std::string(1, std::toupper(owner[0]));
  } else {
    prefix = "?";
  }
  return prefix + std::to_string(idx);
}

void FactorGraphVisualization::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  // Rate limit
  auto now = node_->now();
  if (publish_rate_ > 0.0 &&
      (now - last_publish_time_).seconds() < 1.0 / publish_rate_) {
    return;
  }
  last_publish_time_ = now;

  const auto& graph = core_->getAccumulatedGraph();

  // Windowed mode: determine spatial filter
  bool windowed = (mode_ == "windowed");
  Eigen::Vector3d window_center;
  double window_r2 = 0.0;
  if (windowed) {
    gtsam::Pose3 current = core_->getCurrentPose();
    window_center = current.translation();
    window_r2 = window_radius_ * window_radius_;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;
  auto stamp = node_->now();

  // ---- Batched state axes: 3 LINE_LIST markers (one per axis color) ----
  visualization_msgs::msg::Marker x_axes, y_axes, z_axes;
  for (auto* ax : {&x_axes, &y_axes, &z_axes}) {
    ax->header.frame_id = map_frame_;
    ax->header.stamp = stamp;
    ax->ns = "fg_state_axes";
    ax->type = visualization_msgs::msg::Marker::LINE_LIST;
    ax->action = visualization_msgs::msg::Marker::ADD;
    ax->pose.orientation.w = 1.0;
    ax->scale.x = state_scale_ * 0.1;
  }
  x_axes.id = marker_id++;
  y_axes.id = marker_id++;
  z_axes.id = marker_id++;
  x_axes.color.r = 1.0f; x_axes.color.a = 1.0f;
  y_axes.color.g = 1.0f; y_axes.color.a = 1.0f;
  z_axes.color.b = 1.0f; z_axes.color.a = 1.0f;

  // ---- Collect state positions, build batched axes + labels ----
  std::unordered_map<gtsam::Key, geometry_msgs::msg::Point> key_positions;
  std::unordered_set<gtsam::Key> visible_keys;
  const auto& map_manager = core_->getMapManager();

  for (const auto& kv : optimized_values) {
    gtsam::Key key = kv.key;

    gtsam::Pose3 pose;
    try {
      pose = optimized_values.at<gtsam::Pose3>(key);
    } catch (...) {
      continue;
    }

    // Windowed mode: skip states outside radius
    if (windowed) {
      Eigen::Vector3d diff = pose.translation() - window_center;
      if (diff.squaredNorm() > window_r2) continue;
    }

    gtsam::Symbol sym(key);
    uint64_t idx = sym.index();

    geometry_msgs::msg::Point pos;
    pos.x = pose.translation().x();
    pos.y = pose.translation().y();
    pos.z = pose.translation().z();
    key_positions[key] = pos;
    visible_keys.insert(key);

    Eigen::Matrix3d rot = pose.rotation().matrix();
    double axis_len = state_scale_;

    // Add axis line segments to batched markers
    geometry_msgs::msg::Point end;

    // X axis
    end.x = pos.x + axis_len * rot(0, 0);
    end.y = pos.y + axis_len * rot(1, 0);
    end.z = pos.z + axis_len * rot(2, 0);
    x_axes.points.push_back(pos);
    x_axes.points.push_back(end);

    // Y axis
    end.x = pos.x + axis_len * rot(0, 1);
    end.y = pos.y + axis_len * rot(1, 1);
    end.z = pos.z + axis_len * rot(2, 1);
    y_axes.points.push_back(pos);
    y_axes.points.push_back(end);

    // Z axis
    end.x = pos.x + axis_len * rot(0, 2);
    end.y = pos.y + axis_len * rot(1, 2);
    end.z = pos.z + axis_len * rot(2, 2);
    z_axes.points.push_back(pos);
    z_axes.points.push_back(end);

    // Text label (individual per state)
    std::string owner = map_manager.getOwnerPlugin(key);
    visualization_msgs::msg::Marker text;
    text.header.frame_id = map_frame_;
    text.header.stamp = stamp;
    text.ns = "fg_labels";
    text.id = marker_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position = pos;
    text.pose.position.z += state_scale_ * 1.2;
    text.pose.orientation.w = 1.0;
    text.scale.z = state_scale_ * 0.6;
    text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
    text.text = stateLabel(owner, idx);
    marker_array.markers.push_back(text);
  }

  if (!x_axes.points.empty()) marker_array.markers.push_back(x_axes);
  if (!y_axes.points.empty()) marker_array.markers.push_back(y_axes);
  if (!z_axes.points.empty()) marker_array.markers.push_back(z_axes);

  // ---- Batched factor edges (one LINE_LIST per factor type) ----
  auto make_line_list = [&](const std::string& ns, float r, float g, float b, float a) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = line_width_;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  };

  visualization_msgs::msg::Marker imu_lines = make_line_list("fg_imu", 0.0f, 1.0f, 1.0f, 0.5f);
  visualization_msgs::msg::Marker between_lines = make_line_list("fg_between", 1.0f, 0.0f, 1.0f, 0.5f);
  visualization_msgs::msg::Marker gps_lines = make_line_list("fg_gps_lines", 1.0f, 1.0f, 0.0f, 0.5f);
  visualization_msgs::msg::Marker unknown_lines = make_line_list("fg_other", 0.5f, 0.5f, 0.5f, 0.5f);

  // Batched SPHERE_LIST for GPS measurement spheres
  visualization_msgs::msg::Marker gps_spheres;
  gps_spheres.header.frame_id = map_frame_;
  gps_spheres.header.stamp = stamp;
  gps_spheres.ns = "fg_gps_spheres";
  gps_spheres.id = marker_id++;
  gps_spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  gps_spheres.action = visualization_msgs::msg::Marker::ADD;
  gps_spheres.pose.orientation.w = 1.0;
  gps_spheres.scale.x = line_width_ * 4.0;
  gps_spheres.scale.y = line_width_ * 4.0;
  gps_spheres.scale.z = line_width_ * 4.0;
  gps_spheres.color.r = 0.0f; gps_spheres.color.g = 1.0f;
  gps_spheres.color.b = 0.0f; gps_spheres.color.a = 0.9f;

  // Batched SPHERE_LIST for unary factor points (Prior etc)
  visualization_msgs::msg::Marker unary_spheres;
  unary_spheres.header.frame_id = map_frame_;
  unary_spheres.header.stamp = stamp;
  unary_spheres.ns = "fg_unary_spheres";
  unary_spheres.id = marker_id++;
  unary_spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  unary_spheres.action = visualization_msgs::msg::Marker::ADD;
  unary_spheres.pose.orientation.w = 1.0;
  unary_spheres.scale.x = line_width_ * 4.0;
  unary_spheres.scale.y = line_width_ * 4.0;
  unary_spheres.scale.z = line_width_ * 4.0;
  unary_spheres.color.r = 1.0f; unary_spheres.color.g = 0.0f;
  unary_spheres.color.b = 0.0f; unary_spheres.color.a = 0.5f;

  // Iterate all factors and batch into appropriate markers
  for (size_t i = 0; i < graph.size(); i++) {
    auto factor = graph.at(i);
    if (!factor) continue;

    auto keys = factor->keys();

    std::string type;
    bool skip = false;

    if (boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor)) {
      type = "imu";
    } else if (boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor)) {
      type = "gps";
    } else if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(factor)) {
      skip = true;
    } else if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
      type = "between";
    } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor)) {
      type = "prior";
    } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(factor) ||
               boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factor) ||
               boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(factor)) {
      skip = true;
    } else {
      type = "unknown";
    }

    if (skip) continue;

    // Collect positioned keys that are visible
    std::vector<geometry_msgs::msg::Point> pts;
    bool any_visible = false;
    for (auto k : keys) {
      auto it = key_positions.find(k);
      if (it != key_positions.end()) {
        pts.push_back(it->second);
        if (visible_keys.count(k)) any_visible = true;
      }
    }
    if (pts.empty()) continue;

    // In windowed mode, skip factors with no visible keys
    if (windowed && !any_visible) continue;

    if (type == "gps") {
      // GPS factor: sphere at measurement position + line to state
      auto gps_f = boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor);
      if (gps_f && !pts.empty()) {
        auto m = gps_f->measurementIn();
        geometry_msgs::msg::Point mp;
        mp.x = m.x(); mp.y = m.y(); mp.z = m.z();
        gps_spheres.points.push_back(mp);
        gps_lines.points.push_back(pts[0]);
        gps_lines.points.push_back(mp);
      }
    } else if (pts.size() == 1) {
      // Unary factor (Prior etc): sphere at state position
      unary_spheres.points.push_back(pts[0]);
    } else if (pts.size() >= 2) {
      // Binary factor: straight line (no Bezier curves)
      visualization_msgs::msg::Marker* target;
      if (type == "imu") target = &imu_lines;
      else if (type == "between") target = &between_lines;
      else target = &unknown_lines;
      target->points.push_back(pts[0]);
      target->points.push_back(pts[1]);
    }
  }

  // Add non-empty batched factor markers
  if (!imu_lines.points.empty()) marker_array.markers.push_back(imu_lines);
  if (!between_lines.points.empty()) marker_array.markers.push_back(between_lines);
  if (!gps_lines.points.empty()) marker_array.markers.push_back(gps_lines);
  if (!unknown_lines.points.empty()) marker_array.markers.push_back(unknown_lines);
  if (!gps_spheres.points.empty()) marker_array.markers.push_back(gps_spheres);
  if (!unary_spheres.points.empty()) marker_array.markers.push_back(unary_spheres);

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::FactorGraphVisualization, eidos::VisualizationPlugin)
