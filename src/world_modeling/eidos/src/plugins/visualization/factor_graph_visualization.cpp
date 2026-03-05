#include "eidos/plugins/visualization/factor_graph_visualization.hpp"

#include <cmath>
#include <map>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/BiasedGPSFactor.h>

#include "eidos/slam_core.hpp"

namespace eidos {

void FactorGraphVisualization::onInitialize() {
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/factor_graph");
  node_->declare_parameter(prefix + ".state_scale", 0.5);
  node_->declare_parameter(prefix + ".line_width", 0.05);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".state_scale", state_scale_);
  node_->get_parameter(prefix + ".line_width", line_width_);
  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void FactorGraphVisualization::activate() {
  active_ = true;
  pub_->on_activate();
}

void FactorGraphVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
}

// Helper to get color for a state key based on its symbol character
static std_msgs::msg::ColorRGBA stateColor(unsigned char chr) {
  std_msgs::msg::ColorRGBA c;
  c.a = 0.9f;
  if (chr == 255) {
    // Prior/init state — white
    c.r = 1.0f; c.g = 1.0f; c.b = 1.0f;
  } else if (chr == 0) {
    // First factor plugin (GPS) — yellow
    c.r = 1.0f; c.g = 1.0f; c.b = 0.0f;
  } else if (chr == 1) {
    // Second factor plugin (loop closure) — orange
    c.r = 1.0f; c.g = 0.5f; c.b = 0.0f;
  } else {
    // Cycling colors for others
    static const float palette[][3] = {
      {0.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {0.5f, 1.0f, 0.5f}, {0.5f, 0.5f, 1.0f}
    };
    int idx = (chr - 2) % 4;
    c.r = palette[idx][0]; c.g = palette[idx][1]; c.b = palette[idx][2];
  }
  return c;
}

// Helper to get a label prefix for a state key
static std::string stateLabel(unsigned char chr) {
  if (chr == 255) return "P";
  if (chr == 0) return "G";
  if (chr == 1) return "L";
  return std::string(1, 'A' + chr);
}

void FactorGraphVisualization::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  const auto& graph = core_->getAccumulatedGraph();

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;
  auto stamp = node_->now();

  // ---- States: collect positions and group overlapping states by index ----
  std::unordered_map<gtsam::Key, geometry_msgs::msg::Point> key_positions;

  // Group states by index — states with same index are in the same keygroup
  // and will overlap spatially, so we combine them into one sphere + merged label
  struct StateInfo {
    unsigned char chr;
    uint64_t idx;
    geometry_msgs::msg::Point pos;
  };
  std::map<uint64_t, std::vector<StateInfo>> index_groups;

  for (const auto& kv : optimized_values) {
    gtsam::Key key = kv.key;

    gtsam::Pose3 pose;
    try {
      pose = optimized_values.at<gtsam::Pose3>(key);
    } catch (...) {
      continue;
    }

    gtsam::Symbol sym(key);
    unsigned char chr = sym.chr();
    uint64_t idx = sym.index();

    geometry_msgs::msg::Point pos;
    pos.x = pose.translation().x();
    pos.y = pose.translation().y();
    pos.z = pose.translation().z();
    key_positions[key] = pos;

    index_groups[idx].push_back({chr, idx, pos});
  }

  // Emit one sphere + one combined text label per group
  for (const auto& [idx, states] : index_groups) {
    if (states.empty()) continue;

    // Use the first state's position (they overlap anyway)
    auto pos = states.front().pos;
    // Use the highest-priority color (prior > plugin states)
    auto color = stateColor(states.front().chr);

    // Sphere marker — full opacity
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = map_frame_;
    sphere.header.stamp = stamp;
    sphere.ns = "fg_states";
    sphere.id = marker_id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position = pos;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = state_scale_;
    sphere.scale.y = state_scale_;
    sphere.scale.z = state_scale_;
    sphere.color = color;
    sphere.color.a = 1.0f;
    marker_array.markers.push_back(sphere);

    // Combined text label
    std::string combined_label;
    for (const auto& s : states) {
      if (!combined_label.empty()) combined_label += "/";
      combined_label += stateLabel(s.chr) + std::to_string(s.idx);
    }

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
    text.text = combined_label;
    marker_array.markers.push_back(text);
  }

  // ---- Factors: classified, labeled, curved when overlapping ----

  // First pass: collect visualizable factors with their type info
  struct FactorVis {
    std::string label;
    std_msgs::msg::ColorRGBA color;
    std::vector<geometry_msgs::msg::Point> pts;  // positioned keys
    std::vector<gtsam::Key> keys;                // corresponding gtsam keys
    bool unary;
  };
  std::vector<FactorVis> factor_list;

  // Track how many factors share a key-pair for curve offset
  // Key: sorted pair of gtsam keys, Value: count so far
  std::map<std::pair<gtsam::Key, gtsam::Key>, int> edge_counts;

  for (size_t i = 0; i < graph.size(); i++) {
    auto factor = graph.at(i);
    if (!factor) continue;

    auto keys = factor->keys();

    std_msgs::msg::ColorRGBA fc;
    fc.a = 0.5f;
    std::string label;
    bool skip = false;

    if (boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor)) {
      fc.r = 0.0f; fc.g = 1.0f; fc.b = 1.0f;
      label = "IMU";
    } else if (boost::dynamic_pointer_cast<gtsam::BiasedGPSFactor>(factor)) {
      fc.r = 1.0f; fc.g = 1.0f; fc.b = 0.0f;
      label = "GPS";
    } else if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(factor)) {
      skip = true;
    } else if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
      fc.r = 1.0f; fc.g = 0.0f; fc.b = 1.0f;
      label = "Between";
    } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor)) {
      fc.r = 1.0f; fc.g = 0.0f; fc.b = 0.0f;
      label = "Prior";
    } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(factor) ||
               boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factor) ||
               boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(factor)) {
      skip = true;
    } else {
      fc.r = 0.5f; fc.g = 0.5f; fc.b = 0.5f;
      label = "?";
    }

    if (skip) continue;

    // Collect positioned keys
    bool has_spatial = false;
    std::vector<geometry_msgs::msg::Point> pts;
    std::vector<gtsam::Key> spatial_keys;
    for (auto k : keys) {
      auto it = key_positions.find(k);
      if (it != key_positions.end()) {
        pts.push_back(it->second);
        spatial_keys.push_back(k);
        has_spatial = true;
      }
    }
    if (!has_spatial) continue;

    factor_list.push_back({label, fc, pts, spatial_keys, keys.size() == 1});
  }

  // Second pass: render factors with curve offsets for overlapping edges
  for (auto& fv : factor_list) {
    if (fv.unary) {
      // Unary factor: small sphere + label at state position
      visualization_msgs::msg::Marker point;
      point.header.frame_id = map_frame_;
      point.header.stamp = stamp;
      point.ns = "fg_factors";
      point.id = marker_id++;
      point.type = visualization_msgs::msg::Marker::SPHERE;
      point.action = visualization_msgs::msg::Marker::ADD;
      point.pose.position = fv.pts[0];
      point.pose.orientation.w = 1.0;
      point.scale.x = line_width_ * 4.0;
      point.scale.y = line_width_ * 4.0;
      point.scale.z = line_width_ * 4.0;
      point.color = fv.color;
      marker_array.markers.push_back(point);

      // Label
      visualization_msgs::msg::Marker text;
      text.header.frame_id = map_frame_;
      text.header.stamp = stamp;
      text.ns = "fg_factor_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = fv.pts[0];
      text.pose.position.z += state_scale_ * 0.8;
      text.pose.orientation.w = 1.0;
      text.scale.z = state_scale_ * 0.4;
      text.color = fv.color;
      text.color.a = 1.0f;
      text.text = fv.label;
      marker_array.markers.push_back(text);
    } else if (fv.pts.size() >= 2) {
      // Binary+ factor: curved line strip + label at midpoint
      auto& p0 = fv.pts[0];
      auto& p1 = fv.pts[1];

      // Determine curve offset based on how many factors share this edge
      gtsam::Key k0 = fv.keys.size() > 0 ? fv.keys[0] : 0;
      gtsam::Key k1 = fv.keys.size() > 1 ? fv.keys[1] : 0;
      auto edge_key = std::make_pair(std::min(k0, k1), std::max(k0, k1));
      int edge_idx = edge_counts[edge_key]++;

      // Direction vector and perpendicular offset for curving
      double dx = p1.x - p0.x;
      double dy = p1.y - p0.y;
      double dz = p1.z - p0.z;
      double len = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Perpendicular in the horizontal plane (rotate direction 90 deg)
      double px = 0.0, py = 0.0, pz = 0.0;
      if (len > 1e-6) {
        px = -dy / len;
        py = dx / len;
        // pz stays 0 — curve in horizontal plane
      }

      // Offset magnitude: alternate sides, increasing distance
      // edge_idx 0 → 0 offset (straight), 1 → +offset, 2 → -offset, etc.
      double curve_amount = 0.0;
      if (edge_idx > 0) {
        double sign = (edge_idx % 2 == 1) ? 1.0 : -1.0;
        curve_amount = sign * ((edge_idx + 1) / 2) * len * 0.3;
      }

      // Long-range factors (spanning intermediate states): curve outward
      // so they don't overlap states in between. Only curve the 2nd+ factor
      // on an edge — the first factor is always drawn straight.
      if (edge_idx > 0 && fv.keys.size() >= 2) {
        gtsam::Symbol s0(fv.keys[0]), s1(fv.keys[1]);
        // Only apply gap curve when both keys use the same symbol character
        // (same plugin's states), otherwise it's a cross-plugin factor
        if (s0.chr() == s1.chr()) {
          int idx_gap = std::abs(static_cast<int>(s0.index()) -
                                 static_cast<int>(s1.index()));
          if (idx_gap > 1) {
            double gap_curve = std::sqrt(static_cast<double>(idx_gap)) * len * 0.25;
            double sign = (curve_amount >= 0.0) ? 1.0 : -1.0;
            curve_amount += sign * gap_curve;
          }
        }
      }

      // Generate a curved line strip with a quadratic bezier (sampled)
      const int num_segments = 8;
      visualization_msgs::msg::Marker line;
      line.header.frame_id = map_frame_;
      line.header.stamp = stamp;
      line.ns = "fg_factors";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = line_width_;
      line.color = fv.color;

      // Midpoint with perpendicular offset (quadratic bezier control point)
      double mx = (p0.x + p1.x) * 0.5 + px * curve_amount;
      double my = (p0.y + p1.y) * 0.5 + py * curve_amount;
      double mz = (p0.z + p1.z) * 0.5 + pz * curve_amount;

      for (int s = 0; s <= num_segments; s++) {
        double t = static_cast<double>(s) / num_segments;
        double u = 1.0 - t;
        // Quadratic bezier: B(t) = (1-t)^2 * P0 + 2(1-t)t * M + t^2 * P1
        geometry_msgs::msg::Point pt;
        pt.x = u * u * p0.x + 2.0 * u * t * mx + t * t * p1.x;
        pt.y = u * u * p0.y + 2.0 * u * t * my + t * t * p1.y;
        pt.z = u * u * p0.z + 2.0 * u * t * mz + t * t * p1.z;
        line.points.push_back(pt);
      }
      marker_array.markers.push_back(line);

      // Label at the curve midpoint (bezier at t=0.5)
      visualization_msgs::msg::Marker text;
      text.header.frame_id = map_frame_;
      text.header.stamp = stamp;
      text.ns = "fg_factor_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = mx;
      text.pose.position.y = my;
      text.pose.position.z = mz + state_scale_ * 0.3;
      text.pose.orientation.w = 1.0;
      text.scale.z = state_scale_ * 0.35;
      text.color = fv.color;
      text.color.a = 1.0f;
      text.text = fv.label;
      marker_array.markers.push_back(text);
    }
  }

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::FactorGraphVisualization, eidos::VisualizationPlugin)
