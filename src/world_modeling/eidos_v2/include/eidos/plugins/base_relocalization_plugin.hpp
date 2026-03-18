#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>

namespace eidos {

class MapManager;

struct RelocalizationResult {
  gtsam::Pose3 pose;
  double fitness_score;
  int matched_keyframe_index;
};

/**
 * @brief Base class for relocalization plugins.
 *
 * Multiple relocalization plugins can be loaded; the first to succeed wins.
 */
class RelocalizationPlugin {
public:
  virtual ~RelocalizationPlugin() = default;

  const std::string& getName() const { return name_; }

  void initialize(
      const std::string& name,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      tf2_ros::Buffer* tf,
      rclcpp::CallbackGroup::SharedPtr callback_group,
      MapManager* map_manager) {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    map_manager_ = map_manager;
    onInitialize();
  }

  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /// Attempt relocalization against the prior map. Returns pose if successful.
  virtual std::optional<RelocalizationResult> tryRelocalize(double timestamp) = 0;

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  MapManager* map_manager_ = nullptr;
};

}  // namespace eidos
