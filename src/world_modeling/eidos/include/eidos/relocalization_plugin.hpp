#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>

namespace eidos {

// Forward declaration
class SlamCore;

/**
 * @brief Result of a successful relocalization attempt.
 */
struct RelocalizationResult {
  gtsam::Pose3 pose;
  double fitness_score;
  int matched_keyframe_index;
};

/**
 * @brief Base class for relocalization plugins.
 *
 * Separate base class for relocalization methods. Multiple relocalization
 * plugins can be loaded; the first one to succeed wins.
 */
class RelocalizationPlugin {
public:
  virtual ~RelocalizationPlugin() = default;

  const std::string& getName() const { return name_; }

  /**
   * @brief Framework calls this, then calls onInitialize().
   */
  void initialize(
      SlamCore* core,
      const std::string& name,
      rclcpp::Node::SharedPtr node,
      tf2_ros::Buffer* tf,
      rclcpp::CallbackGroup::SharedPtr callback_group) {
    core_ = core;
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    onInitialize();
  }

  /**
   * @brief Plugin creates its own subs/pubs, declares its params.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Activate the plugin.
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate the plugin.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Attempt relocalization against the loaded prior map.
   *
   * Called repeatedly by SlamCore during RELOCALIZING state.
   * @param timestamp Current timestamp.
   * @return A pose if relocalization succeeds, std::nullopt if still searching.
   */
  virtual std::optional<RelocalizationResult> tryRelocalize(double timestamp) = 0;

protected:
  SlamCore* core_ = nullptr;
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace eidos
