// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace eidos
{

class MapManager;

/**
 * @brief Result of a successful relocalization attempt.
 */
struct RelocalizationResult
{
  gtsam::Pose3 pose;  ///< Relocalized pose in map frame.
  double fitness_score;  ///< GICP fitness score (lower is better).
  int matched_keyframe_index;  ///< Index of the best-matching prior map keyframe.
};

/**
 * @brief Base class for relocalization plugins.
 *
 * Multiple relocalization plugins can be loaded; the first to succeed wins.
 */
class RelocalizationPlugin
{
public:
  virtual ~RelocalizationPlugin() = default;

  /// @brief Get the plugin instance name (as configured in YAML).
  /// @return Reference to the plugin name string.
  const std::string & getName() const
  {
    return name_;
  }

  /**
   * @brief Framework entry point: stores shared resources, then calls onInitialize().
   *
   * Called exactly once by EidosNode during plugin loading. Subclasses must NOT
   * override this — use onInitialize() for plugin-specific setup.
   *
   * @param name Plugin instance name from YAML configuration.
   * @param node Shared pointer to the lifecycle node (for creating subs/pubs/params).
   * @param tf TF buffer for extrinsic lookups. Non-owning pointer, must outlive plugin.
   * @param callback_group Dedicated callback group for this plugin's subscriptions.
   * @param map_manager Pointer to the shared keyframe/map data store. Non-owning.
   */
  void initialize(
    const std::string & name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapManager * map_manager)
  {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    map_manager_ = map_manager;
    onInitialize();
  }

  /**
   * @brief Called once after initialize() stores shared resources.
   *
   * Implementations MUST declare ROS parameters, create subscriptions, and
   * perform any one-time setup here.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Transition the plugin to the active state.
   *
   * Implementations MUST enable subscriptions and internal processing.
   */
  virtual void activate() = 0;

  /**
   * @brief Transition the plugin to the inactive state.
   *
   * Implementations MUST stop publishing and disable internal processing.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Attempt relocalization against the prior map.
   *
   * Called periodically by EidosNode while in RELOCALIZING state. Multiple
   * relocalization plugins may be loaded; the first to return a result wins.
   *
   * @param timestamp Current SLAM cycle timestamp (seconds).
   * @return RelocalizationResult if a valid pose was found, std::nullopt otherwise.
   */
  virtual std::optional<RelocalizationResult> tryRelocalize(double timestamp) = 0;

protected:
  std::string name_;  ///< Plugin instance name (from YAML).
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  ///< ROS2 node handle.
  tf2_ros::Buffer * tf_ = nullptr;  ///< TF lookup buffer (non-owning).
  rclcpp::CallbackGroup::SharedPtr callback_group_;  ///< Dedicated callback group.
  MapManager * map_manager_ = nullptr;  ///< Keyframe/map data store (non-owning).
};

}  // namespace eidos
