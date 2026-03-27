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
#include <gtsam/nonlinear/Values.h>
#include <tf2_ros/buffer.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "eidos/utils/lock_free_pose.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

// Forward declarations
class MapManager;

/**
 * @brief Base class for factor plugins that produce GTSAM factors from sensor data.
 *
 * Each plugin is self-contained: manages its own subscriptions, publishers, and
 * internal state. EidosNode does NOT broker raw data between plugins.
 *
 * Lock-free pose outputs are enforced by the base class. Subclasses call
 * setMapPose() / setOdomPose() from their sensor callbacks; consumers call
 * getMapPose() / getOdomPose() from any thread without blocking.
 */
class FactorPlugin
{
public:
  virtual ~FactorPlugin() = default;

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
   * @param estimator_pose Lock-free read handle for Estimator's latest optimized pose.
   * @param state Lock-free read handle for EidosNode's current SlamState.
   */
  void initialize(
    const std::string & name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapManager * map_manager,
    const LockFreePose * estimator_pose,
    const std::atomic<SlamState> * state)
  {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    map_manager_ = map_manager;
    estimator_pose_ = estimator_pose;
    state_ = state;
    onInitialize();
  }

  // ---- Lifecycle (pure virtual) ----

  /**
   * @brief Called once after initialize() stores shared resources.
   *
   * Implementations MUST declare ROS parameters, create subscriptions and
   * publishers, and perform any one-time setup here. The node, tf, and
   * map_manager pointers are valid by the time this is called.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Transition the plugin to the active state.
   *
   * Implementations MUST enable their subscriptions and any internal processing.
   * Called when the lifecycle node transitions to ACTIVE.
   */
  virtual void activate() = 0;

  /**
   * @brief Transition the plugin to the inactive state.
   *
   * Implementations MUST stop publishing and disable internal processing.
   * Called when the lifecycle node transitions to INACTIVE.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Called every SLAM tick. Return factors to create a new state.
   * Set result.timestamp to the sensor timestamp of the new state.
   * Return empty if no new data available.
   *
   * Only state-creating plugins (e.g. LISO) override this.
   *
   * @param key GTSAM key allocated for this potential new state.
   * @param timestamp Current SLAM cycle timestamp.
   */
  virtual StampedFactorResult produceFactor(gtsam::Key /*key*/, double /*timestamp*/)
  {
    return {};
  }

  /**
   * @brief Called after a new state is created, to attach additional factors.
   * Return factors that reference the given key (e.g. GPS unary factor).
   * Return empty if nothing to attach.
   *
   * Latching plugins (e.g. GPS, loop closure) override this.
   *
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state.
   */
  virtual StampedFactorResult latchFactor(gtsam::Key /*key*/, double /*timestamp*/)
  {
    return {};
  }

  /**
   * @brief Whether the plugin has warmed up and is ready for tracking.
   *
   * Default: true (no warmup needed). Plugins with sensor warmup requirements
   * (e.g. IMU stationarity detection) override this to return false until
   * sufficient data has been collected.
   *
   * @return true if the plugin is ready for SLAM tracking, false otherwise.
   */
  virtual bool isReady() const
  {
    return true;
  }

  /**
   * @brief Called when EidosNode transitions to TRACKING after relocalization.
   *
   * Use to initialize against a prior map (e.g. build initial submap around
   * the relocalized pose). Only relevant in localization mode.
   *
   * @param pose The relocalized pose in map frame to begin tracking from.
   */
  virtual void onTrackingBegin(const gtsam::Pose3 & /*pose*/)
  {}

  /**
   * @brief Called after ISAM2 optimization with corrected values.
   *
   * Use to update internal state to match corrected poses (e.g. LISO
   * re-anchors its GICP initial guess and triggers submap rebuild).
   *
   * @param values The full set of optimized GTSAM values after ISAM2 update.
   * @param loop_closure Whether a loop closure factor was included in this optimization cycle.
   */
  virtual void onOptimizationComplete(const gtsam::Values & /*values*/, bool /*loop_closure*/)
  {}

  // ---- Lock-free pose outputs (base class enforced) ----

  /**
   * @brief Read the latest map-frame pose (e.g. GICP match result).
   * @return The pose if one has been written, or std::nullopt if no pose is available yet.
   * @note Always non-blocking. Safe to call from any thread.
   */
  std::optional<gtsam::Pose3> getMapPose() const
  {
    return map_pose_.load();
  }

  /**
   * @brief Read the latest odom-frame pose (e.g. incremental scan-to-scan odometry).
   * @return The pose if one has been written, or std::nullopt if no pose is available yet.
   * @note Always non-blocking. Safe to call from any thread.
   */
  std::optional<gtsam::Pose3> getOdomPose() const
  {
    return odom_pose_.load();
  }

protected:
  /**
   * @brief Write the map-frame pose from a sensor callback.
   * @param pose The new map-frame pose to publish.
   * @note Single writer only — must be called from one thread (typically the sensor callback).
   */
  void setMapPose(const gtsam::Pose3 & pose)
  {
    map_pose_.store(pose);
  }

  /**
   * @brief Write the odom-frame pose from a sensor callback.
   * @param pose The new odom-frame pose to publish.
   * @note Single writer only — must be called from one thread (typically the sensor callback).
   */
  void setOdomPose(const gtsam::Pose3 & pose)
  {
    odom_pose_.store(pose);
  }

  std::string name_;  ///< Plugin instance name (from YAML, e.g. "liso_factor")
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  ///< ROS2 node handle for creating subs/pubs/params
  tf2_ros::Buffer * tf_ = nullptr;  ///< TF lookup buffer (read-only, for extrinsics etc.)
  rclcpp::CallbackGroup::SharedPtr callback_group_;  ///< Dedicated callback group for this plugin's subs
  MapManager * map_manager_ = nullptr;  ///< Keyframe + global data store (direct access)
  const LockFreePose * estimator_pose_ = nullptr;  ///< Lock-free read of Estimator's latest optimized pose
  const std::atomic<SlamState> * state_ = nullptr;  ///< Lock-free read of EidosNode's current state

private:
  LockFreePose map_pose_;
  LockFreePose odom_pose_;
};

}  // namespace eidos
