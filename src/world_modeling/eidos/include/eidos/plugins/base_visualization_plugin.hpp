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

#include <gtsam/nonlinear/Values.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "eidos/utils/atomic_slot.hpp"
#include "eidos/utils/lock_free_pose.hpp"

namespace eidos
{

class MapManager;

/**
 * @brief Base class for read-only visualization plugins.
 *
 * Each visualization plugin runs on its own timer and callback group,
 * reading optimized values from an AtomicSlot on the Estimator (lock-free).
 * Plugins never block each other — each ticks independently.
 *
 * Subclasses implement render() to publish RViz messages.
 * The base class handles the timer lifecycle.
 */
class VisualizationPlugin
{
public:
  virtual ~VisualizationPlugin() = default;

  /// @brief Get the plugin instance name (as configured in YAML).
  /// @return Reference to the plugin name string.
  const std::string & getName() const
  {
    return name_;
  }

  /**
   * @brief Framework entry point: stores shared resources, creates the render timer,
   *        then calls onInitialize().
   *
   * Called exactly once by EidosNode during plugin loading. The timer starts
   * cancelled; activate() enables it. Subclasses must NOT override this.
   *
   * @param name Plugin instance name from YAML configuration.
   * @param node Shared pointer to the lifecycle node (for creating pubs/params).
   * @param tf TF buffer for extrinsic lookups. Non-owning pointer, must outlive plugin.
   * @param map_manager Pointer to the shared keyframe/map data store. Non-owning.
   * @param values_slot Lock-free slot for reading Estimator's latest optimized GTSAM Values.
   * @param rate_hz Desired render frequency in Hz.
   */
  void initialize(
    const std::string & name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const AtomicSlot<gtsam::Values> * values_slot,
    double rate_hz)
  {
    name_ = name;
    node_ = node;
    tf_ = tf;
    map_manager_ = map_manager;
    values_slot_ = values_slot;

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VisualizationPlugin::tick, this),
      callback_group_);
    timer_->cancel();  // starts inactive, activate() enables it

    onInitialize();
  }

  /**
   * @brief Called once after initialize() stores shared resources and creates the timer.
   *
   * Implementations MUST declare ROS parameters and create publishers here.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Enable the render timer and transition to the active state.
   *
   * Resets the wall timer so tick() begins firing at the configured rate,
   * then calls onActivate() for subclass-specific activation logic.
   */
  void activate()
  {
    timer_->reset();
    onActivate();
  }

  /**
   * @brief Cancel the render timer and transition to the inactive state.
   *
   * Stops the wall timer so tick() no longer fires, then calls onDeactivate()
   * for subclass-specific cleanup.
   */
  void deactivate()
  {
    timer_->cancel();
    onDeactivate();
  }

protected:
  /// @brief Optional hook called after the render timer is enabled.
  virtual void onActivate()
  {}

  /// @brief Optional hook called after the render timer is cancelled.
  virtual void onDeactivate()
  {}

  /**
   * @brief Publish visualization messages to RViz.
   *
   * Called at the configured rate by the internal timer. Subclasses MUST
   * implement this to convert optimized values and/or MapManager data into
   * publishable ROS messages.
   *
   * @param optimized_values Latest optimized GTSAM Values from ISAM2, or
   *        empty Values in localization mode (where no ISAM2 exists).
   * @note In localization mode, use MapManager directly for pose data.
   */
  virtual void render(const gtsam::Values & optimized_values) = 0;

  std::string name_;                                   ///< Plugin instance name (from YAML).
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;    ///< ROS2 node handle.
  tf2_ros::Buffer * tf_ = nullptr;                     ///< TF lookup buffer (non-owning).
  rclcpp::CallbackGroup::SharedPtr callback_group_;    ///< Dedicated callback group for render timer.
  MapManager * map_manager_ = nullptr;                 ///< Keyframe/map data store (non-owning).

private:
  void tick()
  {
    auto values = values_slot_->load();
    // Render even without optimized values (localization mode has no ISAM2).
    // Pass empty Values if slot is null — render() uses MapManager poses.
    static const gtsam::Values empty_values;
    render(values ? *values : empty_values);
  }

  const AtomicSlot<gtsam::Values> * values_slot_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace eidos
