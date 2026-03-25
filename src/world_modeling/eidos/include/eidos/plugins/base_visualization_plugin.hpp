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

  const std::string & getName() const
  {
    return name_;
  }

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

  virtual void onInitialize() = 0;

  void activate()
  {
    timer_->reset();
    onActivate();
  }

  void deactivate()
  {
    timer_->cancel();
    onDeactivate();
  }

protected:
  /// Subclass lifecycle hooks.
  virtual void onActivate()
  {}

  virtual void onDeactivate()
  {}

  /// Subclass implements this to publish visualization messages.
  /// Called at the configured rate with the latest optimized values.
  /// Only called when new values are available.
  virtual void render(const gtsam::Values & optimized_values) = 0;

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer * tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  MapManager * map_manager_ = nullptr;

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
