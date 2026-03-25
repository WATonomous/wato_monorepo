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

#include <tf2_ros/buffer.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/plugins/base_motion_model_plugin.hpp"
#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/plugins/base_visualization_plugin.hpp"
#include "eidos/utils/atomic_slot.hpp"
#include "eidos/utils/lock_free_pose.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

class MapManager;

/**
 * @brief Loads, owns, and provides lookup for all plugins.
 *
 * Owns the pluginlib ClassLoaders and the plugin instances.
 * Other components (TransformManager, EidosNode) hold a const pointer
 * for lock-free plugin lookups.
 *
 * Loading is done once during EidosNode::on_configure(). After that,
 * the registry is read-only.
 */
class PluginRegistry
{
public:
  /// Load all plugins from ROS parameters on the given node.
  /// Plugins are initialized with narrow references (no back-pointer to EidosNode).
  void loadAll(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const LockFreePose * estimator_pose,
    const std::atomic<SlamState> * state,
    const AtomicSlot<gtsam::Values> * estimator_values);

  /// Activate all plugins (called during EidosNode::on_activate).
  void activateAll();

  /// Deactivate all plugins (called during EidosNode::on_deactivate).
  void deactivateAll();

  /// Find a factor plugin by name. Returns nullptr if not found.
  std::shared_ptr<FactorPlugin> findFactor(const std::string & name) const
  {
    for (const auto & p : factor_plugins) {
      if (p->getName() == name) return p;
    }
    return nullptr;
  }

  // ---- Plugin collections (public for read access) ----
  std::vector<std::shared_ptr<FactorPlugin>> factor_plugins;
  std::shared_ptr<MotionModelPlugin> motion_model;
  std::vector<std::shared_ptr<RelocalizationPlugin>> reloc_plugins;
  std::vector<std::shared_ptr<VisualizationPlugin>> vis_plugins;

private:
  void loadFactorPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const LockFreePose * estimator_pose,
    const std::atomic<SlamState> * state);
  void loadMotionModel(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer * tf, const std::atomic<SlamState> * state);
  void loadRelocalizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer * tf, MapManager * map_manager);
  void loadVisualizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const AtomicSlot<gtsam::Values> * estimator_values);

  // ---- ClassLoaders (owned, kept alive for plugin lifetime) ----
  std::unique_ptr<pluginlib::ClassLoader<FactorPlugin>> factor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<MotionModelPlugin>> motion_model_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RelocalizationPlugin>> reloc_loader_;
  std::unique_ptr<pluginlib::ClassLoader<VisualizationPlugin>> vis_loader_;
};

}  // namespace eidos
