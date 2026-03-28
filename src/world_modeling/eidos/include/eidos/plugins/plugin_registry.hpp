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
 * Loading is done once during EidosNode::on_configure().
 */
class PluginRegistry
{
public:
  /**
   * @brief Load and initialize all configured plugins (factor, relocalization,
   *        visualization) via pluginlib.
   *
   * Reads plugin lists from the node's parameters and instantiates each
   * plugin through the corresponding ClassLoader. Each plugin is configured
   * with shared dependencies (TF buffer, MapManager, etc.).
   * @param node             Shared pointer to the parent lifecycle node.
   * @param tf               Pointer to the TF2 buffer for extrinsic lookups.
   * @param map_manager      Pointer to the shared MapManager.
   * @param estimator_pose   Pointer to the lock-free optimized pose slot.
   * @param state            Pointer to the atomic SLAM state.
   * @param estimator_values Pointer to the lock-free optimized values slot.
   * @note Called once during EidosNode::on_configure(). After this, the
   *       registry is effectively read-only.
   */
  void loadAll(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const LockFreePose * estimator_pose,
    const std::atomic<SlamState> * state,
    const AtomicSlot<gtsam::Values> * estimator_values);

  /// @brief Activate all loaded plugins (called during on_activate).
  void activateAll();

  /// @brief Deactivate all loaded plugins (called during on_deactivate).
  void deactivateAll();

  /**
   * @brief Find a factor plugin by name.
   * @param name The plugin name to search for.
   * @return Shared pointer to the plugin, or nullptr if not found.
   */
  std::shared_ptr<FactorPlugin> findFactor(const std::string & name) const
  {
    for (const auto & p : factor_plugins) {
      if (p->getName() == name) return p;
    }
    return nullptr;
  }

  // ---- Plugin collections (public for read access) ----
  std::vector<std::shared_ptr<FactorPlugin>> factor_plugins;
  std::vector<std::shared_ptr<RelocalizationPlugin>> reloc_plugins;
  std::vector<std::shared_ptr<VisualizationPlugin>> vis_plugins;

private:
  /**
   * @brief Load and configure all factor plugins listed in parameters.
   * @param node           Shared pointer to the parent lifecycle node.
   * @param tf             Pointer to the TF2 buffer.
   * @param map_manager    Pointer to the shared MapManager.
   * @param estimator_pose Pointer to the lock-free optimized pose slot.
   * @param state          Pointer to the atomic SLAM state.
   */
  void loadFactorPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const LockFreePose * estimator_pose,
    const std::atomic<SlamState> * state);

  /**
   * @brief Load and configure all relocalization plugins listed in parameters.
   * @param node        Shared pointer to the parent lifecycle node.
   * @param tf          Pointer to the TF2 buffer.
   * @param map_manager Pointer to the shared MapManager.
   */
  void loadRelocalizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, tf2_ros::Buffer * tf, MapManager * map_manager);

  /**
   * @brief Load and configure all visualization plugins listed in parameters.
   * @param node             Shared pointer to the parent lifecycle node.
   * @param tf               Pointer to the TF2 buffer.
   * @param map_manager      Pointer to the shared MapManager.
   * @param estimator_values Pointer to the lock-free optimized values slot.
   */
  void loadVisualizationPlugins(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    MapManager * map_manager,
    const AtomicSlot<gtsam::Values> * estimator_values);

  // ---- ClassLoaders (owned, kept alive for plugin lifetime) ----
  std::unique_ptr<pluginlib::ClassLoader<FactorPlugin>> factor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RelocalizationPlugin>> reloc_loader_;
  std::unique_ptr<pluginlib::ClassLoader<VisualizationPlugin>> vis_loader_;
};

}  // namespace eidos
