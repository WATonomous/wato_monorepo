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

#ifndef WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define WORLD_MODEL__WORLD_MODEL_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model/world_model_writer.hpp"
#include "world_model/world_state.hpp"

namespace world_model
{

/**
 * @brief ROS2 Lifecycle Node that orchestrates the world model.
 *
 * Thin orchestrator that owns WorldState (entity storage), LaneletHandler
 * (map queries), WorldModelWriter (single inbound subscription), and the
 * set of interface components (publishers, services). Delegates all ROS
 * communication to interface components and all data management to
 * WorldState/LaneletHandler.
 */
class WorldModelNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WorldModelNode(const rclcpp::NodeOptions & options);
  ~WorldModelNode() override = default;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // CORE COMPONENTS
  std::unique_ptr<WorldState> world_state_;  // Entity storage
  std::unique_ptr<LaneletHandler> lanelet_handler_;  // Map queries

  // TF (for map loading - utm->map transform)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // SINGLE INBOUND SUBSCRIBER
  std::unique_ptr<WorldModelWriter> writer_;

  // INTERFACE COMPONENTS
  std::vector<std::unique_ptr<InterfaceBase>> interfaces_;

  // FRAMES AND MAP LOADING
  std::string osm_map_path_;
  std::string map_frame_;
  std::string base_frame_;
  std::string utm_frame_;
  std::string projector_type_;
  rclcpp::TimerBase::SharedPtr map_init_timer_;

  /**
   * @brief Attempt to load the lanelet map using TF-derived UTM origin.
   *
   * Called by a wall timer after activation. For UTM projector, waits for the
   * utm-to-map transform to determine the origin offset. For local_cartesian
   * projector (simulation), loads immediately with zero offset. Cancels the
   * timer after success or failure.
   */
  void tryLoadMap();

  /**
   * @brief Create all interface components (publishers, services, subscriber).
   *
   * Instantiates and registers all publisher, service, and subscriber interface
   * objects. Each interface reads its own parameters from the node.
   */
  void createInterfaces();
};

}  // namespace world_model

#endif  // WORLD_MODEL__WORLD_MODEL_NODE_HPP_
