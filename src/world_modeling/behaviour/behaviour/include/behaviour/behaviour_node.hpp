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

#ifndef BEHAVIOUR__BEHAVIOUR_NODE_HPP_
#define BEHAVIOUR__BEHAVIOUR_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviour/area_occupancy_store.hpp"
#include "behaviour/behaviour_tree.hpp"
#include "behaviour/dynamic_object_store.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
   * @class BehaviourNode
   * @brief A standard node that manages Eve's behavior tree and handles world state updates.
   */
class BehaviourNode : public rclcpp::Node
{
public:
  /**
     * @brief Initializes parameters, TF listeners, subscribers, and the behavior tree.
     * @param options Node options for configuration.
     */
  explicit BehaviourNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
     * @brief Virtual destructor.
     */
  virtual ~BehaviourNode() = default;

  /**
     * @brief Creates and initializes the behavior tree after node construction.
     */
  void init();

private:
  /**
   * @brief Time callback that ticks the behavior tree.
   */
  void tickTreeTimerCallback();

  // Core component
  std::shared_ptr<BehaviourTree> tree_;
  std::shared_ptr<DynamicObjectStore> dynamic_objects_store_;
  std::shared_ptr<AreaOccupancyStore> area_occupancy_store_;

  // TF for ego pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Communications
  rclcpp::TimerBase::SharedPtr tick_tree_timer_;

  // Subs
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr current_lane_context_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<world_model_msgs::msg::WorldObjectArray>::SharedPtr dynamic_objects_sub_;
  rclcpp::Subscription<world_model_msgs::msg::AreaOccupancy>::SharedPtr area_occupancy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;

  // Configuration
  std::string map_frame_;
  std::string base_frame_;

};
}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_NODE_HPP_
