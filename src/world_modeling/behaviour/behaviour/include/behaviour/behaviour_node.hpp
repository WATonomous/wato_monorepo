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
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "behaviour/behaviour_tree.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
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
   * @brief Loads the current node parameters used to build and seed the BT.
   */
  void load_params();

  /**
   * @brief Rebuilds the BT and repopulates static blackboard values.
   */
  void rebuild_tree();

  /**
   * @brief Clears all virtual walls from the costmap before rebuilding the BT.
   * @param error_message Populated when the clear-walls request fails.
   * @return true when the wall clear service succeeds.
   */
  bool clear_virtual_walls(std::string & error_message);

  // service callbacks
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // sub callbacks
  void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);
  void route_ahead_callback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg);
  void lanelet_ahead_callback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odom_incremental_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Time callback that ticks the behavior tree.
   */
  void tick_tree_callback();

  // Core component
  std::shared_ptr<BehaviourTree> tree_;

  // TF for ego pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Communications
  rclcpp::TimerBase::SharedPtr tick_tree_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_bt_srv_;
  rclcpp::CallbackGroup::SharedPtr clear_walls_client_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_walls_client_;

  // Subs
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr current_lane_context_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr route_ahead_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelets_ahead_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_incremental_sub_;

  // Configuration
  std::string tree_file_path_;
  std::string map_frame_;
  std::string base_frame_;
  std::string goal_reached_mode_;
  bool enable_console_logging_;
  int service_timeout_ms_;
  int wait_for_service_timeout_ms_;
  double rate_hz_;
  std::size_t traffic_light_state_hypothesis_index_;
  std::size_t world_objects_hypothesis_index_;
  std::vector<std::string> left_lane_change_areas_;
  std::vector<std::string> right_lane_change_areas_;
  double stop_line_wall_width_;
  double stop_line_wall_length_;
  double stop_sign_ego_stop_line_threshold_m_;
  double ego_stopped_velocity_threshold_;
  double intersection_lookahead_m_;
  double goal_reached_threshold_m_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_NODE_HPP_
