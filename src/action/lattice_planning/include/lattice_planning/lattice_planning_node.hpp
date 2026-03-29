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

#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
#include "lattice_planning/lattice_planning_core.hpp"
#include "lattice_planning_msgs/msg/path_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LatticePlanningNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LatticePlanningNode(const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // path generation functions
  bool point_ahead_of_car(const geometry_msgs::msg::Point & pt);
  bool is_lane_change_requested() const;

  void plan_and_publish_path();
  void plan_lane_follow();
  void plan_lane_change();
  void extend_path_with_centerline(Path & path);

  PathPoint create_terminal_point(
    const geometry_msgs::msg::Point & pt,
    size_t pt_idx,
    const std::vector<geometry_msgs::msg::Point> & centerline,
    const geometry_msgs::msg::Point * prev_pt);

  std::vector<std::vector<int64_t>> get_id_order(
    int64_t curr_id, const std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> & ll_map);

  // subscriber callbacks
  void lanelet_update_callback(const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg);
  void update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void set_preferred_lanes(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);

  // publisher wrappers
  void publish_final_path(const Path & path);
  void publish_available_paths(const std::vector<Path> & paths);

  std::unique_ptr<LatticePlanningCore> core_;

  // subscription topic names
  std::string lanelet_ahead_topic, odom_topic, bt_topic;

  // publisher topic names
  std::string final_path_topic, available_paths_topic;
  double publish_rate_hz_;
  double min_path_length_;

  // parameter structs
  CostFunctionParams cf_params;

  // corridor construction
  bool lane_follow_mode_ = true;
  int num_horizons;
  std::vector<double> lookahead_s_m;  // in metres
  std::vector<double> lane_change_lookahead_s_m_;
  std::vector<std::pair<PathPoint, int64_t>> corridor_terminals;

  std::optional<geometry_msgs::msg::PoseStamped> car_pose;
  std::optional<PathPoint> car_frenet_point;
  std::unordered_map<int64_t, int> preferred_lanelets;
  std::string bt_behaviour_;
  std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> cached_lanelets_;
  int64_t cached_current_lanelet_id_ = -1;

  // subscribers
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr route_ahead_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_;

  // timers
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<lattice_planning_msgs::msg::PathArray>::SharedPtr available_paths_pub_;
};
