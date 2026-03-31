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

#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

#include "mpc_controller/mpc_core.hpp"

namespace mpc_controller
{

/**
 * @brief ROS2 lifecycle node that runs the MPC controller.
 *
 * Subscribes to a trajectory from the planner, vehicle odometry, and behaviour
 * tree commands. On each control tick, samples a reference from the trajectory,
 * solves the MPC QP via MpcCore, and publishes an Ackermann drive command.
 *
 * Follows the same lifecycle and idle/standby conventions as PurePursuitNode
 * so it can be swapped in as a drop-in replacement in the action launch file.
 *
 * Lifecycle transitions:
 *  - on_configure: declares and loads parameters, creates subs/pubs and MpcCore.
 *  - on_activate: starts the control timer.
 *  - on_deactivate: stops the control timer.
 *  - on_cleanup / on_shutdown: releases all resources.
 */
class MpcControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MpcControllerNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // ---- Callbacks ----
  void trajectory_callback(const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);
  void control_callback();

  // ---- Publishing helpers ----
  void publish_ackermann(const std::string & frame_id, double speed, double steering_angle);
  void publish_predicted_path(const std::vector<StateVec> & states);

  // ---- MPC ----

  // QP-based trajectory tracking solver
  std::unique_ptr<MpcCore> core_;

  // MPC tuning parameters (populated in on_configure)
  MpcConfig config_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

  // ---- Subscribers ----
  rclcpp::Subscription<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---- TF ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- Trajectory state ----
  wato_trajectory_msgs::msg::Trajectory::SharedPtr latest_trajectory_;

  // Time of last trajectory receive (for idle detection)
  rclcpp::Time last_trajectory_time_;

  // Latest behaviour command from the behaviour tree
  std::string bt_requested_behaviour_;

  // Latest longitudinal speed from odometry (m/s)
  double current_speed_;

  // Previous steering command sent (for MPC rate constraint)
  double prev_steering_;

  // Previous acceleration command sent (for MPC jerk constraint)
  double prev_accel_;

  // ---- Parameters (loaded in on_configure) ----
  std::string base_frame_;
  std::string standby_msg_;
  double standby_speed_;
  double standby_steering_;
  double control_rate_hz_;
  double wheelbase_;
  double idle_timeout_sec_;
  bool invert_steering_;
  bool disable_standby_;
};

}  // namespace mpc_controller
