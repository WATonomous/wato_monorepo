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
  void trajectory_callback(const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);
  void control_callback();

  void publish_ackermann(const std::string & frame_id, double speed, double steering_angle);
  void publish_predicted_path(const std::vector<StateVec> & states);

  std::unique_ptr<MpcCore> core_;
  MpcConfig config_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

  // Subscribers
  rclcpp::Subscription<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  wato_trajectory_msgs::msg::Trajectory::SharedPtr latest_trajectory_;
  rclcpp::Time last_trajectory_time_;
  std::string bt_requested_behaviour_;
  double current_speed_ = 0.0;
  double prev_steering_ = 0.0;
  double prev_accel_ = 0.0;

  // Params
  std::string base_frame_;
  std::string standby_msg_;
  double standby_speed_ = 0.0;
  double standby_steering_ = 0.0;
  double control_rate_hz_ = 20.0;
  double wheelbase_ = 2.5667;
  double idle_timeout_sec_ = 2.0;
  bool invert_steering_ = false;
  bool disable_standby_ = false;
};

}  // namespace mpc_controller
