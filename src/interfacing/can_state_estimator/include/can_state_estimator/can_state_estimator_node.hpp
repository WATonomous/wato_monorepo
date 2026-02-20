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
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <std_msgs/msg/float64.hpp>

namespace can_state_estimator
{

class CanStateEstimatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CanStateEstimatorNode(const rclcpp::NodeOptions & options);
  ~CanStateEstimatorNode();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  void read_loop();
  void process_steering_frame(const uint8_t * data);
  void process_wheel_speed_frame(const uint8_t * data);
  void publish_velocity_and_odom();
  bool lookup_wheelbase();
  void stop_can_thread();
  void close_can_socket();

  // CAN socket
  int sock_{-1};
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string can_interface_;
  double steering_conversion_factor_;
  std::string rear_axle_frame_;
  std::string front_axle_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  // Wheelbase (resolved from TF)
  double wheelbase_{0.0};
  bool has_wheelbase_{false};

  // State (protected by mutex, written by CAN thread)
  std::mutex state_mutex_;
  double current_steering_angle_rad_{0.0};
  bool has_steering_angle_{false};
  double wheel_speed_nw_{0.0};  // front-left, km/h
  double wheel_speed_ne_{0.0};  // front-right, km/h
  double wheel_speed_sw_{0.0};  // rear-left, km/h
  double wheel_speed_se_{0.0};  // rear-right, km/h
  bool has_wheel_speeds_{false};

  // Odometry state (only accessed from CAN thread)
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};
  rclcpp::Time last_odom_time_;
  bool odom_initialized_{false};

  // Lifecycle Publishers
  rclcpp_lifecycle::LifecyclePublisher<roscco_msg::msg::SteeringAngle>::SharedPtr steering_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace can_state_estimator
