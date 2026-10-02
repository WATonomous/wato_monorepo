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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace can_state_estimator
{

/**
 * @brief Lifecycle node that estimates vehicle state from CAN bus data.
 *
 * Reads steering angle (CAN 0x2B0) and wheel speed (CAN 0x4B0) frames directly
 * from SocketCAN. Publishes steering angle and body velocity for control feedback,
 * and per-wheel joint velocities (sensor_msgs/JointState) for wheel odometry
 * consumers such as eidos::WheelOdomFactor.
 */
class CanStateEstimatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CanStateEstimatorNode(const rclcpp::NodeOptions & options);
  ~CanStateEstimatorNode();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Reads parameters, creates publishers, opens and binds the CAN socket.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  /**
   * @brief Activates publishers and starts the CAN read thread.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  /**
   * @brief Stops the CAN read thread and deactivates publishers.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

  /**
   * @brief Closes the CAN socket and destroys publishers.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  /**
   * @brief Full teardown from any state.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  /**
   * @brief Background thread function that blocks on CAN socket reads and dispatches frames.
   */
  void read_loop();

  /**
   * @brief Decodes a steering angle CAN frame (0x2B0) and publishes SteeringAngle.
   * @param data Raw CAN frame payload (8 bytes).
   */
  void process_steering_frame(const uint8_t * data);

  /**
   * @brief Decodes a wheel speed CAN frame (0x4B0), publishes wheel joint states and body velocity.
   * @param data Raw CAN frame payload (8 bytes).
   */
  void process_wheel_speed_frame(const uint8_t * data);

  /**
   * @brief Publishes per-wheel joint velocities (rad/s) from the latest wheel speeds.
   */
  void publish_wheel_joint_states(double nw, double ne, double sw, double se);

  /**
   * @brief Computes and publishes body velocity from current state.
   */
  void publish_velocity();

  /**
   * @brief Signals the CAN read thread to stop and joins it.
   */
  void stop_can_thread();

  /**
   * @brief Closes the CAN socket file descriptor if open.
   */
  void close_can_socket();

  // CAN socket
  int sock_{-1};
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  // Parameters
  std::string can_interface_;
  double steering_conversion_factor_;
  double wheel_radius_;
  std::string front_left_joint_;
  std::string front_right_joint_;
  std::string rear_left_joint_;
  std::string rear_right_joint_;

  // State (protected by mutex, written by CAN thread)
  std::mutex state_mutex_;
  double current_steering_angle_rad_{0.0};
  bool has_steering_angle_{false};
  double wheel_speed_nw_{0.0};  // front-left, km/h
  double wheel_speed_ne_{0.0};  // front-right, km/h
  double wheel_speed_sw_{0.0};  // rear-left, km/h
  double wheel_speed_se_{0.0};  // rear-right, km/h
  bool has_wheel_speeds_{false};

  // Lifecycle Publishers
  rclcpp_lifecycle::LifecyclePublisher<roscco_msg::msg::SteeringAngle>::SharedPtr steering_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub_;
};

}  // namespace can_state_estimator
