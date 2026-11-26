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

#include <deque>
#include <Eigen/Core>
#include <mutex>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Roll/pitch prior from the IMU's gravity reading (latching, unary attitude factor).
 *
 * The LiDAR odometry constrains each state only relative to the previous one, so roll and pitch
 * drift as a random walk, and with them the map's height. This plugin measures the gravity
 * direction in the body frame over a short window ending at each new state and latches a
 * gtsam::Pose3AttitudeFactor onto it: a 2-DoF constraint on the direction of "up", which leaves
 * yaw and position free.
 *
 * Gravity sources (mode):
 *   accelerometer  mean specific force over the window, minus the vehicle's own acceleration
 *                  estimated from a velocity topic (body-frame twist, e.g. the LiDAR odometry's).
 *                  Absolute, so it cannot drift; windows with fast rotation or a specific force
 *                  far from g are skipped.
 *   orientation    "up" from the IMU message's orientation (needs an INS-grade orientation that
 *                  does not drift).
 *
 * It never creates states, and needs nothing from other plugins beyond a ROS topic.
 */
class GravityFactor : public FactorPlugin
{
public:
  GravityFactor() = default;
  ~GravityFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Attach an attitude factor to a new state if the gravity measurement over the window
   *        ending at its timestamp passes the gates.
   * @param key GTSAM key of the new state.
   * @param timestamp Its sensor timestamp (s).
   * @return The factor, or nothing.
   */
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  struct ImuSample
  {
    double t;
    Eigen::Vector3d accel;  ///< Specific force in base_link (m/s^2)
    Eigen::Vector3d gyro;  ///< Angular rate in base_link (rad/s)
    std::optional<Eigen::Vector3d> up;  ///< World "up" in base_link from the message orientation
  };

  struct VelocitySample
  {
    double t;
    Eigen::Vector3d v;  ///< Body-frame linear velocity (m/s)
  };

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Vehicle acceleration in the body frame over [t0, t1] from the buffered velocity.
   * @return a = dv/dt + w x v, or nothing if the velocity does not cover the window.
   */
  std::optional<Eigen::Vector3d> bodyAcceleration(double t0, double t1, const Eigen::Vector3d & gyro);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;

  std::deque<ImuSample> imu_buffer_;
  std::deque<VelocitySample> velocity_buffer_;
  std::mutex buffer_mtx_;

  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;
  bool active_ = false;

  // Counters for the periodic summary log
  size_t added_ = 0;
  size_t skipped_dynamics_ = 0;
  size_t skipped_data_ = 0;

  // Parameters
  bool add_factors_ = true;
  std::string mode_ = "accelerometer";
  std::string imu_topic_ = "imu/data";
  std::string imu_frame_ = "imu_link";
  std::string velocity_topic_ = "liso/odometry";
  std::string base_link_frame_ = "base_link";
  double window_ = 1.0;  ///< Length of the measurement window ending at the state (s)
  double sigma_ = 0.03;  ///< Standard deviation of the measured gravity direction (rad)
  double huber_k_ = 1.345;  ///< Huber threshold in sigmas (0 = plain Gaussian)
  double gravity_ = 9.81;  ///< Magnitude used by the specific-force gate (m/s^2)
  double max_accel_deviation_ = 0.5;  ///< Skip if |f - a| differs from gravity by more (m/s^2)
  double max_angular_rate_ = 0.3;  ///< Skip if the mean angular rate over the window is higher (rad/s)
  int min_imu_samples_ = 10;
};

}  // namespace eidos
