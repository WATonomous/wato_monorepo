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

#include <gtsam/linear/NoiseModel.h>

#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/odometry_buffer.hpp"

namespace eidos
{

/**
 * @brief Between-factor plugin for an external odometry source (e.g. visual odometry).
 *
 * Subscribes to a nav_msgs/Odometry stream, buffers its poses, and constrains every pair of
 * consecutive SLAM states with the source's relative motion between the two state timestamps
 * (a BetweenFactor<Pose3> with a robust, distance-scaled noise model).
 *
 * The source usually lags the state-creating sensor (camera transport + processing), so the
 * factor for states (k-1, k) is delivered on a later latchFactor() call, once the odometry
 * covers state k's timestamp -- the same deferred delivery the loop closure plugin uses.
 * Buffer gaps (tracking loss / reset of the source) are never bridged.
 */
class OdometryFactor : public FactorPlugin
{
public:
  OdometryFactor() = default;
  ~OdometryFactor() override = default;

  /// @brief Declare ROS parameters, create the odometry subscription and residual publisher.
  void onInitialize() override;

  /// @brief Start accepting odometry and producing factors.
  void activate() override;

  /// @brief Stop producing factors and drop buffered data.
  void deactivate() override;

  /**
   * @brief Queue the new state and deliver factors for earlier state pairs the odometry now covers.
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state (seconds).
   * @return BetweenFactor<Pose3> for each covered pair (possibly none).
   */
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

  /// @brief Cache optimized poses of pending states for gating against the graph.
  void onOptimizationComplete(const gtsam::Values & values, bool graph_corrected) override;

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// @brief Resolve base_link <- odometry child frame once. @return false if TF is not available yet.
  bool resolveExtrinsic(const std::string & child_frame);

  /// @brief Noise model for a relative motion of the given length.
  gtsam::SharedNoiseModel noiseFor(double distance) const;

  void logCounters();

  // ---- ROS ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr residual_pub_;

  // ---- Odometry buffer (written by the subscription, read by latchFactor) ----
  std::mutex buffer_mtx_;
  std::optional<OdometryBuffer> buffer_;
  std::optional<gtsam::Pose3> T_base_child_;
  std::string source_frame_;

  // ---- SLAM-thread state ----
  KeyframePairQueue pairs_;
  std::map<gtsam::Key, gtsam::Pose3> graph_poses_;

  // ---- Parameters ----
  std::string odom_topic_;
  std::string base_link_frame_;
  double time_offset_ = 0.0;
  double max_gap_ = 0.15;
  double max_pending_age_ = 3.0;
  double buffer_duration_ = 30.0;
  double rot_sigma_ = 0.005;
  double rot_sigma_per_m_ = 0.002;
  double trans_sigma_ = 0.05;
  double trans_sigma_per_m_ = 0.03;
  std::string robust_kernel_ = "huber";
  double robust_k_ = 1.345;
  double gate_trans_ = 1.0;
  double gate_rot_ = 0.1;
  bool add_factors_ = true;
  bool active_ = false;

  // ---- Counters ----
  struct Counters
  {
    std::size_t emitted = 0;
    std::size_t gated = 0;
    std::size_t gaps = 0;
    std::size_t out_of_range = 0;
    std::size_t expired = 0;
    std::size_t ungated = 0;
  } counters_;

  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace eidos
