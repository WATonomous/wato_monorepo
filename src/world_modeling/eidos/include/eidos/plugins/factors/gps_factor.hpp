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

#include <gtsam/geometry/Point3.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <atomic>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/types.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos
{

/**
 * @brief GPS factor plugin (latching, unary GPSFactor).
 *
 * Subscribes to NavSatFix, converts to UTM, and attaches unary GPS factors
 * to states created by other plugins (e.g. LISO) via latchFactor().
 *
 * Owns its own StaticTransformBroadcaster for the utm→map TF.
 * This is the only plugin that broadcasts TF directly — all other TF
 * is handled by eidos_transform.
 *
 * GPS never creates its own states in the graph.
 */
class GpsFactor : public FactorPlugin
{
public:
  GpsFactor() = default;
  ~GpsFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  /// GPS latches onto existing states — never creates new ones.
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void broadcastUtmToMap();
  Eigen::Vector3d utmToMap(const Eigen::Vector3d & utm_pos) const;

  // Subscriptions + publishers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // GPS queue
  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  mutable std::mutex gps_lock_;

  // Distance gating
  PointType last_gps_point_;
  bool has_last_gps_ = false;
  bool active_ = false;

  // IMU heading
  std::mutex imu_orientation_lock_;
  double latest_imu_yaw_ = 0.0;
  std::atomic<bool> has_imu_orientation_{false};
  double initial_yaw_ = 0.0;
  Eigen::Matrix3d R_map_enu_ = Eigen::Matrix3d::Identity();

  // UTM→map offset
  bool offset_initialized_ = false;
  gtsam::Point3 utm_to_map_offset_{0, 0, 0};
  int utm_zone_ = 0;
  bool utm_is_north_ = true;

  // Frame names
  std::string map_frame_ = "map";
  std::string utm_frame_ = "utm";

  // Parameters
  double max_cov_;
  double min_radius_;
  bool use_elevation_;
  std::vector<double> gps_cov_;
  double pose_cov_threshold_;
  bool add_factors_ = true;
};

}  // namespace eidos
