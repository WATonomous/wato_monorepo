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

#include <gtsam/geometry/Pose3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos_msgs/srv/predict_relative_transform.hpp"
#include "eidos_transform/ekf_model_plugin.hpp"

namespace eidos_transform
{

/// @brief Measurement source descriptor for an Odometry topic.
///
/// Used for both odom_sources (fed to both EKFs) and map_sources (fed to
/// global EKF only). Each source has per-DOF pose/twist masks and noise.
struct MeasurementSource
{
  std::string name;
  std::string odom_topic;

  std::array<bool, 6> pose_mask = {false, false, false, false, false, false};
  std::array<bool, 6> twist_mask = {false, false, false, false, false, false};
  gtsam::Vector6 pose_noise = gtsam::Vector6::Ones();
  gtsam::Vector6 twist_noise = gtsam::Vector6::Ones();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;

  nav_msgs::msg::Odometry::SharedPtr latest_msg;
  bool has_new_data = false;

  gtsam::Pose3 last_pose;
  bool has_last_pose = false;

  bool logged_first_msg = false;
};

/// @brief Dual-EKF lifecycle node for multi-source odometry fusion and TF broadcasting.
///
/// Architecture (robot_localization style):
/// - Local EKF (odom frame): fuses odom_sources, broadcasts odom→base_link, publishes /odom
/// - Global EKF (map frame): fuses odom_sources + map_sources, derives map→odom via TF lookup
///
/// map→odom is computed as: global_ekf_pose * inv(odom→base_link from TF)
/// This eliminates timestamp matching issues — TF provides time-indexed lookups.
class EidosTransformNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EidosTransformNode(const rclcpp::NodeOptions & options);
  ~EidosTransformNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// @brief Main tick: predict both EKFs, fuse sources, broadcast TF, publish odom.
  void tick();

  /// @brief Fuse a measurement into an EKF using the source's masks and noise.
  void fuseSource(
    std::shared_ptr<EKFModelPlugin> & ekf,
    MeasurementSource & src);

  /// @brief Broadcast odom→base_link from the local EKF.
  void broadcastOdomToBaseTF(const rclcpp::Time & stamp);

  /// @brief Broadcast map→odom derived from global EKF and TF lookup.
  void broadcastMapToOdomTF(const rclcpp::Time & stamp);

  /// @brief Publish fused odometry from the local EKF.
  void publishOdometry(const rclcpp::Time & stamp);

  /// @brief Callback for static utm→map transform.
  void utmToMapCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

  /// @brief PredictRelativeTransform service callback.
  void predictRelativeTransformCallback(
    const std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Request> request,
    std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Response> response);

  static gtsam::Pose3 odomMsgToPose3(const nav_msgs::msg::Odometry & msg);
  static gtsam::Vector6 odomMsgToTwist(const nav_msgs::msg::Odometry & msg);

  // ---- Dual EKF (loaded via pluginlib) ----
  pluginlib::ClassLoader<EKFModelPlugin> ekf_loader_;
  std::shared_ptr<EKFModelPlugin> local_ekf_;   ///< Odom frame: odom→base_link
  std::shared_ptr<EKFModelPlugin> global_ekf_;  ///< Map frame: derives map→odom

  // ---- Measurement sources ----
  std::vector<MeasurementSource> odom_sources_;  ///< Fed to BOTH EKFs
  std::vector<MeasurementSource> map_sources_;   ///< Fed to global EKF ONLY
  std::mutex sources_mutex_;

  // ---- TF ----
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool has_map_source_data_ = false;  ///< True once any map source has been received

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::CallbackGroup::SharedPtr tick_callback_group_;
  rclcpp::Time last_tick_time_;
  bool first_tick_ = true;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // ---- Subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr utm_to_map_sub_;

  // ---- Service ----
  rclcpp::Service<eidos_msgs::srv::PredictRelativeTransform>::SharedPtr predict_srv_;

  // ---- Cached static transforms ----
  geometry_msgs::msg::TransformStamped cached_utm_to_map_;
  bool has_utm_to_map_ = false;

  // ---- Parameters ----
  double tick_rate_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string map_frame_;
  std::string utm_frame_;
  std::string odom_topic_;
  std::string utm_to_map_topic_;
  std::string ekf_plugin_name_;
};

}  // namespace eidos_transform
