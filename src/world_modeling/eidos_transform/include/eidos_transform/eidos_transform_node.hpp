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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

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

/**
 * @brief Measurement source descriptor.
 *
 * Each source corresponds to a nav_msgs/Odometry topic whose pose and/or
 * twist fields are fused into the EKF with the given masks and noise.
 */
struct MeasurementSource
{
  std::string name;
  std::string odom_topic;

  std::array<bool, 6> pose_mask = {false, false, false, false, false, false};
  std::array<bool, 6> twist_mask = {false, false, false, false, false, false};
  gtsam::Vector6 pose_noise = gtsam::Vector6::Ones();
  gtsam::Vector6 twist_noise = gtsam::Vector6::Ones();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;

  // Latest received message (guarded by mutex in the node).
  nav_msgs::msg::Odometry::SharedPtr latest_msg;
  bool has_new_data = false;

  // Previous pose for change detection.
  gtsam::Pose3 last_pose;
  bool has_last_pose = false;
};

/**
 * @brief Lifecycle node for EKF-based multi-source odometry fusion and TF broadcasting.
 *
 * Responsibilities:
 * - Fuse multiple nav_msgs/Odometry sources via a pluggable EKF model.
 * - Broadcast odom->base_link TF at a configurable rate.
 * - Publish fused /odom topic.
 * - Accept map->odom corrections from a pose topic (e.g. SLAM output).
 * - Accept a static utm->map transform from a topic.
 * - Provide a PredictRelativeTransform service for dead-reckoning queries.
 */
class EidosTransformNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct the EidosTransformNode.
   * @param options ROS 2 node options (composable node support).
   */
  explicit EidosTransformNode(const rclcpp::NodeOptions & options);
  ~EidosTransformNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Declare parameters, load EKF plugin, create measurement sources.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /// @brief Activate publishers, start tick timer.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /// @brief Stop tick timer, deactivate publishers.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /// @brief Release all resources and reset state.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /// @brief Shutdown hook; delegates to on_cleanup.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Main EKF tick: predict, fuse all available measurements, broadcast TF, publish odom.
   * @note Runs in its own callback group at tick_rate_ Hz.
   */
  void tick();

  /**
   * @brief Broadcast the odom->base_link TF from the current EKF state.
   * @param stamp Timestamp for the published transform.
   */
  void broadcastOdomToBaseTF(const rclcpp::Time & stamp);

  /**
   * @brief Broadcast the map->odom TF from the cached correction.
   * @param stamp Timestamp for the published transform.
   */
  void broadcastMapToOdomTF(const rclcpp::Time & stamp);

  /**
   * @brief Publish the fused odometry message on the odom topic.
   * @param stamp Timestamp for the published message.
   */
  void publishOdometry(const rclcpp::Time & stamp);

  /**
   * @brief Callback for map->odom correction poses (e.g. from SLAM).
   * @param msg PoseStamped representing the robot pose in the map frame.
   */
  void mapSourceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for static utm->map transform updates.
   * @param msg The TransformStamped defining the utm->map relationship.
   */
  void utmToMapCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

  /**
   * @brief Service callback for dead-reckoning relative transform predictions.
   * @param request Contains the two timestamps to predict between.
   * @param response Filled with the predicted relative transform.
   */
  void predictRelativeTransformCallback(
    const std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Request> request,
    std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Response> response);

  /**
   * @brief Extract a gtsam::Pose3 from an Odometry message's pose field.
   * @param msg The Odometry message.
   * @return Equivalent gtsam::Pose3.
   */
  static gtsam::Pose3 odomMsgToPose3(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief Extract a 6-DOF twist vector from an Odometry message.
   * @param msg The Odometry message.
   * @return gtsam::Vector6 [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z].
   */
  static gtsam::Vector6 odomMsgToTwist(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief Extract a gtsam::Pose3 from a PoseStamped message.
   * @param msg The PoseStamped message.
   * @return Equivalent gtsam::Pose3.
   */
  static gtsam::Pose3 poseStampedToPose3(const geometry_msgs::msg::PoseStamped & msg);

  // ---- EKF model (loaded via pluginlib) ----
  pluginlib::ClassLoader<EKFModelPlugin> ekf_loader_;
  std::shared_ptr<EKFModelPlugin> ekf_model_;

  // ---- Measurement sources ----
  std::vector<MeasurementSource> sources_;
  std::mutex sources_mutex_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::CallbackGroup::SharedPtr tick_callback_group_;
  rclcpp::Time last_tick_time_;
  bool first_tick_ = true;

  // ---- TF broadcasters ----
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // ---- Subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_source_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr utm_to_map_sub_;

  // ---- Service ----
  rclcpp::Service<eidos_msgs::srv::PredictRelativeTransform>::SharedPtr predict_srv_;

  // ---- Cached transforms ----
  gtsam::Pose3 cached_map_to_odom_;
  std::atomic<bool> has_map_to_odom_{false};
  std::mutex map_to_odom_mtx_;

  geometry_msgs::msg::TransformStamped cached_utm_to_map_;
  bool has_utm_to_map_ = false;

  // ---- Parameters ----
  double tick_rate_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string map_frame_;
  std::string utm_frame_;
  std::string odom_topic_;
  std::string map_source_topic_;
  std::string utm_to_map_topic_;
  std::string ekf_plugin_name_;
};

}  // namespace eidos_transform
