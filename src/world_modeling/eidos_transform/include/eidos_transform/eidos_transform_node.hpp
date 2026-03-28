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
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos_msgs/srv/predict_relative_transform.hpp"
#include "eidos_transform/ekf_model_plugin.hpp"

namespace eidos_transform
{

/// @brief Unified measurement source descriptor.
///
/// Supports both nav_msgs/Odometry and sensor_msgs/Imu topics via the
/// `type` field. Can appear in odom_sources, map_sources, or both.
struct MeasurementSource
{
  std::string name;
  std::string type = "odom";  ///< "odom" or "imu"
  std::string topic;

  // ---- Common: per-DOF masks and noise ----
  std::array<bool, 6> pose_mask = {false, false, false, false, false, false};
  std::array<bool, 6> twist_mask = {false, false, false, false, false, false};
  gtsam::Vector6 pose_noise = gtsam::Vector6::Ones();
  gtsam::Vector6 twist_noise = gtsam::Vector6::Ones();

  // ---- Odom-type state ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  // ---- IMU-type config ----
  std::string imu_frame = "imu_link";
  bool use_orientation = true;
  bool use_angular_velocity = true;
  bool use_linear_acceleration = false;
  double gravity = 9.80511;
  bool gravity_compensated = false;
  gtsam::Vector6 orientation_noise = gtsam::Vector6::Ones();
  gtsam::Vector6 angular_velocity_noise = gtsam::Vector6::Ones();
  gtsam::Vector6 linear_velocity_noise = gtsam::Vector6::Ones();

  // ---- IMU-type state ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  sensor_msgs::msg::Imu::SharedPtr latest_imu;
  Eigen::Matrix3d R_base_imu = Eigen::Matrix3d::Identity();
  bool has_imu_tf = false;
  double last_imu_time = 0.0;

  // ---- Common state ----
  bool has_new_data = false;
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

  /// @brief Fuse a measurement source into an EKF (handles both odom and imu types).
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

  // ---- Rewind-replay for delayed measurements (global EKF only) ----

  /// @brief Record of an applied measurement for replay during rewind.
  struct MeasurementRecord
  {
    double time;
    enum class Target { LOCAL, GLOBAL, BOTH } target;

    // Which source produced this
    std::string source_name;
    std::string source_type;  // "odom" or "imu"

    // Odom data
    gtsam::Pose3 pose;
    gtsam::Vector6 twist = gtsam::Vector6::Zero();
    std::array<bool, 6> pose_mask = {};
    std::array<bool, 6> twist_mask = {};
    gtsam::Vector6 pose_noise = gtsam::Vector6::Ones();
    gtsam::Vector6 twist_noise = gtsam::Vector6::Ones();

    // IMU data
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_rpy = Eigen::Vector3d::Zero();
    bool has_orientation = false;
    bool use_orientation = false;
    bool use_angular_velocity = false;
    bool use_linear_acceleration = false;
    gtsam::Vector6 imu_orientation_noise = gtsam::Vector6::Ones();
    gtsam::Vector6 imu_angular_velocity_noise = gtsam::Vector6::Ones();
    Eigen::Vector3d imu_accel_noise = Eigen::Vector3d::Ones();
    double imu_dt = 0.0;
  };

  /// @brief Apply a measurement record to an EKF.
  void applyMeasurement(std::shared_ptr<EKFModelPlugin> & ekf, const MeasurementRecord & rec);

  /// @brief Rewind the global EKF and replay with delayed measurement inserted.
  void rewindAndReplay(double delayed_time);

  std::deque<StateSnapshot> global_state_history_;
  std::deque<MeasurementRecord> global_measurement_history_;
  double global_ekf_time_ = 0.0;
  static constexpr size_t kMaxHistory = 500;

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
