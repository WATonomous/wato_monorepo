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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace lidar_aggregator
{

/// @brief A rigid 6-DOF transform (rotation + translation in Euclidean space).
struct RigidTransform
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

/// @brief ROS 2 node that deskews and merges multiple lidar pointclouds.
///
/// Subscribes to a center lidar (CC), two side lidars (NE and NW), and an IMU.
/// Uses message_filters to synchronize the three lidar streams, then applies
/// IMU-based motion compensation and merges them into a single output cloud.
/// Side-to-center extrinsics are looked up via tf2 rather than hard-coded.
class LidarAggregatorNode : public rclcpp::Node
{
public:
  explicit LidarAggregatorNode(const rclcpp::NodeOptions & options);

private:
  /// @brief A single IMU sample stored in the rolling buffer.
  struct ImuSample
  {
    rclcpp::Time stamp;
    Eigen::Quaterniond orientation;
    double gyro_z = 0.0;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  };

  /// @brief Configuration for the online timing-offset estimator.
  struct OffsetEstimatorConfig
  {
    bool enabled = false;
    bool write_back_to_params = true;
    double search_half_window_sec = 0.03;
    double search_step_sec = 0.003;
    double ema_alpha = 0.2;
    double min_yaw_rate_rad_s = 0.05;
    double min_score_gain = 0.01;
    double voxel_size_m = 0.5;
    int min_points = 300;
    double min_offset_sec = -0.1;
    double max_offset_sec = 0.1;
  };

  /// @brief Mutable runtime state for the offset estimator (scores per side).
  struct OffsetEstimatorRuntime
  {
    double ne_last_score = 0.0;
    double nw_last_score = 0.0;
  };

  /// @brief Declare and load all ROS 2 parameters from the parameter server.
  void declare_and_load_parameters();

  /// @brief Attempt to look up NE and NW extrinsics from tf2.
  /// @return true if both transforms were found and stored successfully.
  bool load_extrinsics_from_tf();

  /// @brief Callback for incoming IMU messages. Buffers orientation and gyro data.
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// @brief Callback for GPS messages. Computes system-to-GPS clock offset.
  void gps_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg);

  /// @brief Synchronized callback for all three lidar streams.
  void synced_lidar_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cc_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & ne_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & nw_msg);

  /// @brief Interpolate IMU orientation at the given timestamp via slerp.
  /// @return false if the timestamp is outside the buffered IMU range.
  bool get_orientation_at_time(const rclcpp::Time & stamp, Eigen::Quaterniond & q_out) const;

  /// @brief Compute the rotation delta from @p from_stamp to @p to_stamp using buffered IMU.
  /// @return false if either timestamp cannot be interpolated.
  bool get_rotation_delta(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp, Eigen::Matrix3d & delta_rotation) const;

  /// @brief Retrieve the absolute yaw rate (gyro_z) nearest to @p stamp from the IMU buffer.
  /// @return false if the IMU buffer is empty.
  bool get_abs_gyro_z_at_time(const rclcpp::Time & stamp, double & abs_gyro_z_out) const;

  /// @brief Apply extrinsic-only transform to a side cloud (no IMU deskewing).
  sensor_msgs::msg::PointCloud2::SharedPtr transform_cloud_extrinsic(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const rclcpp::Time & stamp,
    const RigidTransform & t_center_side,
    const std::string & output_frame) const;

  /// @brief Motion-compensate a side cloud into the center lidar frame.
  ///
  /// Applies the side-to-center extrinsic and per-point IMU-derived rotation deltas to
  /// align each point's capture time with @p center_stamp. Falls back to scan-level
  /// deskew if the cloud has no per-point `time` field.
  sensor_msgs::msg::PointCloud2::SharedPtr compensate_side_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & side_msg,
    const rclcpp::Time & center_stamp,
    const RigidTransform & t_center_side,
    double side_time_offset_sec,
    const std::string & output_frame) const;

  /// @brief Within-scan deskew the center lidar cloud.
  ///
  /// Uses per-point `time` offsets to rotate each CC point from its capture time
  /// back to @p center_stamp. Returns a copy with updated x/y/z; returns the
  /// input unchanged (with updated stamp) if no `time` field is present.
  sensor_msgs::msg::PointCloud2::SharedPtr deskew_center_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & center_msg, const rclcpp::Time & center_stamp) const;

  /// @brief Concatenate center, NE, and NW clouds into a single merged output cloud.
  sensor_msgs::msg::PointCloud2::SharedPtr merge_clouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cc,
    const sensor_msgs::msg::PointCloud2::SharedPtr & ne,
    const sensor_msgs::msg::PointCloud2::SharedPtr & nw,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const;

  /// @brief Score a candidate time offset by measuring voxel-overlap after alignment.
  double score_side_offset_candidate(
    const pcl::PointCloud<pcl::PointXYZI> & center_cloud,
    const pcl::PointCloud<pcl::PointXYZI> & side_cloud,
    const rclcpp::Time & center_stamp,
    const rclcpp::Time & side_stamp,
    const RigidTransform & t_center_side,
    double candidate_offset_sec) const;

  /// @brief Run one iteration of the online offset estimator for a side lidar.
  ///
  /// Updates @p side_time_offset_sec in-place if a better-scoring offset is found.
  bool update_side_offset(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & center_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & side_msg,
    const RigidTransform & t_center_side,
    double & side_time_offset_sec,
    double & last_score_out,
    const char * side_name,
    const char * side_param_name);

  /// @brief Publish current time offsets and voxel-overlap scores as diagnostics.
  void publish_offset_diagnostics(const rclcpp::Time & stamp);

  /// @brief Main fusion pipeline. Called when all three lidar clouds are synchronized.
  void try_publish_fusion(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & center_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & ne_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & nw_msg);

  /// @brief Convert a geometry_msgs Transform message to a RigidTransform.
  static RigidTransform rigid_from_transform(const geometry_msgs::msg::Transform & t);

  mutable std::mutex imu_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool extrinsics_loaded_ = false;

  std::deque<ImuSample> imu_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr sub_gps_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cc_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_ne_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_nw_;

  using LidarSyncPolicy = message_filters::sync_policies::
    ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<LidarSyncPolicy>> lidar_sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ne_deskewed_cc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nw_deskewed_cc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_estimated_offsets_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_offset_scores_;

  std::string gps_topic_;
  std::string imu_topic_;
  std::string center_topic_;
  std::string ne_topic_;
  std::string nw_topic_;

  std::string deskewed_ne_output_topic_;
  std::string deskewed_nw_output_topic_;
  std::string merged_output_topic_;
  std::string estimated_offsets_output_topic_;
  std::string offset_scores_output_topic_;
  std::string center_frame_;
  std::string ne_frame_;
  std::string nw_frame_;

  int qos_depth_ = 20;
  double max_pair_dt_sec_ = 0.08;
  double max_imu_buffer_sec_ = 20.0;
  double max_imu_interp_gap_sec_ = 0.05;
  double scan_period_sec_ = 0.1;

  double ne_time_offset_sec_ = 0.0;
  double nw_time_offset_sec_ = 0.0;

  /// @brief Offset (seconds) to add to system-clock timestamps to get GPS time.
  /// Computed from Novatel BESTPOS: gps_unix_time - header.stamp.
  std::atomic<double> clock_offset_sec_{0.0};
  std::atomic<bool> clock_offset_valid_{false};

  OffsetEstimatorConfig estimator_cfg_;
  OffsetEstimatorRuntime estimator_runtime_;

  RigidTransform t_center_ne_;
  RigidTransform t_center_nw_;
};

}  // namespace lidar_aggregator
