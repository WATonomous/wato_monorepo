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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_aggregator
{

struct RigidTransform
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

class LidarAggregatorNode : public rclcpp::Node
{
public:
  explicit LidarAggregatorNode(const rclcpp::NodeOptions & options);

private:
  struct ImuSample
  {
    rclcpp::Time stamp;
    Eigen::Quaterniond orientation;
    double gyro_z = 0.0;
  };

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

  struct OffsetEstimatorRuntime
  {
    double ne_last_score = 0.0;
    double nw_last_score = 0.0;
  };

  void declare_and_load_parameters();
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidar_cc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidar_ne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidar_nw_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool get_orientation_at_time(const rclcpp::Time & stamp, Eigen::Quaterniond & q_out) const;
  bool get_rotation_delta(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp, Eigen::Matrix3d & delta_rotation) const;
  bool get_abs_gyro_z_at_time(const rclcpp::Time & stamp, double & abs_gyro_z_out) const;

  sensor_msgs::msg::PointCloud2::SharedPtr compensate_side_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & side_msg,
    const rclcpp::Time & center_stamp,
    const RigidTransform & t_center_side,
    double side_time_offset_sec,
    const std::string & output_frame) const;

  sensor_msgs::msg::PointCloud2::SharedPtr merge_clouds(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cc,
    const sensor_msgs::msg::PointCloud2::SharedPtr & ne,
    const sensor_msgs::msg::PointCloud2::SharedPtr & nw,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const;

  bool maybe_update_side_offset_locked(
    const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg,
    const sensor_msgs::msg::PointCloud2::SharedPtr & side_msg,
    const RigidTransform & t_center_side,
    double & side_time_offset_sec,
    double & last_score_out,
    const char * side_name,
    const char * side_param_name);

  double score_side_offset_candidate(
    const pcl::PointCloud<pcl::PointXYZI> & center_cloud,
    const pcl::PointCloud<pcl::PointXYZI> & side_cloud,
    const rclcpp::Time & center_stamp,
    const rclcpp::Time & side_stamp,
    const RigidTransform & t_center_side,
    double candidate_offset_sec) const;

  void publish_offset_diagnostics_locked(const rclcpp::Time & stamp);

  void try_publish_fusion_locked(const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg);

  static RigidTransform rigid_from_xyz_rpy(double x, double y, double z, double roll, double pitch, double yaw);

  mutable std::mutex mutex_;

  std::deque<ImuSample> imu_buffer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_ne_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_nw_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cc_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ne_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_nw_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ne_deskewed_cc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nw_deskewed_cc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_estimated_offsets_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_offset_scores_;

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

  int qos_depth_ = 20;
  double max_pair_dt_sec_ = 0.08;
  double max_imu_buffer_sec_ = 20.0;
  double max_imu_interp_gap_sec_ = 0.05;

  double ne_time_offset_sec_ = 0.0;
  double nw_time_offset_sec_ = 0.0;

  OffsetEstimatorConfig estimator_cfg_;
  OffsetEstimatorRuntime estimator_runtime_;

  RigidTransform t_center_ne_;
  RigidTransform t_center_nw_;
};

}  // namespace lidar_aggregator
