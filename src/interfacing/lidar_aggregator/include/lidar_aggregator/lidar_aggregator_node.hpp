// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
  };

  void declare_and_load_parameters();
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidar_cc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidar_ne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidar_nw_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool get_orientation_at_time(const rclcpp::Time & stamp, Eigen::Quaterniond & q_out) const;
  bool get_rotation_delta(
    const rclcpp::Time & from_stamp,
    const rclcpp::Time & to_stamp,
    Eigen::Matrix3d & delta_rotation) const;

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

  void try_publish_fusion_locked(const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg);

  static RigidTransform rigid_from_xyz_rpy(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw);

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

  std::string imu_topic_;
  std::string center_topic_;
  std::string ne_topic_;
  std::string nw_topic_;

  std::string deskewed_ne_output_topic_;
  std::string deskewed_nw_output_topic_;
  std::string merged_output_topic_;
  std::string center_frame_;

  int qos_depth_ = 20;
  double max_pair_dt_sec_ = 0.08;
  double max_imu_buffer_sec_ = 20.0;
  double max_imu_interp_gap_sec_ = 0.05;

  double ne_time_offset_sec_ = 0.0;
  double nw_time_offset_sec_ = 0.0;

  RigidTransform t_center_ne_;
  RigidTransform t_center_nw_;
};

}  // namespace lidar_aggregator
