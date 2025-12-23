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

#include "lidar_lidar_calib/lidar_lidar_calib.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace lidar_lidar_calib
{

LidarLidarCalibNode::LidarLidarCalibNode(const rclcpp::NodeOptions & options)
: Node("lidar_lidar_calib", options)
{
  // Declare topic parameters
  this->declare_parameter<int>("queue_size", 10);

  // Declare GICP parameters
  this->declare_parameter<int>("num_threads", 4);
  this->declare_parameter<double>("downsampling_resolution", 0.25);
  this->declare_parameter<double>("max_correspondence_distance", 1.0);
  this->declare_parameter<int>("max_iterations", 50);

  // Declare initial guess parameters
  this->declare_parameter<double>("initial_translation_x", 0.0);
  this->declare_parameter<double>("initial_translation_y", 0.0);
  this->declare_parameter<double>("initial_translation_z", 0.0);
  this->declare_parameter<double>("initial_rotation_roll", 0.0);
  this->declare_parameter<double>("initial_rotation_pitch", 0.0);
  this->declare_parameter<double>("initial_rotation_yaw", 0.0);

  // Get parameters
  int queue_size = this->get_parameter("queue_size").as_int();

  // Setup message filters for synchronized subscription
  source_sub_.subscribe(this, "source_pointcloud");
  target_sub_.subscribe(this, "target_pointcloud");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size), source_sub_, target_sub_);
  sync_->registerCallback(
    std::bind(&LidarLidarCalibNode::cloudCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "LidarLidarCalibNode initialized");
}

// Helper function to convert PointCloud2 to small_gicp point cloud
std::shared_ptr<small_gicp::PointCloud> convertToSmallGICP(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  auto cloud = std::make_shared<small_gicp::PointCloud>();

  // Reserve space
  cloud->resize(cloud_msg->width * cloud_msg->height);

  // Create iterators for x, y, z fields
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

  size_t valid_points = 0;
  for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
    // Skip invalid points (NaN or inf)
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    cloud->points[valid_points] = Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    valid_points++;
  }

  // Resize to actual number of valid points
  cloud->resize(valid_points);

  return cloud;
}

void LidarLidarCalibNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received synchronized pointclouds");
  RCLCPP_INFO(this->get_logger(), "Source cloud size: %d points", source_msg->width * source_msg->height);
  RCLCPP_INFO(this->get_logger(), "Target cloud size: %d points", target_msg->width * target_msg->height);

  try {
    // Convert to small_gicp format
    auto source_cloud = convertToSmallGICP(source_msg);
    auto target_cloud = convertToSmallGICP(target_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "After filtering - Source: %zu points, Target: %zu points",
      source_cloud->size(),
      target_cloud->size());

    if (source_cloud->empty() || target_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty after filtering");
      return;
    }

    // Get GICP parameters
    int num_threads = this->get_parameter("num_threads").as_int();
    double downsampling_resolution = this->get_parameter("downsampling_resolution").as_double();
    double max_correspondence_distance = this->get_parameter("max_correspondence_distance").as_double();

    // Get initial guess parameters
    double tx = this->get_parameter("initial_translation_x").as_double();
    double ty = this->get_parameter("initial_translation_y").as_double();
    double tz = this->get_parameter("initial_translation_z").as_double();
    double roll = this->get_parameter("initial_rotation_roll").as_double();
    double pitch = this->get_parameter("initial_rotation_pitch").as_double();
    double yaw = this->get_parameter("initial_rotation_yaw").as_double();

    // Construct initial guess transformation
    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    initial_guess.rotate(yawAngle * pitchAngle * rollAngle);
    initial_guess.translation() = Eigen::Vector3d(tx, ty, tz);

    RCLCPP_INFO(
      this->get_logger(),
      "Initial guess - Translation: [%.3f, %.3f, %.3f], Rotation (RPY): [%.3f, %.3f, %.3f]",
      tx,
      ty,
      tz,
      roll,
      pitch,
      yaw);

    // Perform ICP registration using small_gicp
    // Configure registration parameters
    small_gicp::RegistrationSetting setting;
    setting.num_threads = num_threads;
    setting.downsampling_resolution = downsampling_resolution;
    setting.max_correspondence_distance = max_correspondence_distance;

    // Perform registration
    auto result = small_gicp::align(target_cloud->points, source_cloud->points, initial_guess, setting);

    // Log the transformation
    Eigen::Isometry3d transform = result.T_target_source;
    Eigen::Matrix3d rotation = transform.rotation();
    Eigen::Vector3d translation = transform.translation();

    // Convert rotation to Euler angles (roll, pitch, yaw)
    Eigen::Vector3d euler = rotation.eulerAngles(0, 1, 2);

    RCLCPP_INFO(this->get_logger(), "========== ICP Registration Result ==========");
    RCLCPP_INFO(this->get_logger(), "Converged: %s", result.converged ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Iterations: %zu", result.iterations);
    RCLCPP_INFO(this->get_logger(), "Error: %.6f", result.error);
    RCLCPP_INFO(
      this->get_logger(),
      "Translation [x, y, z]: [%.6f, %.6f, %.6f]",
      translation.x(),
      translation.y(),
      translation.z());
    RCLCPP_INFO(
      this->get_logger(),
      "Rotation (Euler) [roll, pitch, yaw]: [%.6f, %.6f, %.6f] rad",
      euler.x(),
      euler.y(),
      euler.z());
    RCLCPP_INFO(
      this->get_logger(),
      "Rotation (Euler) [roll, pitch, yaw]: [%.4f, %.4f, %.4f] deg",
      euler.x() * 180.0 / M_PI,
      euler.y() * 180.0 / M_PI,
      euler.z() * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Transformation Matrix:");
    RCLCPP_INFO(
      this->get_logger(),
      "[%.6f, %.6f, %.6f, %.6f]",
      transform.matrix()(0, 0),
      transform.matrix()(0, 1),
      transform.matrix()(0, 2),
      transform.matrix()(0, 3));
    RCLCPP_INFO(
      this->get_logger(),
      "[%.6f, %.6f, %.6f, %.6f]",
      transform.matrix()(1, 0),
      transform.matrix()(1, 1),
      transform.matrix()(1, 2),
      transform.matrix()(1, 3));
    RCLCPP_INFO(
      this->get_logger(),
      "[%.6f, %.6f, %.6f, %.6f]",
      transform.matrix()(2, 0),
      transform.matrix()(2, 1),
      transform.matrix()(2, 2),
      transform.matrix()(2, 3));
    RCLCPP_INFO(
      this->get_logger(),
      "[%.6f, %.6f, %.6f, %.6f]",
      transform.matrix()(3, 0),
      transform.matrix()(3, 1),
      transform.matrix()(3, 2),
      transform.matrix()(3, 3));
    RCLCPP_INFO(this->get_logger(), "============================================");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error during registration: %s", e.what());
  }
}

}  // namespace lidar_lidar_calib

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_lidar_calib::LidarLidarCalibNode)
