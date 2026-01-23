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

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lidar_lidar_calib
{

LidarLidarCalibNode::LidarLidarCalibNode(const rclcpp::NodeOptions & options)
: Node("lidar_lidar_calib", options)
, best_error_(std::numeric_limits<double>::infinity())
{
  configure();

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Setup message filters for synchronized subscription
  source_sub_.subscribe(this, "source_pointcloud");
  target_sub_.subscribe(this, "target_pointcloud");

  sync_ =
    std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), source_sub_, target_sub_);
  sync_->registerCallback(
    std::bind(&LidarLidarCalibNode::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "LidarLidarCalibNode initialized");
}

void LidarLidarCalibNode::configure()
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

  // Declare frame ID parameters
  this->declare_parameter<std::string>("target_frame", "lidar_target");
  this->declare_parameter<std::string>("source_frame", "lidar_source");

  // Get parameters (assign to member variables, not local variables!)
  queue_size_ = this->get_parameter("queue_size").as_int();

  // Get GICP parameters
  num_threads_ = this->get_parameter("num_threads").as_int();
  downsampling_resolution_ = this->get_parameter("downsampling_resolution").as_double();
  max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
  max_iterations_ = this->get_parameter("max_iterations").as_int();

  // Get initial guess parameters
  tx_ = this->get_parameter("initial_translation_x").as_double();
  ty_ = this->get_parameter("initial_translation_y").as_double();
  tz_ = this->get_parameter("initial_translation_z").as_double();
  roll_ = this->get_parameter("initial_rotation_roll").as_double();
  pitch_ = this->get_parameter("initial_rotation_pitch").as_double();
  yaw_ = this->get_parameter("initial_rotation_yaw").as_double();

  // Get frame IDs
  target_frame_ = this->get_parameter("target_frame").as_string();
  source_frame_ = this->get_parameter("source_frame").as_string();
}

small_gicp::RegistrationResult LidarLidarCalibNode::perform_registration(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg)
{
  // Convert to small_gicp format
  auto source_cloud = convert_to_small_gicp(source_msg);
  auto target_cloud = convert_to_small_gicp(target_msg);

  if (source_cloud->empty() || target_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty after filtering");
    // Return a failed result
    small_gicp::RegistrationResult failed_result;
    failed_result.converged = false;
    failed_result.error = std::numeric_limits<double>::infinity();
    return failed_result;
  }

  // Construct initial guess transformation
  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd rollAngle(roll_, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_, Eigen::Vector3d::UnitZ());
  initial_guess.rotate(yawAngle * pitchAngle * rollAngle);
  initial_guess.translation() = Eigen::Vector3d(tx_, ty_, tz_);

  // Perform ICP registration using small_gicp
  // Configure registration parameters
  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads_;
  setting.downsampling_resolution = downsampling_resolution_;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.max_iterations = max_iterations_;

  // Perform registration and return
  return small_gicp::align(target_cloud->points, source_cloud->points, initial_guess, setting);
}

void LidarLidarCalibNode::log_results()
{
  RCLCPP_INFO(this->get_logger(), "========== NEW BEST Registration Result ==========");
  RCLCPP_INFO(this->get_logger(), "Error: %.6f", best_error_);
  RCLCPP_INFO(
    this->get_logger(),
    "Translation [x, y, z]: [%.6f %.6f %.6f]",
    translation_.x(),
    translation_.y(),
    translation_.z());
  RCLCPP_INFO(
    this->get_logger(),
    "Rotation (Euler) [roll, pitch, yaw]: [%.6f %.6f %.6f] rad",
    euler_.x(),
    euler_.y(),
    euler_.z());
  RCLCPP_INFO(
    this->get_logger(),
    "Rotation (Euler) [roll, pitch, yaw]: [%.4f %.4f %.4f] deg",
    euler_.x() * 180.0 / M_PI,
    euler_.y() * 180.0 / M_PI,
    euler_.z() * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "Transformation Matrix:");
  RCLCPP_INFO(
    this->get_logger(),
    "[%.6f %.6f %.6f %.6f]",
    transform_.matrix()(0, 0),
    transform_.matrix()(0, 1),
    transform_.matrix()(0, 2),
    transform_.matrix()(0, 3));
  RCLCPP_INFO(
    this->get_logger(),
    "[%.6f %.6f %.6f %.6f]",
    transform_.matrix()(1, 0),
    transform_.matrix()(1, 1),
    transform_.matrix()(1, 2),
    transform_.matrix()(1, 3));
  RCLCPP_INFO(
    this->get_logger(),
    "[%.6f %.6f %.6f %.6f]",
    transform_.matrix()(2, 0),
    transform_.matrix()(2, 1),
    transform_.matrix()(2, 2),
    transform_.matrix()(2, 3));
  RCLCPP_INFO(
    this->get_logger(),
    "[%.6f %.6f %.6f %.6f]",
    transform_.matrix()(3, 0),
    transform_.matrix()(3, 1),
    transform_.matrix()(3, 2),
    transform_.matrix()(3, 3));
  RCLCPP_INFO(this->get_logger(), "============================================");
}

void LidarLidarCalibNode::publish_transform()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = target_frame_;
  transform_stamped.child_frame_id = source_frame_;

  // Set translation
  transform_stamped.transform.translation.x = translation_.x();
  transform_stamped.transform.translation.y = translation_.y();
  transform_stamped.transform.translation.z = translation_.z();

  // Convert rotation matrix to quaternion and normalize
  Eigen::Quaterniond quat(rotation_);
  quat.normalize();
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();

  RCLCPP_INFO(
    this->get_logger(), "Quaternion [x, y, z, w]: [%.6f %.6f %.6f %.6f]", quat.x(), quat.y(), quat.z(), quat.w());

  // Broadcast the static transform
  tf_broadcaster_->sendTransform(transform_stamped);

  RCLCPP_INFO(
    this->get_logger(),
    "Published static transform from '%s' to '%s' at time %.3f",
    target_frame_.c_str(),
    source_frame_.c_str(),
    transform_stamped.header.stamp.sec + transform_stamped.header.stamp.nanosec * 1e-9);
}

std::shared_ptr<small_gicp::PointCloud> LidarLidarCalibNode::convert_to_small_gicp(
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

void LidarLidarCalibNode::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg)
{
  try {
    auto result = perform_registration(source_msg, target_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      200,
      "After filtering - Source: %d points, Target: %d points (converged: %s, error: %.6f)",
      static_cast<int>(source_msg->width * source_msg->height),
      static_cast<int>(target_msg->width * target_msg->height),
      result.converged ? "true" : "false",
      result.error);

    // Only process and publish if error is better than previous best
    if (result.converged && result.error < best_error_) {
      // Update best error
      best_error_ = result.error;

      // Store the transformation data in member variables
      transform_ = result.T_target_source;
      rotation_ = transform_.rotation();
      translation_ = transform_.translation();

      // Convert rotation to Euler angles (roll, pitch, yaw)
      euler_ = rotation_.eulerAngles(0, 1, 2);

      log_results();
      publish_transform();
    } else if (!result.converged) {
      RCLCPP_WARN(this->get_logger(), "Registration did not converge (error: %.6f)", result.error);
    } else {
      RCLCPP_DEBUG(
        this->get_logger(), "Skipping result - error %.6f is not better than best %.6f", result.error, best_error_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error during registration: %s", e.what());
  }
}

}  // namespace lidar_lidar_calib

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_lidar_calib::LidarLidarCalibNode)
