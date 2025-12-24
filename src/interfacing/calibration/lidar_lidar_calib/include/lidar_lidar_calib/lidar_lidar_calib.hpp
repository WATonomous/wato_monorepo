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
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Forward declare small_gicp types
namespace small_gicp
{
class PointCloud;
struct RegistrationResult;
}  // namespace small_gicp

namespace lidar_lidar_calib
{

class LidarLidarCalibNode : public rclcpp::Node
{
public:
  explicit LidarLidarCalibNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Loads parameters
   */
  void configure();

  /**
   * @brief Takes pointclouds and performs GICP registration
   */
  small_gicp::RegistrationResult perform_registration(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg);

  /**
   * @brief Logs the results of the GICP registration
   */
  void log_results();

  /**
   * @brief Publishes the transform of the best gicp registration
   */
  void publish_transform();

  /**
   * @brief Helper function to convert PointCloud2 to small_gicp pointcloud
   */
  std::shared_ptr<small_gicp::PointCloud> convert_to_small_gicp(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  /**
   * @brief Takes in source and target pointclouds, publishes a transform.
   *
   * This is the main hot-loop.
   *
   * @param source_msg the source pointcloud, the pointcloud we start with
   * @param target_msg the target pointcloud, the pointcloud we are aiming to match with
   */
  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg);

  // Message filters for synchronized pointcloud input
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> source_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> target_sub_;

  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TF broadcaster for publishing calibration result
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  // Track best calibration error
  double best_error_;

  // Parameters
  int queue_size_;

  int num_threads_;
  double downsampling_resolution_;
  double max_correspondence_distance_;
  int max_iterations_;

  double tx_;
  double ty_;
  double tz_;
  double roll_;
  double pitch_;
  double yaw_;

  std::string target_frame_;
  std::string source_frame_;

  // Current registration result data
  Eigen::Isometry3d transform_;
  Eigen::Matrix3d rotation_;
  Eigen::Vector3d translation_;
  Eigen::Vector3d euler_;
};

}  // namespace lidar_lidar_calib
