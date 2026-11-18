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

#include <tf2_ros/buffer.h>

#include <deque>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "eidos/utils/visual_frames.hpp"

namespace eidos
{

/**
 * @brief Lidar points as metric depth for camera images, for the multi-view model.
 *
 * Keeps the recent scans of a spinning lidar and the vehicle's body twist from an odometry topic
 * (eidos' own, e.g. ImuFactor's). For each image of a frame set, the points the lidar fired within
 * `window` of the image's time are moved to that time (per-row firing times, constant twist) and
 * into the camera's optical frame; the pose service turns them into a sparse depth map.
 *
 * Extrinsics come from TF: base_link <- lidar from the scans' frame_id, camera <- base_link from the
 * images' frame_id.
 */
class LidarDepth
{
public:
  /// What one call produced, for logging.
  struct Stats
  {
    std::size_t points = 0;  ///< over all images
    std::size_t images_without_points = 0;
    bool deskewed = true;  ///< false if no twist was known near an image's time
  };

  /**
   * @brief Declare the parameters under `<prefix>.depth.` and subscribe.
   * @param node Lifecycle node of the plugin.
   * @param prefix Plugin instance name.
   * @param callback_group Callback group for the subscriptions.
   * @param tf TF buffer for the extrinsics.
   */
  LidarDepth(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & prefix,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    tf2_ros::Buffer * tf);

  /// @brief Start or stop keeping scans (stopping drops them).
  void setActive(bool active);

  /// @brief Whether scans up to `window` past t have arrived, so every point near t is known.
  bool covers(double t);

  /**
   * @brief One cloud per image of the set, in that image's camera frame at its time.
   * @param set Frame set.
   * @param time_offset Image taken at its stamp + time_offset (s).
   * @param stats Filled with what was produced.
   */
  std::vector<sensor_msgs::msg::PointCloud2> clouds(const VisualFrameSet & set, double time_offset, Stats & stats);

private:
  struct Twist
  {
    double t;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
  };

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  std::optional<Eigen::Isometry3d> lookup(const std::string & target, const std::string & source);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  tf2_ros::Buffer * tf_;
  std::string base_frame_;
  double sweep_period_ = 0.05;
  double window_ = 0.03;
  double min_range_ = 1.0;
  double max_range_ = 60.0;
  double buffer_duration_ = 2.0;

  std::mutex mtx_;
  bool active_ = false;
  std::deque<std::pair<double, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> scans_;
  std::deque<Twist> twists_;
  std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> extrinsics_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace eidos
