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
#include <gtsam/inference/Key.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <eidos_msgs/srv/compute_place_descriptors.hpp>
#include <eidos_msgs/srv/estimate_relative_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "eidos/utils/visual_frames.hpp"

namespace eidos
{

class MapManager;

/// A relative rig pose measured from two camera frame sets by the pose service.
struct VisualPoseEstimate
{
  gtsam::Pose3 a_from_b;  ///< T_rig(a)^-1 * T_rig(b), each at its frame set's stamp
  double trans_spread = 0.0;  ///< m, disagreement between the per-camera estimates
  double rot_spread = 0.0;  ///< rad
  float confidence = 0.0f;
  uint32_t cameras = 0;
  double duration = 0.0;  ///< s, wall time of the service call
  /// Dense points of frame set b in the rig frame at b (x, y, z, rgb), if they were asked for.
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
};

/// Optional metric depth per image (e.g. lidar points from LidarDepth), one cloud per image in the
/// frame set's camera order; empty = none.
struct VisualDepth
{
  std::vector<sensor_msgs::msg::PointCloud2> a;
  std::vector<sensor_msgs::msg::PointCloud2> b;
};

/// What to ask the pose service for besides the pose.
struct VisualCloudRequest
{
  bool enabled = false;  ///< also return frame set b's dense points (slower)
  float voxel = 0.0f;  ///< m, one point per voxel (0 = every pixel)
  float max_range = 0.0f;  ///< m from the camera (0 = no limit)
};

/**
 * @brief Camera input and model access shared by the visual plugins.
 *
 * Subscribes to the compressed image topics of a set of cameras and keeps a short history, from
 * which the frame set nearest to a keyframe's time is taken. Relative poses between frame sets and
 * place descriptors come from two ROS services (perception's map_anything server), so eidos itself
 * needs neither a GPU nor an image library: images are passed through still compressed.
 *
 * estimateRelativePose() and describe() block on the service and must be called from a thread of
 * the plugin's own, never from a ROS callback or the SLAM loop.
 */
class VisualKeyframes
{
public:
  /**
   * @brief Declare the parameters under prefix and create the subscriptions and service clients.
   * @param node Lifecycle node the plugin belongs to.
   * @param prefix Parameter prefix (the plugin instance name).
   * @param callback_group Callback group for the image subscriptions.
   */
  VisualKeyframes(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & prefix,
    rclcpp::CallbackGroup::SharedPtr callback_group);

  /// @brief Start keeping images.
  void activate();

  /// @brief Stop keeping images and drop the history.
  void deactivate();

  /// @brief Whether both instants of every camera around t are known (a later frame has arrived).
  bool covers(double t);

  /// @brief Newest image stamp of any camera, or nullopt before the first image.
  std::optional<double> newestStamp();

  /// @brief Frame set nearest to t, or nullopt if a camera has no frame within frame_tolerance.
  std::optional<VisualFrameSet> framesNear(double t);

  /// @brief The most recent frame set that all cameras have reached.
  std::optional<VisualFrameSet> latestFrames();

  /// @brief Whether the pose service (and, if asked, the descriptor service) is available.
  bool servicesReady(bool need_descriptors) const;

  /**
   * @brief Rig motion between two frame sets, from the pose service. Blocks up to service_timeout.
   * @param a Frame set of the first instant.
   * @param b Frame set of the second instant.
   * @param error Set to the reason when nullopt is returned.
   * @param cloud Whether (and how) to also return frame set b's dense points.
   * @param depth Optional metric depth for the images of a and b.
   */
  std::optional<VisualPoseEstimate> estimateRelativePose(
    const VisualFrameSet & a,
    const VisualFrameSet & b,
    std::string & error,
    const VisualCloudRequest & cloud = {},
    const VisualDepth & depth = {});

  /**
   * @brief Place descriptors of a frame set (one per camera, concatenated). Blocks up to service_timeout.
   * @param set Frame set to describe.
   * @param dim Set to the length of one descriptor.
   * @param error Set to the reason when nullopt is returned.
   */
  std::optional<std::vector<float>> describe(const VisualFrameSet & set, std::size_t & dim, std::string & error);

  /// @brief Number of cameras.
  std::size_t numCameras() const
  {
    return cameras_.size();
  }

  /// @brief Frame the relative poses are expressed for (frames.base_link of the node).
  const std::string & rigFrame() const
  {
    return rig_frame_;
  }

private:
  void imageCallback(std::size_t camera, const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  static void fill(std::vector<sensor_msgs::msg::CompressedImage> & out, const VisualFrameSet & set);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::string rig_frame_;
  std::vector<std::string> cameras_;
  double frame_tolerance_ = 0.06;
  double service_timeout_ = 10.0;

  std::mutex mtx_;
  std::optional<VisualFrameBuffer> buffer_;
  bool active_ = false;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subs_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<eidos_msgs::srv::EstimateRelativePose>::SharedPtr pose_client_;
  rclcpp::Client<eidos_msgs::srv::ComputePlaceDescriptors>::SharedPtr descriptor_client_;
};

/**
 * @brief Motion of the rig from a keyframe's time to the time its images were taken.
 *
 * Camera images are not taken at the keyframe's (lidar) time. The offset is small, so the motion in
 * between is taken as constant-velocity along the neighbouring keyframes.
 *
 * @param map_manager Keyframe poses and times.
 * @param key Keyframe.
 * @param image_stamp Stamp of the keyframe's frame set (s).
 * @return T_rig(keyframe time)^-1 * T_rig(image_stamp); identity if the keyframe has no usable neighbour.
 */
gtsam::Pose3 motionSinceKeyframe(const MapManager & map_manager, gtsam::Key key, double image_stamp);

}  // namespace eidos
