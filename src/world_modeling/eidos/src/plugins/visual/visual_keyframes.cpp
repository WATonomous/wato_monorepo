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

#include "eidos/plugins/visual/visual_keyframes.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

namespace
{

/// Neighbouring keyframes further apart in time than this are not used to estimate the velocity.
constexpr double kMaxNeighbourGapSec = 10.0;

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace

VisualKeyframes::VisualKeyframes(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & prefix,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node)
, name_(prefix)
{
  std::string image_topic, pose_service, descriptor_service;
  double buffer_duration = 2.0;
  node_->declare_parameter(prefix + ".cameras", std::vector<std::string>{});
  node_->declare_parameter(prefix + ".image_topic", std::string("/{camera}/image_rect_compressed"));
  node_->declare_parameter(prefix + ".buffer_duration", buffer_duration);
  node_->declare_parameter(prefix + ".frame_tolerance", frame_tolerance_);
  node_->declare_parameter(prefix + ".pose_service", std::string("/perception/map_anything/estimate_relative_pose"));
  node_->declare_parameter(
    prefix + ".descriptor_service", std::string("/perception/map_anything/compute_place_descriptors"));
  node_->declare_parameter(prefix + ".service_timeout", service_timeout_);

  node_->get_parameter(prefix + ".cameras", cameras_);
  node_->get_parameter(prefix + ".image_topic", image_topic);
  node_->get_parameter(prefix + ".buffer_duration", buffer_duration);
  node_->get_parameter(prefix + ".frame_tolerance", frame_tolerance_);
  node_->get_parameter(prefix + ".pose_service", pose_service);
  node_->get_parameter(prefix + ".descriptor_service", descriptor_service);
  node_->get_parameter(prefix + ".service_timeout", service_timeout_);
  node_->get_parameter("frames.base_link", rig_frame_);

  if (cameras_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] no cameras configured; the plugin will do nothing", name_.c_str());
  }
  buffer_.emplace(cameras_.size(), buffer_duration);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group;
  const std::string placeholder = "{camera}";
  for (std::size_t i = 0; i < cameras_.size(); ++i) {
    std::string topic = image_topic;
    const auto at = topic.find(placeholder);
    if (at != std::string::npos) topic.replace(at, placeholder.size(), cameras_[i]);
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, i](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { imageCallback(i, msg); },
      sub_opts));
  }

  // Responses are handled on their own group so a plugin thread waiting on one is never starved
  // by the image callbacks.
  client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  pose_client_ =
    node_->create_client<eidos_msgs::srv::EstimateRelativePose>(pose_service, rclcpp::ServicesQoS(), client_group_);
  descriptor_client_ = node_->create_client<eidos_msgs::srv::ComputePlaceDescriptors>(
    descriptor_service, rclcpp::ServicesQoS(), client_group_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %zu cameras on %s, pose service %s",
    name_.c_str(),
    cameras_.size(),
    image_topic.c_str(),
    pose_service.c_str());
}

void VisualKeyframes::activate()
{
  std::lock_guard<std::mutex> lock(mtx_);
  active_ = true;
}

void VisualKeyframes::deactivate()
{
  std::lock_guard<std::mutex> lock(mtx_);
  active_ = false;
  buffer_->clear();
}

void VisualKeyframes::imageCallback(std::size_t camera, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  auto frame = std::make_shared<VisualFrame>();
  frame->frame_id = msg->header.frame_id.empty() ? cameras_[camera] : msg->header.frame_id;
  frame->stamp = toSec(msg->header.stamp);
  frame->format = msg->format;
  frame->data = msg->data;

  std::lock_guard<std::mutex> lock(mtx_);
  if (!active_) return;
  // A stamp far behind the newest means the clock restarted (a bag was replayed).
  const auto newest = buffer_->newest();
  if (newest && frame->stamp < *newest - 1.0) buffer_->clear();
  buffer_->push(camera, std::move(frame));
}

bool VisualKeyframes::covers(double t)
{
  std::lock_guard<std::mutex> lock(mtx_);
  return buffer_->covers(t);
}

std::optional<double> VisualKeyframes::newestStamp()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return buffer_->newest();
}

std::optional<VisualFrameSet> VisualKeyframes::framesNear(double t)
{
  std::lock_guard<std::mutex> lock(mtx_);
  return buffer_->nearest(t, frame_tolerance_);
}

std::optional<VisualFrameSet> VisualKeyframes::latestFrames()
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto common = buffer_->newestCommon();
  if (!common) return std::nullopt;
  return buffer_->nearest(*common, frame_tolerance_);
}

bool VisualKeyframes::servicesReady(bool need_descriptors) const
{
  return pose_client_->service_is_ready() && (!need_descriptors || descriptor_client_->service_is_ready());
}

void VisualKeyframes::fill(std::vector<sensor_msgs::msg::CompressedImage> & out, const VisualFrameSet & set)
{
  out.reserve(set.frames.size());
  for (const auto & frame : set.frames) {
    sensor_msgs::msg::CompressedImage img;
    img.header.frame_id = frame->frame_id;
    img.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->stamp * 1e9), RCL_ROS_TIME);
    img.format = frame->format;
    img.data = frame->data;
    out.push_back(std::move(img));
  }
}

std::optional<VisualPoseEstimate> VisualKeyframes::estimateRelativePose(
  const VisualFrameSet & a,
  const VisualFrameSet & b,
  std::string & error,
  const VisualCloudRequest & cloud,
  const VisualDepth & depth)
{
  if (!pose_client_->service_is_ready()) {
    error = "pose service not available";
    return std::nullopt;
  }
  auto request = std::make_shared<eidos_msgs::srv::EstimateRelativePose::Request>();
  fill(request->images_a, a);
  fill(request->images_b, b);
  request->rig_frame = rig_frame_;
  request->depth_a = depth.a;
  request->depth_b = depth.b;
  request->return_cloud = cloud.enabled;
  request->cloud_voxel = cloud.voxel;
  request->cloud_max_range = cloud.max_range;

  const auto start = std::chrono::steady_clock::now();
  auto future = pose_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_)) != std::future_status::ready) {
    pose_client_->remove_pending_request(future);
    error = "pose service timed out";
    return std::nullopt;
  }
  const auto response = future.get();
  if (!response->success) {
    error = response->message;
    return std::nullopt;
  }
  const auto & p = response->relative_pose.position;
  const auto & q = response->relative_pose.orientation;
  VisualPoseEstimate estimate;
  estimate.a_from_b = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
  estimate.trans_spread = response->trans_spread;
  estimate.rot_spread = response->rot_spread;
  estimate.confidence = response->confidence;
  estimate.cameras = response->cameras;
  estimate.duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (cloud.enabled) {
    estimate.cloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(std::move(response->cloud));
  }
  return estimate;
}

std::optional<std::vector<float>> VisualKeyframes::describe(
  const VisualFrameSet & set, std::size_t & dim, std::string & error)
{
  if (!descriptor_client_->service_is_ready()) {
    error = "descriptor service not available";
    return std::nullopt;
  }
  auto request = std::make_shared<eidos_msgs::srv::ComputePlaceDescriptors::Request>();
  fill(request->images, set);
  auto future = descriptor_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_)) != std::future_status::ready) {
    descriptor_client_->remove_pending_request(future);
    error = "descriptor service timed out";
    return std::nullopt;
  }
  const auto response = future.get();
  if (!response->success || response->dim == 0) {
    error = response->success ? "empty descriptors" : response->message;
    return std::nullopt;
  }
  dim = response->dim;
  return response->descriptors;
}

gtsam::Pose3 motionSinceKeyframe(const MapManager & map_manager, gtsam::Key key, double image_stamp)
{
  const auto poses = map_manager.getKeyPoses6D();
  const int idx = map_manager.getCloudIndex(key);
  const int n = static_cast<int>(poses->size());
  if (idx < 0 || idx >= n) return gtsam::Pose3();
  const double t_key = poses->points[idx].time;

  // Neighbours in time (insertion order is time order within one run; a prior map and the current
  // run are separated by the gap check).
  auto usable = [&](int i) {
    return i >= 0 && i < n && i != idx && std::abs(poses->points[i].time - t_key) <= kMaxNeighbourGapSec &&
           map_manager.isPriorMapKey(map_manager.getKeyFromCloudIndex(i)) == map_manager.isPriorMapKey(key);
  };
  const int before = usable(idx - 1) ? idx - 1 : idx;
  const int after = usable(idx + 1) ? idx + 1 : idx;
  if (before == after) return gtsam::Pose3();
  return constantVelocityMotion(
    poseTypeToGtsamPose3(poses->points[before]),
    poses->points[before].time,
    poseTypeToGtsamPose3(poses->points[after]),
    poses->points[after].time,
    image_stamp - t_key);
}

}  // namespace eidos
