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

#include "eidos/plugins/visualization/keyframe_map_visualization.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/core/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

void KeyframeMapVisualization::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/map");
  node_->declare_parameter(prefix + ".pointcloud_from", std::string("liso_factor"));
  node_->declare_parameter(prefix + ".voxel_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);
  node_->declare_parameter(prefix + ".mode", std::string("accumulate"));
  node_->declare_parameter(prefix + ".accumulate.skip_factor", 5);
  node_->declare_parameter(prefix + ".windowed.radius", 50.0);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".voxel_leaf_size", voxel_leaf_size_);
  node_->get_parameter(prefix + ".publish_rate", publish_rate_);
  node_->get_parameter(prefix + ".mode", mode_);
  node_->get_parameter(prefix + ".accumulate.skip_factor", skip_factor_);
  node_->get_parameter(prefix + ".windowed.radius", window_radius_);
  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);
  accumulated_cloud_ = pcl::make_shared<pcl::PointCloud<PointType>>();

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (mode=%s)", name_.c_str(), mode_.c_str());
}

void KeyframeMapVisualization::onActivate()
{
  pub_->on_activate();
}

void KeyframeMapVisualization::onDeactivate()
{
  pub_->on_deactivate();
}

void KeyframeMapVisualization::render(const gtsam::Values & optimized_values)
{
  if (!pub_->is_activated()) return;

  auto now = node_->now();
  if (publish_rate_ > 0.0 && (now - last_publish_time_).seconds() < 1.0 / publish_rate_) return;
  last_publish_time_ = now;

  auto key_list = map_manager_->getKeyList();
  auto poses_6d = map_manager_->getKeyPoses6D();
  if (key_list.empty()) return;

  pcl::PointCloud<PointType> output;

  if (mode_ == "accumulate") {
    // Track which keys to include (skip logic), but rebuild output
    // from body-frame clouds + current poses every render so that
    // graph optimizations (GPS, loop closure) are reflected.
    for (size_t i = 0; i < key_list.size(); i++) {
      gtsam::Key k = key_list[i];
      if (seen_keys_.count(k)) continue;
      seen_keys_.insert(k);

      if (!accepted_keys_.empty()) {
        skip_counter_++;
        if (skip_counter_ < skip_factor_) continue;  // skip this key
        skip_counter_ = 0;
      }

      // Check cloud exists before accepting
      if (!map_manager_->hasKeyframeData(k, pointcloud_from_)) continue;
      accepted_keys_.insert(k);
    }

    for (gtsam::Key k : accepted_keys_) {
      auto cloud = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(k, pointcloud_from_);
      if (!cloud || (*cloud)->empty()) continue;

      int idx = map_manager_->getCloudIndex(k);
      if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

      Eigen::Affine3f world_T = poseTypeToAffine3f(poses_6d->points[idx]);
      pcl::PointCloud<PointType> transformed;
      pcl::transformPointCloud(**cloud, transformed, world_T);
      output += transformed;
    }
  } else {
    // Window center: latest keyframe pose from optimized values
    gtsam::Key latest_key = key_list.back();
    if (!optimized_values.exists(latest_key)) return;
    auto latest_pose = optimized_values.at<gtsam::Pose3>(latest_key);
    Eigen::Vector3f current_pos(
      static_cast<float>(latest_pose.translation().x()),
      static_cast<float>(latest_pose.translation().y()),
      static_cast<float>(latest_pose.translation().z()));

    for (size_t i = 0; i < key_list.size(); i++) {
      int idx = map_manager_->getCloudIndex(key_list[i]);
      if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

      Eigen::Vector3f pos(poses_6d->points[idx].x, poses_6d->points[idx].y, poses_6d->points[idx].z);
      if ((pos - current_pos).norm() > window_radius_) continue;

      auto cloud = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(key_list[i], pointcloud_from_);
      if (!cloud || (*cloud)->empty()) continue;

      Eigen::Affine3f world_T = poseTypeToAffine3f(poses_6d->points[idx]);
      pcl::PointCloud<PointType> transformed;
      pcl::transformPointCloud(**cloud, transformed, world_T);
      output += transformed;
    }
  }

  if (output.empty()) return;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(output, msg);
  msg.header.stamp = now;
  msg.header.frame_id = map_frame_;
  pub_->publish(msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::KeyframeMapVisualization, eidos::VisualizationPlugin)
