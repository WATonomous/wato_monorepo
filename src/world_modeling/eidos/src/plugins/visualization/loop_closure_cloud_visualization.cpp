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

#include "eidos/plugins/visualization/loop_closure_cloud_visualization.hpp"

#include <gtsam/inference/Symbol.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

void LoopClosureCloudVisualization::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic_source", std::string("slam/visualization/loop_closure_source"));
  node_->declare_parameter(prefix + ".topic_target", std::string("slam/visualization/loop_closure_target"));
  node_->declare_parameter(prefix + ".topic_markers", std::string("slam/visualization/loop_closure_markers"));
  node_->declare_parameter(prefix + ".loop_closure_factor_name", std::string("euclidean_distance_loop_closure_factor"));
  node_->declare_parameter(prefix + ".pointcloud_from", std::string("liso_factor/cloud"));
  node_->declare_parameter(prefix + ".publish_rate", 1.0);
  node_->declare_parameter(prefix + ".line_width", 0.5);
  node_->declare_parameter(prefix + ".dump_dir", std::string(""));

  std::string topic_source, topic_target, topic_markers;
  node_->get_parameter(prefix + ".topic_source", topic_source);
  node_->get_parameter(prefix + ".topic_target", topic_target);
  node_->get_parameter(prefix + ".topic_markers", topic_markers);
  node_->get_parameter(prefix + ".loop_closure_factor_name", loop_closure_factor_name_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".publish_rate", publish_rate_);
  node_->get_parameter(prefix + ".line_width", line_width_);
  node_->get_parameter(prefix + ".dump_dir", dump_dir_);
  node_->get_parameter("frames.map", map_frame_);

  loop_target_key_ = loop_closure_factor_name_ + "/loop_target";
  loop_initial_key_ = loop_closure_factor_name_ + "/loop_initial";
  loop_corrected_key_ = loop_closure_factor_name_ + "/loop_corrected";

  source_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_source, 1);
  target_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_target, 1);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_markers, 1);

  if (!dump_dir_.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(dump_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[%s] cannot create dump_dir '%s' (%s) — dumping disabled",
        name_.c_str(),
        dump_dir_.c_str(),
        ec.message().c_str());
      dump_dir_.clear();
    } else {
      RCLCPP_INFO(node_->get_logger(), "[%s] dumping closures to %s", name_.c_str(), dump_dir_.c_str());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (watching %s)", name_.c_str(), loop_target_key_.c_str());
}

void LoopClosureCloudVisualization::onActivate()
{
  source_pub_->on_activate();
  target_pub_->on_activate();
  marker_pub_->on_activate();
}

void LoopClosureCloudVisualization::onDeactivate()
{
  source_pub_->on_deactivate();
  target_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  dumped_keys_.clear();
  latest_source_key_.reset();
}

void LoopClosureCloudVisualization::render(const gtsam::Values & optimized_values)
{
  (void)optimized_values;
  if (!source_pub_->is_activated()) return;

  auto now = node_->now();
  if (publish_rate_ > 0.0 && (now - last_publish_time_).seconds() < 1.0 / publish_rate_) return;
  last_publish_time_ = now;

  auto key_list = map_manager_->getKeyList();
  auto poses_6d = map_manager_->getKeyPoses6D();

  // Most recent closure wins — key_list is chronological.
  for (auto it = key_list.rbegin(); it != key_list.rend(); ++it) {
    if (map_manager_->hasKeyframeData(*it, loop_target_key_)) {
      latest_source_key_ = *it;
      break;
    }
  }
  if (!latest_source_key_) return;

  gtsam::Key source_key = *latest_source_key_;
  auto target_key_opt = map_manager_->retrieve<gtsam::Key>(source_key, loop_target_key_);
  if (!target_key_opt) return;
  gtsam::Key target_key = *target_key_opt;

  auto source_cloud = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(source_key, pointcloud_from_);
  auto target_cloud = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(target_key, pointcloud_from_);
  if (!source_cloud || !*source_cloud || (*source_cloud)->empty()) return;
  if (!target_cloud || !*target_cloud || (*target_cloud)->empty()) return;

  int source_idx = map_manager_->getCloudIndex(source_key);
  int target_idx = map_manager_->getCloudIndex(target_key);
  if (source_idx < 0 || source_idx >= static_cast<int>(poses_6d->size())) return;
  if (target_idx < 0 || target_idx >= static_cast<int>(poses_6d->size())) return;

  Eigen::Affine3f source_T = poseTypeToAffine3f(poses_6d->points[source_idx]);
  Eigen::Affine3f target_T = poseTypeToAffine3f(poses_6d->points[target_idx]);

  pcl::PointCloud<PointType> source_world, target_world;
  pcl::transformPointCloud(**source_cloud, source_world, source_T);
  pcl::transformPointCloud(**target_cloud, target_world, target_T);

  sensor_msgs::msg::PointCloud2 source_msg, target_msg;
  pcl::toROSMsg(source_world, source_msg);
  pcl::toROSMsg(target_world, target_msg);
  source_msg.header.stamp = now;
  source_msg.header.frame_id = map_frame_;
  target_msg.header.stamp = now;
  target_msg.header.frame_id = map_frame_;
  source_pub_->publish(source_msg);
  target_pub_->publish(target_msg);

  // Constraint edge between the two keyframe poses.
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker line;
  line.header.stamp = now;
  line.header.frame_id = map_frame_;
  line.ns = "loop_closure";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = line_width_;
  line.color.r = 1.0f;
  line.color.g = 1.0f;
  line.color.b = 0.0f;
  line.color.a = 1.0f;
  geometry_msgs::msg::Point p1, p2;
  p1.x = poses_6d->points[source_idx].x;
  p1.y = poses_6d->points[source_idx].y;
  p1.z = poses_6d->points[source_idx].z;
  p2.x = poses_6d->points[target_idx].x;
  p2.y = poses_6d->points[target_idx].y;
  p2.z = poses_6d->points[target_idx].z;
  line.points.push_back(p1);
  line.points.push_back(p2);
  markers.markers.push_back(line);
  marker_pub_->publish(markers);

  if (dump_dir_.empty() || dumped_keys_.count(source_key)) return;

  auto initial = map_manager_->retrieve<gtsam::Pose3>(source_key, loop_initial_key_);
  auto corrected = map_manager_->retrieve<gtsam::Pose3>(source_key, loop_corrected_key_);
  if (!initial || !corrected) return;

  dumpClosure(source_key, target_key, **source_cloud, **target_cloud, *initial, *corrected);
  dumped_keys_.insert(source_key);
}

void LoopClosureCloudVisualization::dumpClosure(
  gtsam::Key source_key,
  gtsam::Key target_key,
  const pcl::PointCloud<PointType> & source_body,
  const pcl::PointCloud<PointType> & target_body,
  const gtsam::Pose3 & initial,
  const gtsam::Pose3 & corrected) const
{
  gtsam::Symbol s(source_key), t(target_key);
  std::string stem = dump_dir_ + "/closure_" + std::to_string(s.index()) + "_to_" + std::to_string(t.index());

  auto write_cloud = [](const std::string & path, const pcl::PointCloud<PointType> & cloud) {
    std::ofstream f(path);
    f << "x,y,z,intensity\n";
    for (const auto & p : cloud.points) {
      f << p.x << "," << p.y << "," << p.z << "," << p.intensity << "\n";
    }
  };
  write_cloud(stem + "_source.csv", source_body);
  write_cloud(stem + "_target.csv", target_body);

  auto write_pose = [](std::ofstream & f, const std::string & name, const gtsam::Pose3 & p) {
    f << name;
    const auto & m = p.matrix();
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) f << "," << m(r, c);
    }
    f << "\n";
  };
  std::ofstream tf(stem + "_transforms.csv");
  tf << "name,m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23,m30,m31,m32,m33\n";
  write_pose(tf, "initial", initial);
  write_pose(tf, "corrected", corrected);

  RCLCPP_INFO(node_->get_logger(), "[%s] dumped closure %s", name_.c_str(), stem.c_str());
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LoopClosureCloudVisualization, eidos::VisualizationPlugin)
