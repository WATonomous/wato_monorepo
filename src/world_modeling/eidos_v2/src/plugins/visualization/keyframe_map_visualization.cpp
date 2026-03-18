#include "eidos/plugins/visualization/keyframe_map_visualization.hpp"
#include "eidos/utils/conversions.hpp"
#include "eidos/core/map_manager.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace eidos {

void KeyframeMapVisualization::onInitialize() {
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

void KeyframeMapVisualization::onActivate() {
  pub_->on_activate();
}

void KeyframeMapVisualization::onDeactivate() {
  pub_->on_deactivate();
}

void KeyframeMapVisualization::render(const gtsam::Values& optimized_values) {
  if (!pub_->is_activated()) return;

  auto now = node_->now();
  if (publish_rate_ > 0.0 && (now - last_publish_time_).seconds() < 1.0 / publish_rate_) return;
  last_publish_time_ = now;

  auto key_list = map_manager_->getKeyList();
  auto poses_6d = map_manager_->getKeyPoses6D();
  if (key_list.empty()) return;

  pcl::PointCloud<PointType> output;

  if (mode_ == "accumulate") {
    for (size_t i = 0; i < key_list.size(); i++) {
      gtsam::Key k = key_list[i];
      if (appended_keys_.count(k)) continue;

      skip_counter_++;
      if (skip_counter_ < skip_factor_) continue;
      skip_counter_ = 0;

      auto cloud_data = map_manager_->getKeyframeData(k, pointcloud_from_ + "/cloud");
      if (!cloud_data.has_value()) continue;

      pcl::PointCloud<PointType>::Ptr body_cloud;
      try {
        body_cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(cloud_data.value());
      } catch (const std::bad_any_cast&) { continue; }
      if (!body_cloud || body_cloud->empty()) continue;

      int idx = map_manager_->getCloudIndex(k);
      if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

      Eigen::Affine3f world_T = poseTypeToAffine3f(poses_6d->points[idx]);
      pcl::PointCloud<PointType> transformed;
      pcl::transformPointCloud(*body_cloud, transformed, world_T);
      *accumulated_cloud_ += transformed;
      appended_keys_.insert(k);
    }
    output = *accumulated_cloud_;
  } else {
    gtsam::Key latest_key = key_list.back();
    if (!optimized_values.exists(latest_key)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "[%s] windowed: latest key not in optimized values (keys=%zu, values=%zu)",
          name_.c_str(), key_list.size(), optimized_values.size());
      return;
    }
    auto current = optimized_values.at<gtsam::Pose3>(latest_key);
    Eigen::Vector3f current_pos(
        static_cast<float>(current.translation().x()),
        static_cast<float>(current.translation().y()),
        static_cast<float>(current.translation().z()));

    for (size_t i = 0; i < key_list.size(); i++) {
      int idx = map_manager_->getCloudIndex(key_list[i]);
      if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

      Eigen::Vector3f pos(poses_6d->points[idx].x, poses_6d->points[idx].y, poses_6d->points[idx].z);
      if ((pos - current_pos).norm() > window_radius_) continue;

      auto cloud_data = map_manager_->getKeyframeData(key_list[i], pointcloud_from_ + "/cloud");
      if (!cloud_data.has_value()) continue;

      pcl::PointCloud<PointType>::Ptr body_cloud;
      try {
        body_cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(cloud_data.value());
      } catch (const std::bad_any_cast&) { continue; }
      if (!body_cloud || body_cloud->empty()) continue;

      Eigen::Affine3f world_T = poseTypeToAffine3f(poses_6d->points[idx]);
      pcl::PointCloud<PointType> transformed;
      pcl::transformPointCloud(*body_cloud, transformed, world_T);
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
