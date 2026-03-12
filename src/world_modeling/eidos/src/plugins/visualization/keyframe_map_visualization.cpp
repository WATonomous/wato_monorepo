#include "eidos/plugins/visualization/keyframe_map_visualization.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "eidos/slam_core.hpp"

namespace eidos {

void KeyframeMapVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/map");
  node_->declare_parameter(prefix + ".pointcloud_from", "");
  node_->declare_parameter(prefix + ".voxel_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);
  node_->declare_parameter(prefix + ".mode", "windowed");
  node_->declare_parameter(prefix + ".accumulate.skip_factor", 1);
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

void KeyframeMapVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  last_publish_time_ = node_->now();
  accumulated_cloud_->clear();
  appended_keys_.clear();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void KeyframeMapVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void KeyframeMapVisualization::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool loop_closure_detected) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  // Rate limit
  auto now = node_->now();
  if (publish_rate_ > 0.0 &&
      (now - last_publish_time_).seconds() < 1.0 / publish_rate_) {
    return;
  }
  last_publish_time_ = now;

  if (mode_ == "accumulate") {
    publishAccumulate(loop_closure_detected);
  } else {
    publishWindowed();
  }
}

void KeyframeMapVisualization::publishAccumulate(bool /*loop_closure_detected*/) {
  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  auto key_list = map_manager.getKeyList();

  // Full rebuild every publish — poses may have shifted from GPS or loop closures
  accumulated_cloud_->clear();
  appended_keys_.clear();
  skip_counter_ = 0;

  // Append keyframes, respecting skip_factor
  for (auto gtsam_key : key_list) {
    if (appended_keys_.count(gtsam_key)) continue;
    skip_counter_ = (skip_counter_ + 1) % skip_factor_;
    if (skip_factor_ > 1 && skip_counter_ != 1) {
      appended_keys_.insert(gtsam_key);  // mark as seen but don't append
      continue;
    }

    auto plugin_data = map_manager.getKeyframeDataForPlugin(gtsam_key, pointcloud_from_);
    if (plugin_data.empty()) continue;

    int cloud_idx = map_manager.getCloudIndex(gtsam_key);
    if (cloud_idx < 0) continue;
    auto& pose = key_poses_6d->points[cloud_idx];

    for (const auto& [data_key, data] : plugin_data) {
      try {
        auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        if (cloud && !cloud->empty()) {
          auto transformed = transformPointCloud(cloud, pose);

          pcl::VoxelGrid<PointType> voxel;
          voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
          voxel.setInputCloud(transformed);
          voxel.filter(*transformed);

          *accumulated_cloud_ += *transformed;
        }
      } catch (const std::bad_any_cast&) {}
    }

    appended_keys_.insert(gtsam_key);
  }

  if (accumulated_cloud_->empty()) return;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*accumulated_cloud_, msg);
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_;
  pub_->publish(msg);
}

void KeyframeMapVisualization::publishWindowed() {
  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  auto key_list = map_manager.getKeyList();
  if (key_list.empty()) return;

  // Get current pose as window center
  gtsam::Pose3 current = core_->getCurrentPose();
  Eigen::Vector3f center(
      static_cast<float>(current.translation().x()),
      static_cast<float>(current.translation().y()),
      static_cast<float>(current.translation().z()));
  float r2 = static_cast<float>(window_radius_ * window_radius_);

  auto windowed = pcl::make_shared<pcl::PointCloud<PointType>>();

  for (auto gtsam_key : key_list) {
    int cloud_idx = map_manager.getCloudIndex(gtsam_key);
    if (cloud_idx < 0) continue;
    auto& pose = key_poses_6d->points[cloud_idx];

    // Distance check from window center
    Eigen::Vector3f kf_pos(pose.x, pose.y, pose.z);
    if ((kf_pos - center).squaredNorm() > r2) continue;

    auto plugin_data = map_manager.getKeyframeDataForPlugin(gtsam_key, pointcloud_from_);
    for (const auto& [data_key, data] : plugin_data) {
      try {
        auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        if (cloud && !cloud->empty()) {
          auto transformed = transformPointCloud(cloud, pose);
          *windowed += *transformed;
        }
      } catch (const std::bad_any_cast&) {}
    }
  }

  if (windowed->empty()) return;

  // Downsample
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.setInputCloud(windowed);
  voxel.filter(*windowed);

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*windowed, msg);
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_;
  pub_->publish(msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::KeyframeMapVisualization, eidos::VisualizationPlugin)
