#include "eidos/plugins/visualization/keyframe_map_visualization.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "eidos/slam_core.hpp"

namespace eidos {

void KeyframeMapVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/map");
  node_->declare_parameter(prefix + ".pointcloud_from", "lidar_kep_factor");
  node_->declare_parameter(prefix + ".voxel_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".publish_rate", 1.0);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".voxel_leaf_size", voxel_leaf_size_);
  node_->get_parameter(prefix + ".publish_rate", publish_rate_);

  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void KeyframeMapVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  last_publish_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void KeyframeMapVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void KeyframeMapVisualization::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  // Rate limit
  auto now = node_->now();
  if (publish_rate_ > 0.0 &&
      (now - last_publish_time_).seconds() < 1.0 / publish_rate_) {
    return;
  }
  last_publish_time_ = now;

  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_keyframes = map_manager.numKeyframes();

  auto accumulated = pcl::make_shared<pcl::PointCloud<PointType>>();

  for (int i = 0; i < num_keyframes; i++) {
    auto plugin_data = map_manager.getKeyframeDataForPlugin(i, pointcloud_from_);
    if (plugin_data.empty()) continue;

    auto& pose = key_poses_6d->points[i];

    for (const auto& [data_key, data] : plugin_data) {
      try {
        auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        if (cloud && !cloud->empty()) {
          auto transformed = transformPointCloud(cloud, pose);
          *accumulated += *transformed;
        }
      } catch (const std::bad_any_cast&) {
        // Not a point cloud, skip
      }
    }
  }

  if (accumulated->empty()) return;

  // Voxel downsample
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.setInputCloud(accumulated);
  voxel.filter(*accumulated);

  // Publish
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*accumulated, msg);
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_;
  pub_->publish(msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::KeyframeMapVisualization, eidos::VisualizationPlugin)
