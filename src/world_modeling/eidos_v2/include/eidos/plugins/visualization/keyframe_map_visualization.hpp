#pragma once

#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <gtsam/inference/Key.h>

#include "eidos/plugins/base_visualization_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos {

/**
 * @brief Publishes accumulated or windowed keyframe point cloud map.
 */
class KeyframeMapVisualization : public VisualizationPlugin {
public:
  KeyframeMapVisualization() = default;
  ~KeyframeMapVisualization() override = default;

  void onInitialize() override;

protected:
  void onActivate() override;
  void onDeactivate() override;
  void render(const gtsam::Values& optimized_values) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string pointcloud_from_;
  std::string map_frame_;
  std::string mode_;
  float voxel_leaf_size_;
  double publish_rate_;
  int skip_factor_;
  double window_radius_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  pcl::PointCloud<PointType>::Ptr accumulated_cloud_;
  std::set<gtsam::Key> appended_keys_;
  int skip_counter_ = 0;
};

}  // namespace eidos
