#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"

namespace eidos {

class KeyframeMapVisualization : public VisualizationPlugin {
public:
  KeyframeMapVisualization() = default;
  ~KeyframeMapVisualization() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  bool active_ = false;
  std::string pointcloud_from_;
  std::string map_frame_ = "map";
  float voxel_leaf_size_ = 0.4f;
  double publish_rate_ = 1.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace eidos
