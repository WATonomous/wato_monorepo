#pragma once

#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <gtsam/inference/Key.h>

#include "eidos/plugins/base_visualization_plugin.hpp"
#include "eidos/types.hpp"

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
  void publishAccumulate(bool loop_closure_detected);
  void publishWindowed();

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  bool active_ = false;
  std::string pointcloud_from_;
  std::string map_frame_;
  std::string mode_;           // "accumulate" or "windowed"
  float voxel_leaf_size_;
  double publish_rate_;

  // Accumulate mode params
  int skip_factor_;            // append every Nth keyframe

  // Windowed mode params
  double window_radius_;       // meters
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  // Accumulate mode state
  pcl::PointCloud<PointType>::Ptr accumulated_cloud_;
  std::set<gtsam::Key> appended_keys_;
  int skip_counter_ = 0;           // persistent counter for skip_factor across calls
};

}  // namespace eidos
