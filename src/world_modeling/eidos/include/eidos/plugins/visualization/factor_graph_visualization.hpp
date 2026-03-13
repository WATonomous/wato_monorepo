#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"

namespace eidos {

class FactorGraphVisualization : public VisualizationPlugin {
public:
  FactorGraphVisualization() = default;
  ~FactorGraphVisualization() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  bool active_ = false;
  std::string map_frame_;
  double state_scale_;
  double line_width_;
  double publish_rate_ = 1.0;
  std::string mode_ = "full";       // "full" or "windowed"
  double window_radius_ = 50.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace eidos
