#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"

namespace eidos {

class GpsVisualization : public VisualizationPlugin {
public:
  GpsVisualization() = default;
  ~GpsVisualization() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  bool active_ = false;
  std::string gps_from_;
  std::string map_frame_;
  double marker_scale_;
};

}  // namespace eidos
