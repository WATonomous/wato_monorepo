#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"

namespace eidos {

class LoopClosureVisualization : public VisualizationPlugin {
public:
  LoopClosureVisualization() = default;
  ~LoopClosureVisualization() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  bool active_ = false;
  std::string loop_closure_from_;
  std::string map_frame_ = "map";
  double line_width_ = 0.1;
};

}  // namespace eidos
