#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/nonlinear/Values.h>

namespace eidos {

// Forward declaration
class SlamCore;

/**
 * @brief Base class for read-only visualization plugins.
 *
 * Visualization plugins publish RViz-compatible messages (PointCloud2,
 * MarkerArray, etc.) derived from the current SLAM state. They do not
 * produce factors or modify the pose graph.
 */
class VisualizationPlugin {
public:
  virtual ~VisualizationPlugin() = default;

  const std::string& getName() const { return name_; }

  /**
   * @brief Framework calls this, then calls onInitialize().
   */
  void initialize(
      SlamCore* core,
      const std::string& name,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      tf2_ros::Buffer* tf,
      rclcpp::CallbackGroup::SharedPtr callback_group) {
    core_ = core;
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    onInitialize();
  }

  /**
   * @brief Plugin creates its own publishers and declares its params.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Activate the plugin (start publishing).
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate the plugin (stop publishing).
   */
  virtual void deactivate() = 0;

  /**
   * @brief Called after factor plugins' onOptimizationComplete().
   * @param optimized_values All optimized values from ISAM2.
   * @param loop_closure_detected Whether a loop closure was detected this cycle.
   */
  virtual void onOptimizationComplete(
      const gtsam::Values& optimized_values,
      bool loop_closure_detected) = 0;

protected:
  SlamCore* core_ = nullptr;
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace eidos
