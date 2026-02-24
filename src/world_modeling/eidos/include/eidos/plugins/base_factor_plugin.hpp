#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace eidos {

// Forward declaration
class SlamCore;

/**
 * @brief Base class for factor plugins that provide GTSAM factors from sensor data.
 *
 * Each plugin is fully self-contained: manages its own subscriptions, publishers,
 * and internal state. SlamCore does NOT broker raw data between plugins.
 */
class FactorPlugin {
public:
  virtual ~FactorPlugin() = default;

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
   * @brief Plugin creates its own subs/pubs, declares its params.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Activate the plugin (start processing).
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate the plugin (stop processing).
   */
  virtual void deactivate() = 0;

  /**
   * @brief Reset the plugin state.
   */
  virtual void reset() = 0;

  /**
   * @brief Called every SLAM cycle. Plugin processes its latest buffered data.
   * @param timestamp Current SLAM cycle timestamp.
   * @return A pose estimate if available (e.g., LiDAR scan match, IMU prediction).
   */
  virtual std::optional<gtsam::Pose3> processFrame(double timestamp) = 0;

  /**
   * @brief Called when SlamCore creates a new state.
   * @param state_index Index of the new state in the pose graph.
   * @param state_pose Pose of the new state.
   * @param timestamp Timestamp of the new state.
   * @return GTSAM factors for the pose graph.
   */
  virtual std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
      int state_index,
      const gtsam::Pose3& state_pose,
      double timestamp) = 0;

  /**
   * @brief Returns true when the plugin has received enough data to begin SLAM.
   * Override in plugins that need sensor warm-up (e.g. IMU, LiDAR).
   */
  virtual bool isReady() const { return true; }

  /**
   * @brief Returns a human-readable description of what the plugin is waiting for.
   * Empty string means the plugin is ready. Used for warmup logging.
   */
  virtual std::string getReadyStatus() const {
    return isReady() ? "" : "waiting for data";
  }

  /**
   * @brief Called after ISAM2 optimization.
   * @param optimized_values All optimized values from ISAM2.
   * @param loop_closure_detected Whether a loop closure was detected this cycle.
   */
  virtual void onOptimizationComplete(
      const gtsam::Values& optimized_values,
      bool loop_closure_detected) {
    (void)optimized_values;
    (void)loop_closure_detected;
  }

protected:
  SlamCore* core_ = nullptr;
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace eidos
