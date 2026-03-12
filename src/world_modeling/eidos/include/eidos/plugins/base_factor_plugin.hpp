#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace eidos {

// Forward declaration
class SlamCore;

/**
 * @brief Return type for getFactors(). Contains an optional sensor timestamp,
 * factors, and optional initial values for any new variables introduced by the plugin.
 *
 * If timestamp has a value, SlamCore will create a new state at that timestamp
 * and connect it via the motion model. If nullopt, the plugin contributes
 * factors but does not create a new state (e.g., loop closure).
 */
struct StampedFactorResult {
  std::optional<double> timestamp;  // sensor timestamp; nullopt = no new state
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  gtsam::Values values;  // initial values for new variables (e.g. bias keys)
};

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
   * @return A pose estimate if available (e.g., LiDAR scan match).
   */
  virtual std::optional<gtsam::Pose3> processFrame(double timestamp) = 0;

  /**
   * @brief Called when SlamCore forms a keygroup.
   * @param key GTSAM Symbol key assigned by SlamCore (Symbol(plugin_index, keygroup)).
   * @return StampedFactorResult with optional timestamp and factors.
   *         If timestamp has a value, a new state is created at that time.
   *         The plugin should use `key` directly in its GTSAM factors.
   */
  virtual StampedFactorResult getFactors(gtsam::Key key) = 0;

  /**
   * @brief Returns true when the plugin has received enough data to begin SLAM.
   * Override in plugins that need sensor warm-up (e.g. LiDAR).
   */
  virtual bool isReady() const { return true; }

  /**
   * @brief Returns true when the plugin has buffered data ready to produce factors.
   * SlamCore polls this each tick to decide whether to allocate a state.
   */
  virtual bool hasData() const { return false; }

  /**
   * @brief Returns a human-readable description of what the plugin is waiting for.
   * Empty string means the plugin is ready. Used for warmup logging.
   */
  virtual std::string getReadyStatus() const {
    return isReady() ? "" : "waiting for data";
  }

  /**
   * @brief Called after each new state is created. Allows non-state-creating
   * plugins to attach factors to states created by other plugins.
   * Default returns empty — non-latching plugins do nothing.
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state.
   * @return StampedFactorResult with factors/values (timestamp should be nullopt).
   */
  virtual StampedFactorResult latchFactors(gtsam::Key key, double timestamp) {
    (void)key; (void)timestamp;
    return {};
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
