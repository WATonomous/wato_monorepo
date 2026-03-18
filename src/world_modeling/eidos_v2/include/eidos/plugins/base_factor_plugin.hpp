#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include "eidos/utils/lock_free_pose.hpp"
#include "eidos/utils/types.hpp"

namespace eidos {

// Forward declarations
class MapManager;

/**
 * @brief Base class for factor plugins that produce GTSAM factors from sensor data.
 *
 * Each plugin is self-contained: manages its own subscriptions, publishers, and
 * internal state. EidosNode does NOT broker raw data between plugins.
 *
 * Lock-free pose outputs are enforced by the base class. Subclasses call
 * setMapPose() / setOdomPose() from their sensor callbacks; consumers call
 * getMapPose() / getOdomPose() from any thread without blocking.
 */
class FactorPlugin {
public:
  virtual ~FactorPlugin() = default;

  const std::string& getName() const { return name_; }

  /**
   * @brief Framework calls this once, then calls onInitialize().
   */
  void initialize(
      const std::string& name,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      tf2_ros::Buffer* tf,
      rclcpp::CallbackGroup::SharedPtr callback_group,
      MapManager* map_manager,
      const std::string& mode,
      const LockFreePose* estimator_pose,
      const std::atomic<SlamState>* state) {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    map_manager_ = map_manager;
    mode_ = mode;
    estimator_pose_ = estimator_pose;
    state_ = state;
    onInitialize();
  }

  // ---- Lifecycle (pure virtual) ----
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /**
   * @brief Called every SLAM tick. Return factors to create a new state.
   * Set result.timestamp to the sensor timestamp of the new state.
   * Return empty if no new data available.
   *
   * Only state-creating plugins (e.g. LISO) override this.
   *
   * @param key GTSAM key allocated for this potential new state.
   * @param timestamp Current SLAM cycle timestamp.
   */
  virtual StampedFactorResult produceFactor(
      gtsam::Key /*key*/, double /*timestamp*/) { return {}; }

  /**
   * @brief Called after a new state is created, to attach additional factors.
   * Return factors that reference the given key (e.g. GPS unary factor).
   * Return empty if nothing to attach.
   *
   * Latching plugins (e.g. GPS, loop closure) override this.
   *
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state.
   */
  virtual StampedFactorResult latchFactor(
      gtsam::Key /*key*/, double /*timestamp*/) { return {}; }

  /**
   * @brief Called when EidosNode transitions to TRACKING after relocalization.
   * Use to initialize against a prior map (e.g. build initial submap).
   */
  virtual void onTrackingBegin(const gtsam::Pose3& /*pose*/) {}

  /**
   * @brief Called after ISAM2 optimization with corrected values.
   * Use to update internal state to match corrected poses (e.g. LISO
   * re-anchors its GICP initial guess and triggers submap rebuild).
   */
  virtual void onOptimizationComplete(
      const gtsam::Values& /*values*/, bool /*loop_closure*/) {}

  // ---- Lock-free pose outputs (base class enforced) ----

  /// Map-frame pose (e.g. GICP match result). Always non-blocking.
  std::optional<gtsam::Pose3> getMapPose() const { return map_pose_.load(); }

  /// Odom-frame pose (e.g. incremental scan-to-scan odometry). Always non-blocking.
  std::optional<gtsam::Pose3> getOdomPose() const { return odom_pose_.load(); }

protected:
  /// Write map-frame pose from sensor callback. Single writer only.
  void setMapPose(const gtsam::Pose3& pose) { map_pose_.store(pose); }

  /// Write odom-frame pose from sensor callback. Single writer only.
  void setOdomPose(const gtsam::Pose3& pose) { odom_pose_.store(pose); }

  std::string name_;                              ///< Plugin instance name (from YAML, e.g. "liso_factor")
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_; ///< ROS2 node handle for creating subs/pubs/params
  tf2_ros::Buffer* tf_ = nullptr;                   ///< TF lookup buffer (read-only, for extrinsics etc.)
  rclcpp::CallbackGroup::SharedPtr callback_group_; ///< Dedicated callback group for this plugin's subs
  MapManager* map_manager_ = nullptr;               ///< Keyframe + global data store (direct access)
  std::string mode_;                                ///< "slam" or "localization", set once at initialize()
  const LockFreePose* estimator_pose_ = nullptr;    ///< Lock-free read of Estimator's latest optimized pose
  const std::atomic<SlamState>* state_ = nullptr;   ///< Lock-free read of EidosNode's current state

private:
  LockFreePose map_pose_;
  LockFreePose odom_pose_;
};

}  // namespace eidos
