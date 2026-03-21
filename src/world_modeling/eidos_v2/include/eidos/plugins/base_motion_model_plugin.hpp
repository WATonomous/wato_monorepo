#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>

#include "eidos/utils/lock_free_pose.hpp"
#include "eidos/utils/types.hpp"

namespace eidos {

/**
 * @brief Base class for motion model plugins.
 *
 * The motion model is purely a high-rate odom filler. It reads a high-rate
 * sensor (e.g. IMU), integrates, and writes its odom-frame pose via
 * setOdomPose(). TransformManager reads this for smooth odom→base TF.
 *
 * The motion model does NOT contribute factors to the SLAM graph.
 */
class MotionModelPlugin {
public:
  virtual ~MotionModelPlugin() = default;

  const std::string& getName() const { return name_; }

  void initialize(
      const std::string& name,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      tf2_ros::Buffer* tf,
      rclcpp::CallbackGroup::SharedPtr callback_group,
      const std::atomic<SlamState>* state) {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    state_ = state;
    onInitialize();
  }

  // ---- Lifecycle ----
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /// Whether the motion model has enough data to begin (e.g. IMU warmup).
  virtual bool isReady() const { return true; }
  virtual std::string getReadyStatus() const { return ""; }

  // ---- Lock-free pose output (base class enforced) ----

  /// Odom-frame pose. Always non-blocking.
  std::optional<gtsam::Pose3> getOdomPose() const { return odom_pose_.load(); }

protected:
  /// Write odom-frame pose from sensor callback. Single writer only.
  void setOdomPose(const gtsam::Pose3& pose) { odom_pose_.store(pose); }

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  const std::atomic<SlamState>* state_ = nullptr;

private:
  LockFreePose odom_pose_;
};

}  // namespace eidos
