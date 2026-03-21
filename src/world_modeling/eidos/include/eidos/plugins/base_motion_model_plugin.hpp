#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace eidos {

// Forward declaration
class SlamCore;

/**
 * @brief Base class for motion model plugins that generate constraints
 * between consecutive states in the pose graph.
 *
 * A motion model connects states at different timestamps by providing
 * inter-state factors (e.g., ImuFactor for preintegrated IMU, or
 * BetweenFactor<Pose3> for a simple holonomic model).
 */
class MotionModelPlugin {
public:
  virtual ~MotionModelPlugin() = default;

  const std::string& getName() const { return name_; }

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

    // TF broadcasting: motion model can opt-in via motion_model.publish_tf
    node_->declare_parameter(name_ + ".publish_tf", false);
    node_->get_parameter(name_ + ".publish_tf", publish_tf_);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    onInitialize();
  }

  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void reset() = 0;

  /**
   * @brief Generate motion model constraints between two consecutive states.
   * @param key_begin GTSAM key of the starting state.
   * @param t_begin Timestamp of the starting state.
   * @param key_end GTSAM key of the ending state.
   * @param t_end Timestamp of the ending state.
   * @param factors Output: factors to add to the graph.
   * @param values Output: initial values for new variables (e.g., V, B keys).
   */
  virtual void generateMotionModel(
      gtsam::Key key_begin, double t_begin,
      gtsam::Key key_end, double t_end,
      gtsam::NonlinearFactorGraph& factors,
      gtsam::Values& values) = 0;

  /**
   * @brief Called when SlamCore creates state 0 (initialization).
   * Motion model can add priors for its variables (e.g., V(0), B(0) for IMU).
   */
  virtual void onStateZero(
      gtsam::Key key, double timestamp,
      const gtsam::Pose3& initial_pose,
      gtsam::NonlinearFactorGraph& factors,
      gtsam::Values& values) {
    (void)key; (void)timestamp; (void)initial_pose;
    (void)factors; (void)values;
  }

  /**
   * @brief Whether the motion model is ready (has enough sensor data).
   */
  virtual bool isReady() const { return true; }
  virtual std::string getReadyStatus() const { return ""; }

  /**
   * @brief Get the current real-time pose estimate from the motion model.
   * Used for displacement checks and processFrame-style pose estimation.
   */
  virtual std::optional<gtsam::Pose3> getCurrentPose() const { return std::nullopt; }

  /**
   * @brief Get gravity-aligned initial orientation from sensor data collected during warmup.
   * Returns Rot3 with roll/pitch from gravity reference, yaw=0 (heading set elsewhere).
   */
  virtual std::optional<gtsam::Rot3> getInitialOrientation() const { return std::nullopt; }

  bool publishesTf() const { return publish_tf_; }

  /**
   * @brief Returns the (frame_id, child_frame_id) pair this plugin broadcasts.
   * Override in plugins that broadcast TF. Used by SlamCore to detect collisions.
   */
  virtual std::pair<std::string, std::string> getTfFrames() const { return {"", ""}; }

protected:
  void sendTransform(const geometry_msgs::msg::TransformStamped& transform) {
    if (publish_tf_ && tf_broadcaster_) {
      tf_broadcaster_->sendTransform(transform);
    }
  }

  SlamCore* core_ = nullptr;
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

private:
  bool publish_tf_ = false;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace eidos
