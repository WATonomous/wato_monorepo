#ifndef ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
#define ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace ackermann_pure_pursuit
{

class PurePursuitNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void controlCallback();
  void trajectoryCallback(const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg);
  double getWheelbase();

  // Parameters
  std::string trajectory_topic_;
  std::string ackermann_topic_;
  std::string idle_topic_;
  std::string base_frame_;
  std::string rear_axle_frame_;
  std::string front_axle_frame_;
  double lookahead_distance_;
  double min_lookahead_distance_;
  double max_speed_;
  double min_speed_;
  double control_rate_hz_;
  double wheelbase_fallback_;
  double max_steering_angle_;
  double idle_timeout_sec_;

  // Cached wheelbase from TF (0.0 = not yet resolved)
  double wheelbase_cached_{0.0};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub/Sub
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    ackermann_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp::Subscription<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  wato_trajectory_msgs::msg::Trajectory::SharedPtr latest_trajectory_;
  rclcpp::Time last_trajectory_time_;
};

}  // namespace ackermann_pure_pursuit

#endif  // ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
