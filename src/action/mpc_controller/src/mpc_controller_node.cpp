// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_controller/mpc_controller_node.hpp"

#include <cmath>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mpc_controller
{

MpcControllerNode::MpcControllerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mpc_controller_node", options)
, last_trajectory_time_(0, 0, RCL_ROS_TIME)
, current_speed_(0.0)
, prev_steering_(0.0)
, prev_accel_(0.0)
{
  // Topic params
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("bt_topic", "execute_behaviour");
  declare_parameter("ackermann_topic", "ackermann");
  declare_parameter("idle_topic", "is_idle");
  declare_parameter("predicted_path_topic", "predicted_path");
  declare_parameter("odom_topic", "odom");

  // Frame params
  declare_parameter("base_frame", "base_footprint");

  // Control params
  declare_parameter("control_rate_hz", 20.0);
  declare_parameter("wheelbase", 2.5667);
  declare_parameter("standby_msg", "standby");
  declare_parameter("standby_speed", 0.0);
  declare_parameter("standby_steering", 0.0);
  declare_parameter("idle_timeout_sec", 2.0);
  declare_parameter("invert_steering", false);
  declare_parameter("disable_standby", false);

  // MPC horizon params
  declare_parameter("horizon_distance", 25.0);
  declare_parameter("point_spacing", 1.0);
  declare_parameter("max_horizon_steps", 30);

  // MPC cost weights
  declare_parameter("w_lateral", 50.0);
  declare_parameter("w_heading", 20.0);
  declare_parameter("w_progress", 5.0);
  declare_parameter("w_steering", 1.0);
  declare_parameter("w_accel", 1.0);
  declare_parameter("w_dsteering", 100.0);
  declare_parameter("w_daccel", 50.0);
  declare_parameter("w_terminal", 5.0);

  // MPC actuator limits
  declare_parameter("max_steering_angle", 0.5);
  declare_parameter("max_accel", 2.5);
  declare_parameter("max_decel", -4.0);
  declare_parameter("max_speed", 15.0);

  // MPC comfort limits
  declare_parameter("max_steering_rate", 0.3);
  declare_parameter("max_jerk", 5.0);

  // Solver params
  declare_parameter("dt_min", 0.5);
  declare_parameter("max_solver_iterations", 200);
  declare_parameter("solver_eps_abs", 0.001);
  declare_parameter("solver_eps_rel", 0.001);
  declare_parameter("warm_start", true);
}

MpcControllerNode::CallbackReturn MpcControllerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Load topic params
  auto trajectory_topic = get_parameter("trajectory_topic").as_string();
  auto bt_topic = get_parameter("bt_topic").as_string();
  auto ackermann_topic = get_parameter("ackermann_topic").as_string();
  auto idle_topic = get_parameter("idle_topic").as_string();
  auto predicted_path_topic = get_parameter("predicted_path_topic").as_string();
  auto odom_topic = get_parameter("odom_topic").as_string();

  // Load frame params
  base_frame_ = get_parameter("base_frame").as_string();

  // Load control params
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  wheelbase_ = get_parameter("wheelbase").as_double();
  standby_msg_ = get_parameter("standby_msg").as_string();
  standby_speed_ = get_parameter("standby_speed").as_double();
  standby_steering_ = get_parameter("standby_steering").as_double();
  idle_timeout_sec_ = get_parameter("idle_timeout_sec").as_double();
  invert_steering_ = get_parameter("invert_steering").as_bool();
  disable_standby_ = get_parameter("disable_standby").as_bool();

  // Load MPC config
  config_.horizon_distance = get_parameter("horizon_distance").as_double();
  config_.point_spacing = get_parameter("point_spacing").as_double();
  config_.max_horizon_steps = get_parameter("max_horizon_steps").as_int();
  config_.w_lateral = get_parameter("w_lateral").as_double();
  config_.w_heading = get_parameter("w_heading").as_double();
  config_.w_progress = get_parameter("w_progress").as_double();
  config_.w_steering = get_parameter("w_steering").as_double();
  config_.w_accel = get_parameter("w_accel").as_double();
  config_.w_dsteering = get_parameter("w_dsteering").as_double();
  config_.w_daccel = get_parameter("w_daccel").as_double();
  config_.w_terminal = get_parameter("w_terminal").as_double();
  config_.max_steering_angle = get_parameter("max_steering_angle").as_double();
  config_.max_accel = get_parameter("max_accel").as_double();
  config_.max_decel = get_parameter("max_decel").as_double();
  config_.max_speed = get_parameter("max_speed").as_double();
  config_.max_steering_rate = get_parameter("max_steering_rate").as_double();
  config_.max_jerk = get_parameter("max_jerk").as_double();
  config_.dt_min = get_parameter("dt_min").as_double();
  config_.max_solver_iterations = get_parameter("max_solver_iterations").as_int();
  config_.solver_eps_abs = get_parameter("solver_eps_abs").as_double();
  config_.solver_eps_rel = get_parameter("solver_eps_rel").as_double();
  config_.warm_start = get_parameter("warm_start").as_bool();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create MPC core
  core_ = std::make_unique<MpcCore>(config_, wheelbase_);

  // Publishers
  ackermann_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    ackermann_topic, rclcpp::QoS(10));
  idle_pub_ = create_publisher<std_msgs::msg::Bool>(idle_topic, rclcpp::QoS(10));
  predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    predicted_path_topic, rclcpp::QoS(10));

  // Subscribers
  trajectory_sub_ = create_subscription<wato_trajectory_msgs::msg::Trajectory>(
    trajectory_topic, rclcpp::QoS(10),
    std::bind(&MpcControllerNode::trajectory_callback, this, std::placeholders::_1));
  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    bt_topic, rclcpp::QoS(10),
    std::bind(&MpcControllerNode::bt_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    std::bind(&MpcControllerNode::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured: MPC control at %.1f Hz", control_rate_hz_);
  return CallbackReturn::SUCCESS;
}

MpcControllerNode::CallbackReturn MpcControllerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  ackermann_pub_->on_activate();
  idle_pub_->on_activate();
  predicted_path_pub_->on_activate();

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MpcControllerNode::control_callback, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

MpcControllerNode::CallbackReturn MpcControllerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  control_timer_.reset();
  ackermann_pub_->on_deactivate();
  idle_pub_->on_deactivate();
  predicted_path_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

MpcControllerNode::CallbackReturn MpcControllerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  ackermann_pub_.reset();
  idle_pub_.reset();
  predicted_path_pub_.reset();
  trajectory_sub_.reset();
  bt_sub_.reset();
  odom_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  latest_trajectory_.reset();
  core_.reset();

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

MpcControllerNode::CallbackReturn MpcControllerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  control_timer_.reset();
  ackermann_pub_.reset();
  idle_pub_.reset();
  predicted_path_pub_.reset();
  trajectory_sub_.reset();
  bt_sub_.reset();
  odom_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void MpcControllerNode::trajectory_callback(
  const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg)
{
  latest_trajectory_ = msg;
  last_trajectory_time_ = now();
}

void MpcControllerNode::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  current_speed_ = msg->twist.twist.linear.x;
}

void MpcControllerNode::bt_callback(
  const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg)
{
  bt_requested_behaviour_ = msg->behaviour;
}

void MpcControllerNode::control_callback()
{
  std_msgs::msg::Bool idle_msg;

  // Idle checks (same logic as pure pursuit)
  bool is_idle = !latest_trajectory_ || latest_trajectory_->points.empty() ||
                 (now() - last_trajectory_time_).seconds() > idle_timeout_sec_ ||
                 bt_requested_behaviour_.empty();

  if (is_idle || (!disable_standby_ && bt_requested_behaviour_ == standby_msg_)) {
    idle_msg.data = true;
    idle_pub_->publish(idle_msg);
    publish_ackermann(base_frame_, standby_speed_, standby_steering_);
    return;
  }

  idle_msg.data = false;
  idle_pub_->publish(idle_msg);

  // Get current vehicle state from TF
  StateVec current_state;
  try {
    auto tf = tf_buffer_->lookupTransform("map", base_frame_, tf2::TimePointZero);
    current_state(0) = tf.transform.translation.x;
    current_state(1) = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    current_state(2) = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    current_state(3) = current_speed_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot get vehicle pose from TF (map -> %s): %s", base_frame_.c_str(), ex.what());
    return;
  }

  // Sample reference from trajectory
  auto reference = core_->sample_reference(*latest_trajectory_, current_state);
  if (reference.size() < 2) {
    publish_ackermann(base_frame_, standby_speed_, standby_steering_);
    return;
  }

  // Solve MPC
  auto solution = core_->solve(current_state, reference, prev_steering_, prev_accel_);

  if (solution.solved) {
    prev_steering_ = solution.steering_angle;
    prev_accel_ = solution.acceleration;

    double speed = solution.target_speed;
    if (speed <= 0.0) speed = 0.0;

    publish_ackermann(base_frame_, speed, solution.steering_angle);

    // Publish predicted path for visualization
    if (!solution.predicted_states.empty()) {
      publish_predicted_path(solution.predicted_states);
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "MPC solver failed, holding previous command");
    publish_ackermann(base_frame_, std::max(0.0, current_speed_ + prev_accel_ * 0.05), prev_steering_);
  }
}

void MpcControllerNode::publish_ackermann(
  const std::string & frame_id, double speed, double steering_angle)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.steering_angle = invert_steering_ ? -steering_angle : steering_angle;
  cmd.drive.speed = speed;
  ackermann_pub_->publish(cmd);
}

void MpcControllerNode::publish_predicted_path(const std::vector<StateVec> & states)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = "map";

  for (const auto & s : states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = s(0);
    pose.pose.position.y = s(1);
    pose.pose.position.z = 0.0;
    double yaw = s(2);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path_msg.poses.push_back(pose);
  }

  predicted_path_pub_->publish(path_msg);
}

}  // namespace mpc_controller

RCLCPP_COMPONENTS_REGISTER_NODE(mpc_controller::MpcControllerNode)
