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

#include "pid_control/vel_driven_feedforward_pid_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace pid_control
{

VelDrivenFeedforwardPidNode::VelDrivenFeedforwardPidNode(const rclcpp::NodeOptions & options)
: LifecycleNode("vel_driven_feedforward_pid_node", options)
{
  this->declare_parameter<double>("update_rate", 100.0);
  this->declare_parameter<double>("steering.output_clamp_max", 1.0);
  this->declare_parameter<double>("steering.output_clamp_min", -1.0);
  this->declare_parameter<double>("steering.d_on_measurement", 0.0);
  this->declare_parameter<std::vector<double>>("steering.feedforward.coefficients", {0.0});
  this->declare_parameter<double>("steering.feedforward.friction_offset", 0.0);
  this->declare_parameter<double>("velocity.filter_alpha", 0.2);
  this->declare_parameter<double>("velocity.output.throttle_scale", 1.0);
  this->declare_parameter<double>("velocity.output.brake_scale", 1.0);
  this->declare_parameter<double>("velocity.output.deadband", 0.0);

  RCLCPP_INFO(this->get_logger(), "VelDrivenFeedforwardPidNode created (unconfigured)");
}

VelDrivenFeedforwardPidNode::CallbackReturn VelDrivenFeedforwardPidNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  output_clamp_max_ = this->get_parameter("steering.output_clamp_max").as_double();
  output_clamp_min_ = this->get_parameter("steering.output_clamp_min").as_double();
  d_on_meas_gain_ = this->get_parameter("steering.d_on_measurement").as_double();
  feedforward_coefficients_ = this->get_parameter("steering.feedforward.coefficients").as_double_array();
  feedforward_friction_offset_ = this->get_parameter("steering.feedforward.friction_offset").as_double();
  velocity_filter_alpha_ = this->get_parameter("velocity.filter_alpha").as_double();
  throttle_scale_ = this->get_parameter("velocity.output.throttle_scale").as_double();
  brake_scale_ = this->get_parameter("velocity.output.brake_scale").as_double();
  velocity_deadband_ = this->get_parameter("velocity.output.deadband").as_double();

  // Initialize Steering PID
  steering_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "steering.pid",
    "steering_pid_state",
    true);
  steering_pid_ros_->initialize_from_ros_parameters();

  // Initialize Velocity PID
  velocity_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "velocity.pid",
    "velocity_pid_state",
    true);
  velocity_pid_ros_->initialize_from_ros_parameters();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann",
    rclcpp::QoS(10),
    std::bind(&VelDrivenFeedforwardPidNode::ackermann_callback, this, std::placeholders::_1));

  steering_meas_sub_ = this->create_subscription<roscco_msg::msg::SteeringAngle>(
    "steering_feedback",
    rclcpp::QoS(10),
    std::bind(&VelDrivenFeedforwardPidNode::steering_feedback_callback, this, std::placeholders::_1));

  velocity_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "velocity_feedback",
    rclcpp::QoS(10),
    std::bind(&VelDrivenFeedforwardPidNode::velocity_feedback_callback, this, std::placeholders::_1));

  odom_meas_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom_feedback",
    rclcpp::QoS(10),
    std::bind(&VelDrivenFeedforwardPidNode::odom_feedback_callback, this, std::placeholders::_1));

  // Publishers
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(10));
  feedforward_pub_ = this->create_publisher<pid_msgs::msg::Feedforward>("feedforward", rclcpp::QoS(10));
  velocity_derived_pub_ = this->create_publisher<std_msgs::msg::Float64>("velocity_derived", rclcpp::QoS(10));

  // Parameter change callback
  param_callback_handle_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
    for (const auto & param : params) {
      if (
        param.get_name() == "steering.feedforward.coefficients" ||
        param.get_name() == "steering.feedforward.friction_offset" || param.get_name() == "steering.output_clamp_max" ||
        param.get_name() == "steering.output_clamp_min" || param.get_name() == "steering.d_on_measurement")
      {
        feedforward_rebuild_pending_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  });

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

VelDrivenFeedforwardPidNode::CallbackReturn VelDrivenFeedforwardPidNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  double update_rate = this->get_parameter("update_rate").as_double();

  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VelDrivenFeedforwardPidNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Activated - control loop running at %.1f Hz", update_rate);
  return CallbackReturn::SUCCESS;
}

VelDrivenFeedforwardPidNode::CallbackReturn VelDrivenFeedforwardPidNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

VelDrivenFeedforwardPidNode::CallbackReturn VelDrivenFeedforwardPidNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  odom_meas_sub_.reset();
  roscco_pub_.reset();
  feedforward_pub_.reset();
  velocity_derived_pub_.reset();
  steering_pid_ros_.reset();
  velocity_pid_ros_.reset();
  param_callback_handle_.reset();

  steering_setpoint_ = 0.0;
  steering_meas_ = 0.0;
  velocity_setpoint_ = 0.0;
  velocity_meas_ = 0.0;
  current_velocity_ = 0.0;
  ackermann_received_ = false;
  steering_meas_received_ = false;
  velocity_meas_received_ = false;
  velocity_source_ = VelocitySource::NONE;
  feedforward_rebuild_pending_ = false;
  steering_meas_prev_ = 0.0;
  steering_meas_prev_valid_ = false;
  velocity_meas_filtered_ = 0.0;
  velocity_filter_initialized_ = false;

  return CallbackReturn::SUCCESS;
}

VelDrivenFeedforwardPidNode::CallbackReturn VelDrivenFeedforwardPidNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  odom_meas_sub_.reset();
  roscco_pub_.reset();
  feedforward_pub_.reset();
  velocity_derived_pub_.reset();
  steering_pid_ros_.reset();
  velocity_pid_ros_.reset();
  param_callback_handle_.reset();

  return CallbackReturn::SUCCESS;
}

void VelDrivenFeedforwardPidNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  steering_setpoint_ = msg->drive.steering_angle;
  velocity_setpoint_ = msg->drive.speed;
  ackermann_received_ = true;
}

void VelDrivenFeedforwardPidNode::steering_feedback_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg)
{
  steering_meas_ = msg->angle;
  steering_meas_received_ = true;
}

void VelDrivenFeedforwardPidNode::velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (velocity_source_ == VelocitySource::ODOM) {
    return;  // Locked to odometry source
  }

  if (velocity_source_ == VelocitySource::NONE) {
    velocity_source_ = VelocitySource::CAN;
    RCLCPP_INFO(this->get_logger(), "Velocity source locked to CAN (Float64)");
  }

  velocity_meas_ = msg->data;
  current_velocity_ = msg->data;
  velocity_meas_received_ = true;
}

void VelDrivenFeedforwardPidNode::odom_feedback_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (velocity_source_ == VelocitySource::CAN) {
    return;  // Locked to CAN source
  }

  if (velocity_source_ == VelocitySource::NONE) {
    velocity_source_ = VelocitySource::ODOM;
    RCLCPP_INFO(this->get_logger(), "Velocity source locked to Odometry");
  }

  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double vz = msg->twist.twist.linear.z;
  double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

  velocity_meas_ = speed;
  current_velocity_ = speed;
  velocity_meas_received_ = true;

  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = speed;
  velocity_derived_pub_->publish(speed_msg);
}

double VelDrivenFeedforwardPidNode::compute_feedforward(double velocity, double steering_setpoint) const
{
  // T_ff = (c0 + c1*v + c2*v^2 + ...) * steering_setpoint + friction_offset * sign(steering_setpoint)
  double ff = 0.0;
  double v_power = 1.0;
  for (const auto & c : feedforward_coefficients_) {
    ff += c * v_power;
    v_power *= velocity;
  }
  double result = ff * steering_setpoint;
  if (feedforward_friction_offset_ != 0.0 && steering_setpoint != 0.0) {
    result += feedforward_friction_offset_ * (steering_setpoint > 0.0 ? 1.0 : -1.0);
  }
  return result;
}

void VelDrivenFeedforwardPidNode::control_loop()
{
  rclcpp::Time now = this->now();
  rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.nanoseconds() <= 0) {
    return;
  }

  if (!ackermann_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for ackermann setpoint...");
    return;
  }

  // Handle pending parameter updates
  if (feedforward_rebuild_pending_) {
    feedforward_coefficients_ = this->get_parameter("steering.feedforward.coefficients").as_double_array();
    feedforward_friction_offset_ = this->get_parameter("steering.feedforward.friction_offset").as_double();
    output_clamp_max_ = this->get_parameter("steering.output_clamp_max").as_double();
    output_clamp_min_ = this->get_parameter("steering.output_clamp_min").as_double();
    d_on_meas_gain_ = this->get_parameter("steering.d_on_measurement").as_double();
    feedforward_rebuild_pending_ = false;
    RCLCPP_INFO(this->get_logger(), "Feedforward parameters updated");
  }

  double steering_command = 0.0;
  double velocity_command = 0.0;

  // Compute steering PID output
  if (steering_meas_received_) {
    double steering_error = steering_setpoint_ - steering_meas_;
    double pid_output = steering_pid_ros_->compute_command(steering_error, dt);

    double ff_output = compute_feedforward(current_velocity_, steering_setpoint_);

    pid_msgs::msg::Feedforward ff_msg;
    ff_msg.header.stamp = now;
    ff_msg.feedforward = ff_output;
    feedforward_pub_->publish(ff_msg);

    // D-on-measurement: -D * d(measurement)/dt (negative because rising measurement should oppose)
    double d_on_meas = 0.0;
    if (steering_meas_prev_valid_ && d_on_meas_gain_ != 0.0) {
      double dt_sec = dt.seconds();
      if (dt_sec > 0.0) {
        d_on_meas = -d_on_meas_gain_ * (steering_meas_ - steering_meas_prev_) / dt_sec;
      }
    }
    steering_meas_prev_ = steering_meas_;
    steering_meas_prev_valid_ = true;

    double combined = pid_output + ff_output + d_on_meas;
    steering_command = std::clamp(combined, output_clamp_min_, output_clamp_max_);

  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for steering feedback...");
  }

  // Compute velocity PID output with asymmetric scaling and deadband
  if (velocity_meas_received_) {
    double velocity_error = velocity_setpoint_ - velocity_meas_;
    double raw_effort = velocity_pid_ros_->compute_command(velocity_error, dt);

    if (std::abs(raw_effort) < velocity_deadband_) {
      velocity_command = 0.0;
    } else if (raw_effort >= 0.0) {
      velocity_command = raw_effort * throttle_scale_;
    } else {
      velocity_command = raw_effort * brake_scale_;
    }
    velocity_command = std::clamp(velocity_command, -1.0, 1.0);

    RCLCPP_INFO(
      this->get_logger(),
      "VEL | err=%.4f raw=%.4f cmd=%.4f setpt=%.4f meas=%.4f",
      velocity_error,
      raw_effort,
      velocity_command,
      velocity_setpoint_,
      velocity_meas_);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for velocity feedback...");
  }

  // Publish Roscco message
  roscco_msg::msg::Roscco msg;
  msg.header.stamp = now;
  msg.steering = static_cast<float>(steering_command);
  msg.forward = static_cast<float>(velocity_command);

  roscco_pub_->publish(msg);
}

}  // namespace pid_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::VelDrivenFeedforwardPidNode)
