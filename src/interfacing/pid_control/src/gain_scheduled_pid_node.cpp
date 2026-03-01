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

#include "pid_control/gain_scheduled_pid_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace pid_control
{

GainScheduledPidNode::GainScheduledPidNode(const rclcpp::NodeOptions & options)
: LifecycleNode("gain_scheduled_pid_node", options)
{
  this->declare_parameter<double>("update_rate", 100.0);
  this->declare_parameter<double>("output_clamp_max", 1.0);
  this->declare_parameter<double>("output_clamp_min", -1.0);
  this->declare_parameter<std::vector<double>>("gain_schedule.speed_breakpoints", {0.0});
  this->declare_parameter<std::vector<double>>("gain_schedule.steering_gains_matrix", {2.5, 0.0, 0.1});
  this->declare_parameter<double>("gain_schedule.i_clamp_max", 1.0);
  this->declare_parameter<double>("gain_schedule.i_clamp_min", -1.0);
  this->declare_parameter<bool>("gain_schedule.antiwindup", true);

  RCLCPP_INFO(this->get_logger(), "GainScheduledPidNode created (unconfigured)");
}

GainScheduledPidNode::CallbackReturn GainScheduledPidNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  output_clamp_max_ = this->get_parameter("output_clamp_max").as_double();
  output_clamp_min_ = this->get_parameter("output_clamp_min").as_double();

  // Load gain schedule
  rebuild_gain_schedule();

  // Initialize steering PID with the first breakpoint's gains
  double init_p = 0.0, init_i = 0.0, init_d = 0.0;
  if (steering_gains_matrix_.size() >= 3) {
    init_p = steering_gains_matrix_[0];
    init_i = steering_gains_matrix_[1];
    init_d = steering_gains_matrix_[2];
  }
  steering_pid_.initPid(init_p, init_i, init_d, i_clamp_max_, i_clamp_min_, antiwindup_);

  // Initialize Velocity PID (fixed gains, uses PidROS)
  velocity_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "velocity_pid",
    "velocity_pid_state",
    true);
  velocity_pid_ros_->initialize_from_ros_parameters();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann", rclcpp::QoS(10),
    std::bind(&GainScheduledPidNode::ackermann_callback, this, std::placeholders::_1));

  steering_meas_sub_ = this->create_subscription<roscco_msg::msg::SteeringAngle>(
    "steering_feedback", rclcpp::QoS(10),
    std::bind(&GainScheduledPidNode::steering_feedback_callback, this, std::placeholders::_1));

  velocity_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "velocity_feedback", rclcpp::QoS(10),
    std::bind(&GainScheduledPidNode::velocity_feedback_callback, this, std::placeholders::_1));

  odom_meas_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom_feedback", rclcpp::QoS(10),
    std::bind(&GainScheduledPidNode::odom_feedback_callback, this, std::placeholders::_1));

  // Publishers
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(10));
  pid_gains_pub_ = this->create_publisher<pid_msgs::msg::PidGains>("pid_gains", rclcpp::QoS(10));

  // Parameter change callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & param : params) {
        const auto & name = param.get_name();
        if (name.find("gain_schedule.") == 0 ||
            name == "output_clamp_max" ||
            name == "output_clamp_min")
        {
          schedule_rebuild_pending_ = true;
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

GainScheduledPidNode::CallbackReturn GainScheduledPidNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  double update_rate = this->get_parameter("update_rate").as_double();

  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GainScheduledPidNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Activated - control loop running at %.1f Hz", update_rate);
  return CallbackReturn::SUCCESS;
}

GainScheduledPidNode::CallbackReturn GainScheduledPidNode::on_deactivate(
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

GainScheduledPidNode::CallbackReturn GainScheduledPidNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  odom_meas_sub_.reset();
  roscco_pub_.reset();
  pid_gains_pub_.reset();
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
  schedule_rebuild_pending_ = false;

  return CallbackReturn::SUCCESS;
}

GainScheduledPidNode::CallbackReturn GainScheduledPidNode::on_shutdown(
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
  pid_gains_pub_.reset();
  velocity_pid_ros_.reset();
  param_callback_handle_.reset();

  return CallbackReturn::SUCCESS;
}

void GainScheduledPidNode::ackermann_callback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  steering_setpoint_ = msg->drive.steering_angle;
  velocity_setpoint_ = msg->drive.speed;
  ackermann_received_ = true;
}

void GainScheduledPidNode::steering_feedback_callback(
  const roscco_msg::msg::SteeringAngle::SharedPtr msg)
{
  steering_meas_ = msg->angle;
  steering_meas_received_ = true;
}

void GainScheduledPidNode::velocity_feedback_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  if (velocity_source_ == VelocitySource::ODOM) {
    return;
  }

  if (velocity_source_ == VelocitySource::NONE) {
    velocity_source_ = VelocitySource::CAN;
    RCLCPP_INFO(this->get_logger(), "Velocity source locked to CAN (Float64)");
  }

  velocity_meas_ = msg->data;
  current_velocity_ = msg->data;
  velocity_meas_received_ = true;
}

void GainScheduledPidNode::odom_feedback_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (velocity_source_ == VelocitySource::CAN) {
    return;
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
}

void GainScheduledPidNode::rebuild_gain_schedule()
{
  speed_breakpoints_ = this->get_parameter("gain_schedule.speed_breakpoints").as_double_array();
  steering_gains_matrix_ = this->get_parameter("gain_schedule.steering_gains_matrix").as_double_array();
  i_clamp_max_ = this->get_parameter("gain_schedule.i_clamp_max").as_double();
  i_clamp_min_ = this->get_parameter("gain_schedule.i_clamp_min").as_double();
  antiwindup_ = this->get_parameter("gain_schedule.antiwindup").as_bool();
  output_clamp_max_ = this->get_parameter("output_clamp_max").as_double();
  output_clamp_min_ = this->get_parameter("output_clamp_min").as_double();

  size_t expected_size = speed_breakpoints_.size() * 3;
  if (steering_gains_matrix_.size() != expected_size) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Gain schedule size mismatch: %zu breakpoints require %zu gain values, got %zu",
      speed_breakpoints_.size(), expected_size, steering_gains_matrix_.size());
  }
}

void GainScheduledPidNode::control_loop()
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
  if (schedule_rebuild_pending_) {
    rebuild_gain_schedule();
    schedule_rebuild_pending_ = false;
    RCLCPP_INFO(this->get_logger(), "Gain schedule parameters updated");
  }

  double steering_command = 0.0;
  double velocity_command = 0.0;

  // Interpolate gains and compute steering command
  if (steering_meas_received_) {
    // Linear interpolation of P, I, D from speed breakpoints
    double interp_p = 0.0, interp_i = 0.0, interp_d = 0.0;
    size_t n = speed_breakpoints_.size();

    if (n > 0 && steering_gains_matrix_.size() >= n * 3) {
      double v = current_velocity_;

      if (v <= speed_breakpoints_[0] || n == 1) {
        // Below or at first breakpoint
        interp_p = steering_gains_matrix_[0];
        interp_i = steering_gains_matrix_[1];
        interp_d = steering_gains_matrix_[2];
      } else if (v >= speed_breakpoints_[n - 1]) {
        // Above or at last breakpoint
        size_t idx = (n - 1) * 3;
        interp_p = steering_gains_matrix_[idx];
        interp_i = steering_gains_matrix_[idx + 1];
        interp_d = steering_gains_matrix_[idx + 2];
      } else {
        // Find the two surrounding breakpoints
        for (size_t i = 0; i < n - 1; ++i) {
          if (v >= speed_breakpoints_[i] && v < speed_breakpoints_[i + 1]) {
            double t = (v - speed_breakpoints_[i]) / (speed_breakpoints_[i + 1] - speed_breakpoints_[i]);
            size_t lo = i * 3;
            size_t hi = (i + 1) * 3;
            interp_p = steering_gains_matrix_[lo] + t * (steering_gains_matrix_[hi] - steering_gains_matrix_[lo]);
            interp_i = steering_gains_matrix_[lo + 1] + t * (steering_gains_matrix_[hi + 1] - steering_gains_matrix_[lo + 1]);
            interp_d = steering_gains_matrix_[lo + 2] + t * (steering_gains_matrix_[hi + 2] - steering_gains_matrix_[lo + 2]);
            break;
          }
        }
      }

      steering_pid_.setGains(interp_p, interp_i, interp_d, i_clamp_max_, i_clamp_min_, antiwindup_);

      pid_msgs::msg::PidGains gains_msg;
      gains_msg.header.stamp = now;
      gains_msg.p = interp_p;
      gains_msg.i = interp_i;
      gains_msg.d = interp_d;
      pid_gains_pub_->publish(gains_msg);
    }

    double steering_error = steering_setpoint_ - steering_meas_;
    steering_command = steering_pid_.computeCommand(steering_error, dt.nanoseconds());
    steering_command = std::clamp(steering_command, output_clamp_min_, output_clamp_max_);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for steering feedback...");
  }

  // Compute velocity PID output
  if (velocity_meas_received_) {
    double velocity_error = velocity_setpoint_ - velocity_meas_;
    velocity_command = velocity_pid_ros_->compute_command(velocity_error, dt);
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
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::GainScheduledPidNode)
