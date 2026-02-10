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

#include "car_velocity_feedback/car_velocity_feedback_node.hpp"

#include <cmath>

namespace car_velocity_feedback
{

CarVelocityFeedbackNode::CarVelocityFeedbackNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("car_velocity_feedback_node", options)
, current_steering_angle_rad_(0.0)
, has_steering_angle_(false)
{
  RCLCPP_INFO(this->get_logger(), "Car Velocity Feedback Node initialized.");
}

CarVelocityFeedbackNode::CallbackReturn CarVelocityFeedbackNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Car Velocity Feedback Node");

  // Subscribe to wheel speeds
  wheel_speeds_sub_ = this->create_subscription<roscco_msg::msg::WheelSpeeds>(
    "/oscc_interfacing/wheel_speeds",
    rclcpp::SensorDataQoS(),
    std::bind(&CarVelocityFeedbackNode::wheel_speeds_callback, this, std::placeholders::_1));

  // Subscribe to steering angle
  steering_angle_sub_ = this->create_subscription<roscco_msg::msg::SteeringAngle>(
    "/oscc_interfacing/steering_angle",
    rclcpp::SensorDataQoS(),
    std::bind(&CarVelocityFeedbackNode::steering_angle_callback, this, std::placeholders::_1));

  // Publisher for body velocity
  body_velocity_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/body_velocity_feedback", rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

CarVelocityFeedbackNode::CallbackReturn CarVelocityFeedbackNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Car Velocity Feedback Node");
  body_velocity_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CarVelocityFeedbackNode::CallbackReturn CarVelocityFeedbackNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Car Velocity Feedback Node");
  body_velocity_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CarVelocityFeedbackNode::CallbackReturn CarVelocityFeedbackNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Car Velocity Feedback Node");
  wheel_speeds_sub_.reset();
  steering_angle_sub_.reset();
  body_velocity_pub_.reset();
  return CallbackReturn::SUCCESS;
}

CarVelocityFeedbackNode::CallbackReturn CarVelocityFeedbackNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Car Velocity Feedback Node");
  wheel_speeds_sub_.reset();
  steering_angle_sub_.reset();
  body_velocity_pub_.reset();
  return CallbackReturn::SUCCESS;
}

void CarVelocityFeedbackNode::steering_angle_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg)
{
  current_steering_angle_rad_ = msg->angle;
  has_steering_angle_ = true;
}

void CarVelocityFeedbackNode::wheel_speeds_callback(const roscco_msg::msg::WheelSpeeds::SharedPtr msg)
{
  if (!body_velocity_pub_->is_activated()) {
    return;
  }

  if (!has_steering_angle_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for steering angle data to calculate body velocity...");
    return;
  }

  // Convert km/h to m/s
  // 1 km/h = 1000m / 3600s = 1/3.6 m/s
  const double KPH_TO_MPS = 1.0 / 3.6;

  // Average front wheel speeds (FL = nw, FR = ne)
  double v_front_avg_kph = (msg->nw + msg->ne) / 2.0;
  double v_front_avg_mps = v_front_avg_kph * KPH_TO_MPS;

  // Calculate body velocity: v_body = v_front_avg * cos(steering_angle)
  // Assuming Ackermann steering geometry approximation
  double v_body = v_front_avg_mps * std::cos(current_steering_angle_rad_);

  std_msgs::msg::Float64 output_msg;
  output_msg.data = v_body;
  body_velocity_pub_->publish(output_msg);
}

}  // namespace car_velocity_feedback

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(car_velocity_feedback::CarVelocityFeedbackNode)
