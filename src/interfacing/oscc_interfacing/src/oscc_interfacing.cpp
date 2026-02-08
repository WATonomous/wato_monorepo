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

#include "oscc_interfacing/oscc_interfacing.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp_components/register_node_macro.hpp>

namespace oscc_interfacing
{

// Static pointer to the node instance for free function callbacks
static OsccInterfacingNode * g_node_instance = nullptr;

// Free function callbacks for OSCC library
void brake_report_callback(oscc_brake_report_s * report)
{
  if (report->operator_override && g_node_instance) {
    RCLCPP_INFO(g_node_instance->get_logger(), "Brake Operator Override");
    if (oscc_disable() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(g_node_instance->arm_mutex_);
        g_node_instance->is_armed_ = false;
      }
      RCLCPP_INFO(g_node_instance->get_logger(), "Vehicle disarmed");
    } else {
      RCLCPP_FATAL(g_node_instance->get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void throttle_report_callback(oscc_throttle_report_s * report)
{
  if (report->operator_override && g_node_instance) {
    RCLCPP_INFO(g_node_instance->get_logger(), "Throttle Operator Override");
    if (oscc_disable() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(g_node_instance->arm_mutex_);
        g_node_instance->is_armed_ = false;
      }
      RCLCPP_INFO(g_node_instance->get_logger(), "Vehicle disarmed");
    } else {
      RCLCPP_FATAL(g_node_instance->get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void steering_report_callback(oscc_steering_report_s * report)
{
  if (report->operator_override && g_node_instance) {
    RCLCPP_INFO(g_node_instance->get_logger(), "Steering Operator Override");
    if (oscc_disable() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(g_node_instance->arm_mutex_);
        g_node_instance->is_armed_ = false;
      }
      RCLCPP_INFO(g_node_instance->get_logger(), "Vehicle disarmed");
    } else {
      RCLCPP_FATAL(g_node_instance->get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void obd_callback(struct can_frame * frame)
{
  if (!g_node_instance) {
    return;
  }
  // this only passes if it is indeed a wheel speed msg
  double se;
  if (get_wheel_speed_right_rear(frame, &se) == OSCC_OK) {
    double ne;
    double nw;
    double sw;
    get_wheel_speed_left_front(frame, &nw);
    get_wheel_speed_left_rear(frame, &sw);
    get_wheel_speed_right_front(frame, &ne);
    g_node_instance->publish_wheel_speeds(
      static_cast<float>(ne), static_cast<float>(nw), static_cast<float>(se), static_cast<float>(sw));
  } else if (get_steering_wheel_angle(frame, &se) == OSCC_OK) {
    g_node_instance->publish_steering_wheel_angle(static_cast<float>(se));
  }
}

void fault_report_callback(oscc_fault_report_s * report)
{
  if (g_node_instance) {
    if (report->fault_origin_id == FAULT_ORIGIN_BRAKE) {
      RCLCPP_INFO(g_node_instance->get_logger(), "Brake Fault");
    } else if (report->fault_origin_id == FAULT_ORIGIN_STEERING) {
      RCLCPP_INFO(g_node_instance->get_logger(), "Steering Fault");
    } else if (report->fault_origin_id == FAULT_ORIGIN_THROTTLE) {
      RCLCPP_INFO(g_node_instance->get_logger(), "Throttle Fault");
    }

    if (oscc_disable() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(g_node_instance->arm_mutex_);
        g_node_instance->is_armed_ = false;
      }
      RCLCPP_INFO(g_node_instance->get_logger(), "Vehicle disarmed");
    } else {
      RCLCPP_FATAL(g_node_instance->get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

OsccInterfacingNode::OsccInterfacingNode(const rclcpp::NodeOptions & options)
: Node("oscc_interfacing_node", options)
{
  configure();
  RCLCPP_INFO(this->get_logger(), "OsccInterfacingNode initialized");
}

OsccInterfacingNode::~OsccInterfacingNode()
{
  // Nullify global pointer to prevent callbacks from accessing deleted object
  g_node_instance = nullptr;
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (is_armed_) {
    oscc_disable();
  }
  oscc_close();
}

void OsccInterfacingNode::configure()
{
  // Set static pointer for free function callbacks
  g_node_instance = this;

  // Declare parameters
  this->declare_parameter<int>("is_armed_publish_rate_hz", 100);
  this->declare_parameter<int>("oscc_can_bus", 0);
  this->declare_parameter<float>("steering_scaling", 1);

  // Read parameters
  is_armed_ = false;
  is_armed_publish_rate_hz = this->get_parameter("is_armed_publish_rate_hz").as_int();
  oscc_can_bus_ = this->get_parameter("oscc_can_bus").as_int();
  steering_scaling_ = this->get_parameter("steering_scaling").as_double();

  if (steering_scaling_ > 1.0 || steering_scaling_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Steering scaling parameter out of range (0.0, 1.0], resetting to 1.0");
    steering_scaling_ = 1.0;
  }

  // Create subscription to /roscco
  roscco_sub_ = this->create_subscription<roscco_msg::msg::Roscco>(
    "/roscco", rclcpp::QoS(1), std::bind(&OsccInterfacingNode::roscco_callback, this, std::placeholders::_1));

  // Create publishers
  is_armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/oscc_interfacing/is_armed", rclcpp::QoS(1));

  wheel_speeds_pub_ =
    this->create_publisher<roscco_msg::msg::WheelSpeeds>("/oscc_interfacing/wheel_speeds", rclcpp::QoS(1));

  steering_wheel_angle_pub_ =
    this->create_publisher<roscco_msg::msg::SteeringAngle>("/oscc_interfacing/steering_wheel_angle", rclcpp::QoS(1));

  // Create arm service
  arm_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/oscc_interfacing/arm",
    std::bind(&OsccInterfacingNode::arm_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Create 100Hz timer for is_armed publication
  std::chrono::milliseconds interval(1000 / is_armed_publish_rate_hz);
  is_armed_timer_ = this->create_wall_timer(interval, std::bind(&OsccInterfacingNode::is_armed_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "OsccInterfacingNode configured: armed=%d, is_armed_publish_rate=%d Hz",
    is_armed_,
    is_armed_publish_rate_hz);

  // we dont use autodetect can
  // if (oscc_init() != OSCC_OK) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSCC library");
  // } else {
  //   RCLCPP_INFO(this->get_logger(), "OSCC library initialized successfully");
  // }

  if (oscc_open(oscc_can_bus_) != OSCC_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open OSCC communication");
  } else {
    RCLCPP_INFO(this->get_logger(), "OSCC communication opened successfully");
    if (oscc_subscribe_to_brake_reports(brake_report_callback) != OSCC_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to brake reports");
    }
    if (oscc_subscribe_to_steering_reports(steering_report_callback) != OSCC_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to steering reports");
    }
    if (oscc_subscribe_to_throttle_reports(throttle_report_callback) != OSCC_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to throttle reports");
    }
    if (oscc_subscribe_to_fault_reports(fault_report_callback) != OSCC_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to fault reports");
    }
    if (oscc_subscribe_to_obd_messages(obd_callback) != OSCC_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to OBD messages");
    }
  }
}

void OsccInterfacingNode::roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg)
{
  // if not armed ignore
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    if (!is_armed_) {
      RCLCPP_WARN(this->get_logger(), "Vehicle not armed, Ignoring roscco message");
      return;
    }
  }

  // Check if at least 40ms has passed since last message
  // To not overload CAN (targeting 20HZ, 50-40=10ms leeway)
  rclcpp::Time now = rclcpp::Clock().now();
  if ((now - last_message_time_).nanoseconds() < 40000000) {  // 40ms in nanoseconds
    RCLCPP_WARN(this->get_logger(), "Message too soon, Ignoring roscco message to avoid CAN overload");
    return;
  }
  last_message_time_ = now;

  float brake = 0.0;
  float throttle = 0.0;
  float forward = msg->forward;
  float steering = msg->steering;

  if (std::abs(forward) > 1.0) {
    RCLCPP_ERROR(this->get_logger(), "Forward command out of range [-1, 1], this should not happen! Ignoring message.");
    return;
  }
  if (std::abs(steering) > 1.0) {
    RCLCPP_ERROR(
      this->get_logger(), "Steering command out of range [-1, 1], this should not happen! Ignoring message.");
    return;
  }

  // If forward is positive, set throttle; if negative, set brake
  if (forward >= 0.0) {
    throttle = forward;
    brake = 0.0;
  } else {
    throttle = 0.0;
    brake = -forward;
  }

  /**
   * @brief Check against past values for a smooth transition order
   * between throttle and brake
   */
  if (forward > 0.0 && last_forward_ < 0.0) {
    // Transitioning from brake to throttle
    // first reduce brake, then apply throttle
    handle_any_errors(oscc_publish_brake_position(brake));
    handle_any_errors(oscc_publish_throttle_position(throttle));

  } else {
    // Transitioning from throttle to brake
    // first reduce throttle, then apply brake

    // Also handles base case of no transition
    handle_any_errors(oscc_publish_throttle_position(throttle));
    handle_any_errors(oscc_publish_brake_position(brake));
  }

  // always pub steering
  handle_any_errors(oscc_publish_steering_torque(steering * steering_scaling_));

  last_forward_ = forward;
}

void OsccInterfacingNode::arm_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {  // data is the boolean, true = arm, false = disarm
    // Arm the vehicle
    if (oscc_enable_steering() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        is_armed_ = true;
      }
      response->success = true;
      response->message = "Vehicle armed successfully";
      RCLCPP_INFO(get_logger(), "Vehicle armed");
    } else {
      response->success = false;
      response->message = "Failed to arm vehicle";
      RCLCPP_ERROR(get_logger(), "Failed to arm vehicle");
    }
  } else {
    // Disarm the vehicle
    if (oscc_disable() == OSCC_OK) {
      {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        is_armed_ = false;
      }
      response->success = true;
      response->message = "Vehicle disarmed successfully";
      RCLCPP_INFO(get_logger(), "Vehicle disarmed");
    } else {
      response->success = false;
      response->message = "Failed to disarm vehicle";
      RCLCPP_FATAL(get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void OsccInterfacingNode::is_armed_timer_callback()
{
  std_msgs::msg::Bool msg;
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    msg.data = is_armed_;
  }
  is_armed_pub_->publish(msg);
}

void OsccInterfacingNode::publish_wheel_speeds(float NE, float NW, float SE, float SW)
{
  roscco_msg::msg::WheelSpeeds msg;
  msg.ne = NE;
  msg.nw = NW;
  msg.se = SE;
  msg.sw = SW;
  // Use get_clock()->now() instead of this->now() for thread safety
  // This maintains ROS 2 time synchronization while being thread-safe for CAN callbacks
  msg.header.stamp = get_clock()->now();
  wheel_speeds_pub_->publish(msg);
}

void OsccInterfacingNode::publish_steering_wheel_angle(float angle_degrees)
{
  roscco_msg::msg::SteeringAngle msg;
  msg.angle = angle_degrees;
  // Use get_clock()->now() instead of this->now() for thread safety
  // This maintains ROS 2 time synchronization while being thread-safe for CAN callbacks
  msg.header.stamp = get_clock()->now();
  steering_wheel_angle_pub_->publish(msg);
}

oscc_result_t OsccInterfacingNode::handle_any_errors(oscc_result_t result)
{
  if (result == OSCC_OK) {
    return OSCC_OK;
  }
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    is_armed_ = false;
  }
  RCLCPP_ERROR(this->get_logger(), "Error from OSCC API: %d, ATTEMPTING TO DISARM ALL BOARDS", result);
  // Attempt to disarm all boards
  if (oscc_disable() != OSCC_OK) {
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
  } else {
    RCLCPP_INFO(this->get_logger(), "All boards disarmed successfully after error");
  }
  return result;  // pass error up
}

}  // namespace oscc_interfacing

RCLCPP_COMPONENTS_REGISTER_NODE(oscc_interfacing::OsccInterfacingNode)
