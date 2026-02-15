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
#include <string>

namespace oscc_interfacing
{

// Atomic pointer to the node instance for CAN thread callbacks.
// Prevents destructor TOCTOU: CAN callback loads pointer, destructor stores nullptr.
static std::atomic<OsccInterfacingNode *> g_node_instance{nullptr};

// --- CAN thread free function callbacks ---

void brake_report_callback(oscc_brake_report_s * report)
{
  auto * node = g_node_instance.load();
  if (report->operator_override && node) {
    node->latest_override_.store(OsccInterfacingNode::OverrideType::BRAKE);
    node->has_override_.store(true);
  }
}

void throttle_report_callback(oscc_throttle_report_s * report)
{
  auto * node = g_node_instance.load();
  if (report->operator_override && node) {
    node->latest_override_.store(OsccInterfacingNode::OverrideType::THROTTLE);
    node->has_override_.store(true);
  }
}

void steering_report_callback(oscc_steering_report_s * report)
{
  auto * node = g_node_instance.load();
  if (report->operator_override && node) {
    node->latest_override_.store(OsccInterfacingNode::OverrideType::STEERING);
    node->has_override_.store(true);
  }
}

void obd_callback(struct can_frame * frame)
{
  auto * node = g_node_instance.load();
  if (!node) {
    return;
  }

  double se;
  if (get_wheel_speed_right_rear(frame, &se) == OSCC_OK) {
    double ne, nw, sw;
    get_wheel_speed_left_front(frame, &nw);
    get_wheel_speed_left_rear(frame, &sw);
    get_wheel_speed_right_front(frame, &ne);

    // Lock to write all 4 values as a consistent snapshot
    {
      std::lock_guard<std::mutex> lock(node->wheel_data_mutex_);
      node->latest_wheel_data_.ne = static_cast<float>(ne);
      node->latest_wheel_data_.nw = static_cast<float>(nw);
      node->latest_wheel_data_.se = static_cast<float>(se);
      node->latest_wheel_data_.sw = static_cast<float>(sw);
    }
    node->has_wheel_data_.store(true);
  } else if (get_steering_wheel_angle(frame, &se) == OSCC_OK) {
    node->latest_steering_angle_.store(static_cast<float>(se));
    node->has_steering_data_.store(true);
  }
}

void fault_report_callback(oscc_fault_report_s * report)
{
  auto * node = g_node_instance.load();
  if (node) {
    if (report->fault_origin_id == FAULT_ORIGIN_BRAKE) {
      node->latest_fault_.store(OsccInterfacingNode::FaultType::BRAKE_FAULT);
    } else if (report->fault_origin_id == FAULT_ORIGIN_STEERING) {
      node->latest_fault_.store(OsccInterfacingNode::FaultType::STEERING_FAULT);
    } else if (report->fault_origin_id == FAULT_ORIGIN_THROTTLE) {
      node->latest_fault_.store(OsccInterfacingNode::FaultType::THROTTLE_FAULT);
    }
    node->has_fault_.store(true);
  }
}

// --- Node implementation ---

OsccInterfacingNode::OsccInterfacingNode(const rclcpp::NodeOptions & options)
: Node("oscc_interfacing_node", options)
{
  configure();
  RCLCPP_INFO(this->get_logger(), "OsccInterfacingNode initialized");
}

OsccInterfacingNode::~OsccInterfacingNode()
{
  g_node_instance.store(nullptr);
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (is_armed_) {
    oscc_disable();
  }
  oscc_close();
}

void OsccInterfacingNode::configure()
{
  g_node_instance.store(this);

  // Declare parameters
  this->declare_parameter<int>("is_armed_publish_rate_hz", 100);
  this->declare_parameter<int>("oscc_can_bus", 0);
  this->declare_parameter<float>("steering_scaling", 1);
  this->declare_parameter<bool>("disable_boards_on_fault", false);
  this->declare_parameter<float>("steering_conversion_factor", 15.7);
  this->declare_parameter<float>("steering_torque_deadzone_pos", 0.08);
  this->declare_parameter<float>("steering_torque_deadzone_neg", 0.12);
  this->declare_parameter<bool>("enable_all", true);
  this->declare_parameter<bool>("enable_steering", true);
  this->declare_parameter<bool>("enable_throttle", true);
  this->declare_parameter<bool>("enable_brakes", true);

  // Read parameters
  is_armed_ = false;
  is_armed_publish_rate_hz = this->get_parameter("is_armed_publish_rate_hz").as_int();
  oscc_can_bus_ = this->get_parameter("oscc_can_bus").as_int();
  steering_scaling_ = this->get_parameter("steering_scaling").as_double();
  disable_boards_on_fault_ = this->get_parameter("disable_boards_on_fault").as_bool();
  steering_conversion_factor_ = this->get_parameter("steering_conversion_factor").as_double();
  steering_torque_deadzone_pos_ = this->get_parameter("steering_torque_deadzone_pos").as_double();
  steering_torque_deadzone_neg_ = this->get_parameter("steering_torque_deadzone_neg").as_double();
  enable_all_ = this->get_parameter("enable_all").as_bool();
  enable_steering_ = this->get_parameter("enable_steering").as_bool();
  enable_throttle_ = this->get_parameter("enable_throttle").as_bool();
  enable_brakes_ = this->get_parameter("enable_brakes").as_bool();

  if (steering_scaling_ > 1.0 || steering_scaling_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Steering scaling parameter out of range (0.0, 1.0], resetting to 1.0");
    steering_scaling_ = 1.0;
  }

  // Create callback groups
  oscc_api_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  feedback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Group A: subscription and service (OSCC API callers)
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = oscc_api_group_;
  roscco_sub_ = this->create_subscription<roscco_msg::msg::Roscco>(
    "/roscco", rclcpp::QoS(1),
    std::bind(&OsccInterfacingNode::roscco_callback, this, std::placeholders::_1),
    sub_options);

  arm_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/oscc_interfacing/arm",
    std::bind(&OsccInterfacingNode::arm_service_callback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    oscc_api_group_);

  // Group A: event timer (handles override/fault → oscc_disable)
  event_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&OsccInterfacingNode::process_events, this),
    oscc_api_group_);

  // Group B: feedback timer (publishes wheel speeds + steering angle, no OSCC API)
  feedback_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&OsccInterfacingNode::publish_feedback, this),
    feedback_group_);

  // Publishers
  is_armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/oscc_interfacing/is_armed", rclcpp::QoS(1));
  wheel_speeds_pub_ =
    this->create_publisher<roscco_msg::msg::WheelSpeeds>("/oscc_interfacing/wheel_speeds", rclcpp::QoS(1));
  steering_angle_pub_ =
    this->create_publisher<roscco_msg::msg::SteeringAngle>("/oscc_interfacing/steering_angle", rclcpp::QoS(1));

  // Default group: is_armed status timer
  std::chrono::milliseconds interval(1000 / is_armed_publish_rate_hz);
  is_armed_timer_ = this->create_wall_timer(
    interval, std::bind(&OsccInterfacingNode::is_armed_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "OsccInterfacingNode configured: armed=%d, is_armed_publish_rate=%d Hz",
    is_armed_,
    is_armed_publish_rate_hz);

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

// --- Group A: OSCC API callbacks (mutually exclusive) ---

void OsccInterfacingNode::roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg)
{
  // No arm_mutex_ needed — Group A serialization guarantees arm_service_callback
  // and process_events cannot run concurrently with this callback
  if (!is_armed_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Vehicle not armed, Ignoring roscco message");
    return;
  }

  float forward = msg->forward;
  float steering = msg->steering;

  if (std::abs(forward) > 1.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Forward command out of range [-1, 1], this should not "
      "happen! Ignoring message.");
    return;
  }
  if (std::abs(steering) > 1.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Steering command out of range [-1, 1], this should not "
      "happen! Ignoring message.");
    return;
  }

  float brake = 0.0;
  float throttle = 0.0;

  if (forward >= 0.0) {
    throttle = forward;
    brake = 0.0;
  } else {
    throttle = 0.0;
    brake = -forward;
  }

  // Smooth transition ordering between throttle and brake
  if (forward > 0.0 && last_forward_ < 0.0) {
    handle_any_errors(oscc_publish_brake_position(brake));
    handle_any_errors(oscc_publish_throttle_position(throttle));
  } else {
    handle_any_errors(oscc_publish_throttle_position(throttle));
    handle_any_errors(oscc_publish_brake_position(brake));
  }

  // Steering with deadzone compensation
  float steering_torque = steering * steering_scaling_;
  if (steering_torque > 0.0) {
    steering_torque += steering_torque_deadzone_pos_;
  } else if (steering_torque < 0.0) {
    steering_torque -= steering_torque_deadzone_neg_;
  }
  handle_any_errors(oscc_publish_steering_torque(steering_torque));

  last_forward_ = forward;
}

void OsccInterfacingNode::arm_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    bool all_ok = true;
    std::string enabled_modules;

    if (enable_all_) {
      if (oscc_enable() != OSCC_OK) {
        all_ok = false;
      } else {
        enabled_modules = "all";
      }
    } else {
      if (enable_steering_) {
        if (oscc_enable_steering() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to enable steering module");
        } else {
          enabled_modules += "steering ";
        }
      }
      if (enable_throttle_) {
        if (oscc_enable_throttle() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to enable throttle module");
        } else {
          enabled_modules += "throttle ";
        }
      }
      if (enable_brakes_) {
        if (oscc_enable_brakes() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to enable brake module");
        } else {
          enabled_modules += "brakes ";
        }
      }
    }

    if (all_ok) {
      is_armed_ = true;
      response->success = true;
      response->message = "Vehicle armed successfully (" + enabled_modules + ")";
      RCLCPP_INFO(get_logger(), "Vehicle armed: %s", enabled_modules.c_str());
    } else {
      response->success = false;
      response->message = "Failed to arm vehicle";
      RCLCPP_ERROR(get_logger(), "Failed to arm vehicle");
    }
  } else {
    bool all_ok = true;
    std::string disabled_modules;

    if (enable_all_) {
      if (oscc_disable() != OSCC_OK) {
        all_ok = false;
      } else {
        disabled_modules = "all";
      }
    } else {
      if (enable_steering_) {
        if (oscc_disable_steering() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to disable steering module");
        } else {
          disabled_modules += "steering ";
        }
      }
      if (enable_throttle_) {
        if (oscc_disable_throttle() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to disable throttle module");
        } else {
          disabled_modules += "throttle ";
        }
      }
      if (enable_brakes_) {
        if (oscc_disable_brakes() != OSCC_OK) {
          all_ok = false;
          RCLCPP_ERROR(get_logger(), "Failed to disable brake module");
        } else {
          disabled_modules += "brakes ";
        }
      }
    }

    if (all_ok) {
      is_armed_ = false;
      response->success = true;
      response->message = "Vehicle disarmed successfully (" + disabled_modules + ")";
      RCLCPP_INFO(get_logger(), "Vehicle disarmed: %s", disabled_modules.c_str());
    } else {
      response->success = false;
      response->message = "Failed to disarm vehicle";
      RCLCPP_FATAL(get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void OsccInterfacingNode::process_events()
{
  // Handle operator override
  if (has_override_.exchange(false)) {
    OverrideType override_type = latest_override_.load();
    if (override_type == OverrideType::BRAKE) {
      RCLCPP_INFO(get_logger(), "Brake Operator Override");
    } else if (override_type == OverrideType::THROTTLE) {
      RCLCPP_INFO(get_logger(), "Throttle Operator Override");
    } else if (override_type == OverrideType::STEERING) {
      RCLCPP_INFO(get_logger(), "Steering Operator Override");
    }

    if (oscc_disable() == OSCC_OK) {
      is_armed_ = false;
      RCLCPP_INFO(get_logger(), "Vehicle disarmed");
    } else {
      RCLCPP_FATAL(get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }

  // Handle fault
  if (has_fault_.exchange(false)) {
    FaultType fault_type = latest_fault_.load();
    if (fault_type == FaultType::BRAKE_FAULT) {
      RCLCPP_INFO(get_logger(), "Brake Fault");
    } else if (fault_type == FaultType::STEERING_FAULT) {
      RCLCPP_INFO(get_logger(), "Steering Fault");
    } else if (fault_type == FaultType::THROTTLE_FAULT) {
      RCLCPP_INFO(get_logger(), "Throttle Fault");
    }

    if (disable_boards_on_fault_) {
      if (oscc_disable() == OSCC_OK) {
        is_armed_ = false;
        RCLCPP_INFO(get_logger(), "Vehicle disarmed due to fault");
      } else {
        RCLCPP_FATAL(get_logger(), "!!!!!! Failed to disarm vehicle after fault");
      }
    }
  }
}

// --- Group B: Feedback publishing (independent of OSCC API) ---

void OsccInterfacingNode::publish_feedback()
{
  // Publish wheel speeds — lock to get a consistent snapshot
  if (has_wheel_data_.exchange(false)) {
    WheelSpeedSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(wheel_data_mutex_);
      snapshot = latest_wheel_data_;
    }

    roscco_msg::msg::WheelSpeeds msg;
    msg.ne = snapshot.ne;
    msg.nw = snapshot.nw;
    msg.se = snapshot.se;
    msg.sw = snapshot.sw;
    msg.header.stamp = this->now();
    wheel_speeds_pub_->publish(msg);
  }

  // Publish steering angle
  if (has_steering_data_.exchange(false)) {
    float angle = latest_steering_angle_.load();
    angle = angle / steering_conversion_factor_;
    angle = angle * (M_PI / 180.0);

    roscco_msg::msg::SteeringAngle msg;
    msg.angle = angle;
    msg.header.stamp = this->now();
    steering_angle_pub_->publish(msg);
  }
}

// --- Default group ---

void OsccInterfacingNode::is_armed_timer_callback()
{
  std_msgs::msg::Bool msg;
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    msg.data = is_armed_;
  }
  is_armed_pub_->publish(msg);
}

// --- Error handling ---

oscc_result_t OsccInterfacingNode::handle_any_errors(oscc_result_t result)
{
  if (result == OSCC_OK) {
    return OSCC_OK;
  }

  is_armed_ = false;

  RCLCPP_ERROR(this->get_logger(), "Error from OSCC API: %d, ATTEMPTING TO DISARM ALL BOARDS", result);
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
  return result;
}

}  // namespace oscc_interfacing
