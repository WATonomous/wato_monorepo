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

#include "can_state_estimator/can_state_estimator_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

static constexpr canid_t STEERING_ANGLE_CAN_ID = 0x2B0;
static constexpr canid_t WHEEL_SPEED_CAN_ID = 0x4B0;
static constexpr double STEERING_ANGLE_SCALAR = 0.1;
static constexpr double KPH_TO_MPS = 1.0 / 3.6;

namespace can_state_estimator
{

CanStateEstimatorNode::CanStateEstimatorNode(const rclcpp::NodeOptions & options)
: LifecycleNode("can_state_estimator_node", options)
{
  this->declare_parameter<std::string>("can_interface", "can1");
  this->declare_parameter<double>("steering_conversion_factor", 15.7);
  this->declare_parameter<std::string>("rear_axle_frame", "rear_axle");
  this->declare_parameter<std::string>("front_axle_frame", "front_axle");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");

  RCLCPP_INFO(this->get_logger(), "CAN State Estimator Node initialized.");
}

CanStateEstimatorNode::~CanStateEstimatorNode()
{
  stop_can_thread();
  close_can_socket();
}

CanStateEstimatorNode::CallbackReturn CanStateEstimatorNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring CAN State Estimator Node");

  // Read parameters
  can_interface_ = this->get_parameter("can_interface").as_string();
  steering_conversion_factor_ = this->get_parameter("steering_conversion_factor").as_double();
  rear_axle_frame_ = this->get_parameter("rear_axle_frame").as_string();
  front_axle_frame_ = this->get_parameter("front_axle_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  steering_pub_ =
    this->create_publisher<roscco_msg::msg::SteeringAngle>("can_state_estimator/steering_angle", rclcpp::QoS(1));
  velocity_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("can_state_estimator/body_velocity", rclcpp::SystemDefaultsQoS());
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("can_state_estimator/odom", rclcpp::QoS(10));

  // Open SocketCAN
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN socket");
    return CallbackReturn::FAILURE;
  }

  // Kernel-level filter: only deliver steering (0x2B0) and wheel speed (0x4B0) frames
  struct can_filter filters[2];
  filters[0].can_id = STEERING_ANGLE_CAN_ID;
  filters[0].can_mask = CAN_SFF_MASK;
  filters[1].can_id = WHEEL_SPEED_CAN_ID;
  filters[1].can_mask = CAN_SFF_MASK;
  setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to find CAN interface %s", can_interface_.c_str());
    close(sock_);
    sock_ = -1;
    return CallbackReturn::FAILURE;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket to %s", can_interface_.c_str());
    close(sock_);
    sock_ = -1;
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(),
    "CAN State Estimator configured on %s (steering=0x%X, wheel_speed=0x%X)",
    can_interface_.c_str(),
    STEERING_ANGLE_CAN_ID,
    WHEEL_SPEED_CAN_ID);

  return CallbackReturn::SUCCESS;
}

CanStateEstimatorNode::CallbackReturn CanStateEstimatorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating CAN State Estimator Node");

  steering_pub_->on_activate();
  velocity_pub_->on_activate();
  odom_pub_->on_activate();

  // Reset odometry state
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;
  odom_initialized_ = false;
  has_wheelbase_ = false;

  running_.store(true);
  read_thread_ = std::thread(&CanStateEstimatorNode::read_loop, this);

  return CallbackReturn::SUCCESS;
}

CanStateEstimatorNode::CallbackReturn CanStateEstimatorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating CAN State Estimator Node");

  stop_can_thread();

  steering_pub_->on_deactivate();
  velocity_pub_->on_deactivate();
  odom_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CanStateEstimatorNode::CallbackReturn CanStateEstimatorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up CAN State Estimator Node");

  close_can_socket();
  steering_pub_.reset();
  velocity_pub_.reset();
  odom_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return CallbackReturn::SUCCESS;
}

CanStateEstimatorNode::CallbackReturn CanStateEstimatorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down CAN State Estimator Node");

  stop_can_thread();
  close_can_socket();
  steering_pub_.reset();
  velocity_pub_.reset();
  odom_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return CallbackReturn::SUCCESS;
}

void CanStateEstimatorNode::stop_can_thread()
{
  running_.store(false);
  if (sock_ >= 0) {
    shutdown(sock_, SHUT_RDWR);
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void CanStateEstimatorNode::close_can_socket()
{
  if (sock_ >= 0) {
    close(sock_);
    sock_ = -1;
  }
}

bool CanStateEstimatorNode::lookup_wheelbase()
{
  if (has_wheelbase_) {
    return true;
  }

  try {
    auto tf = tf_buffer_->lookupTransform(rear_axle_frame_, front_axle_frame_, tf2::TimePointZero);
    double dx = tf.transform.translation.x;
    double dy = tf.transform.translation.y;
    wheelbase_ = std::hypot(dx, dy);
    has_wheelbase_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Wheelbase from TF (%s -> %s): %.4f m",
      rear_axle_frame_.c_str(),
      front_axle_frame_.c_str(),
      wheelbase_);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for %s -> %s TF to determine wheelbase: %s",
      rear_axle_frame_.c_str(),
      front_axle_frame_.c_str(),
      ex.what());
    return false;
  }
}

void CanStateEstimatorNode::read_loop()
{
  struct can_frame frame;

  while (running_.load(std::memory_order_relaxed)) {
    ssize_t nbytes = read(sock_, &frame, sizeof(frame));
    if (nbytes < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (nbytes != sizeof(frame)) {
      continue;
    }

    if (frame.can_id == STEERING_ANGLE_CAN_ID) {
      process_steering_frame(frame.data);
    } else if (frame.can_id == WHEEL_SPEED_CAN_ID) {
      process_wheel_speed_frame(frame.data);
    }
  }
}

void CanStateEstimatorNode::process_steering_frame(const uint8_t * data)
{
  // int16_t little-endian from bytes 0-1, scale by 0.1, negate
  int16_t raw = static_cast<int16_t>((data[1] << 8) | data[0]);
  double steering_wheel_angle_deg = -(static_cast<double>(raw) * STEERING_ANGLE_SCALAR);

  // Convert steering wheel angle to wheel angle in radians
  double angle_rad = (steering_wheel_angle_deg / steering_conversion_factor_) * (M_PI / 180.0);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_steering_angle_rad_ = angle_rad;
    has_steering_angle_ = true;
  }

  if (!steering_pub_->is_activated()) {
    return;
  }

  roscco_msg::msg::SteeringAngle msg;
  msg.angle = static_cast<float>(angle_rad);
  msg.header.stamp = this->now();
  steering_pub_->publish(msg);
}

void CanStateEstimatorNode::process_wheel_speed_frame(const uint8_t * data)
{
  // 12-bit unsigned values at 2-byte offsets, same decoding as OSCC:
  //   raw = ((data[offset+1] & 0x0F) << 8) | data[offset]
  //   speed_kph = (int)(raw / 3.2) / 10.0
  auto decode_wheel = [](const uint8_t * d, size_t offset) -> double {
    uint16_t raw = ((d[offset + 1] & 0x0F) << 8) | d[offset];
    return static_cast<double>(static_cast<int>(static_cast<double>(raw) / 3.2)) / 10.0;
  };

  double nw = decode_wheel(data, 0);  // left front
  double ne = decode_wheel(data, 2);  // right front
  double sw = decode_wheel(data, 4);  // left rear
  double se = decode_wheel(data, 6);  // right rear

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    wheel_speed_nw_ = nw;
    wheel_speed_ne_ = ne;
    wheel_speed_sw_ = sw;
    wheel_speed_se_ = se;
    has_wheel_speeds_ = true;
  }

  publish_velocity_and_odom();
}

void CanStateEstimatorNode::publish_velocity_and_odom()
{
  if (!velocity_pub_->is_activated()) {
    return;
  }

  if (!lookup_wheelbase()) {
    return;
  }

  double steering_angle;
  double nw, ne;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_steering_angle_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Waiting for steering angle from CAN to compute velocity/odom...");
      return;
    }
    steering_angle = current_steering_angle_rad_;
    nw = wheel_speed_nw_;
    ne = wheel_speed_ne_;
  }

  rclcpp::Time now = this->now();

  // Average front wheel speeds, convert to m/s
  double v_front_avg_mps = ((nw + ne) / 2.0) * KPH_TO_MPS;

  // Body velocity via Ackermann bicycle model (rear axle reference)
  double v_body = v_front_avg_mps * std::cos(steering_angle);

  // Publish body velocity
  std_msgs::msg::Float64 vel_msg;
  vel_msg.data = v_body;
  velocity_pub_->publish(vel_msg);

  // Odometry integration
  if (!odom_initialized_) {
    last_odom_time_ = now;
    odom_initialized_ = true;
    return;
  }

  double dt = (now - last_odom_time_).seconds();
  last_odom_time_ = now;

  if (dt <= 0.0 || dt > 1.0) {
    return;  // Skip bogus dt
  }

  // Ackermann bicycle model: yaw rate = v_body * tan(steering_angle) / wheelbase
  double omega = v_body * std::tan(steering_angle) / wheelbase_;

  // Integrate pose
  odom_x_ += v_body * std::cos(odom_theta_) * dt;
  odom_y_ += v_body * std::sin(odom_theta_) * dt;
  odom_theta_ += omega * dt;

  // Publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Yaw to quaternion (2D rotation about Z)
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(odom_theta_ / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(odom_theta_ / 2.0);

  odom_msg.twist.twist.linear.x = v_body;
  odom_msg.twist.twist.angular.z = omega;

  odom_pub_->publish(odom_msg);
}

}  // namespace can_state_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_state_estimator::CanStateEstimatorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
