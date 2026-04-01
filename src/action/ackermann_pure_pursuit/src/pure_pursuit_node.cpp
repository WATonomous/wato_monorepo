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

#include "ackermann_pure_pursuit/pure_pursuit_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ackermann_pure_pursuit
{

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pure_pursuit_node", options)
, last_trajectory_time_(0, 0, RCL_ROS_TIME)
{
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("bt_topic", "execute_behaviour");
  declare_parameter("ackermann_topic", "/action/ackermann");
  declare_parameter("idle_topic", "/action/is_idle");
  declare_parameter("base_frame", "base_footprint");
  declare_parameter("rear_axle_frame", "rear_axle");
  declare_parameter("front_axle_frame", "front_axle");
  declare_parameter("standby_msg", "standby");
  declare_parameter("lookahead_distance", 5.0);
  declare_parameter("min_lookahead_distance", 2.0);
  declare_parameter("steering_angle_gain", 1.0);
  declare_parameter("max_speed", 5.0);
  declare_parameter("min_speed", 0.5);
  declare_parameter("standby_speed", 0.0);
  declare_parameter("standby_steering", 0.0);
  declare_parameter("control_rate_hz", 20.0);
  declare_parameter("wheelbase_fallback", 2.5667);
  declare_parameter("max_steering_angle", 0.5);
  declare_parameter("idle_timeout_sec", 2.0);
  declare_parameter("invert_steering", false);
  declare_parameter("disable_standby", false);
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  bt_topic_ = get_parameter("bt_topic").as_string();
  ackermann_topic_ = get_parameter("ackermann_topic").as_string();
  idle_topic_ = get_parameter("idle_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  rear_axle_frame_ = get_parameter("rear_axle_frame").as_string();
  front_axle_frame_ = get_parameter("front_axle_frame").as_string();
  standby_msg_ = get_parameter("standby_msg").as_string();
  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
  steering_angle_gain = get_parameter("steering_angle_gain").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  min_speed_ = get_parameter("min_speed").as_double();
  standby_speed_ = get_parameter("standby_speed").as_double();
  standby_steering_ = get_parameter("standby_steering").as_double();
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  wheelbase_fallback_ = get_parameter("wheelbase_fallback").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  idle_timeout_sec_ = get_parameter("idle_timeout_sec").as_double();
  invert_steering_ = get_parameter("invert_steering").as_bool();
  disable_standby_ = get_parameter("disable_standby").as_bool();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  ackermann_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic_, rclcpp::QoS(10));

  idle_pub_ = create_publisher<std_msgs::msg::Bool>(idle_topic_, rclcpp::QoS(10));

  trajectory_sub_ = create_subscription<wato_trajectory_msgs::msg::Trajectory>(
    trajectory_topic_, rclcpp::QoS(10), std::bind(&PurePursuitNode::trajectoryCallback, this, std::placeholders::_1));
  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    bt_topic_, rclcpp::QoS(10), std::bind(&PurePursuitNode::bt_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured: control at %.1f Hz", control_rate_hz_);
  return CallbackReturn::SUCCESS;
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  ackermann_pub_->on_activate();
  idle_pub_->on_activate();

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&PurePursuitNode::controlCallback, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  control_timer_.reset();
  ackermann_pub_->on_deactivate();
  idle_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  ackermann_pub_.reset();
  idle_pub_.reset();
  trajectory_sub_.reset();
  bt_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  latest_trajectory_.reset();
  wheelbase_cached_ = 0.0;

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  control_timer_.reset();
  ackermann_pub_.reset();
  idle_pub_.reset();
  trajectory_sub_.reset();
  bt_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void PurePursuitNode::trajectoryCallback(const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg)
{
  latest_trajectory_ = msg;
  last_trajectory_time_ = now();
}

double PurePursuitNode::getWheelbase()
{
  if (wheelbase_cached_ > 0.0) {
    return wheelbase_cached_;
  }

  try {
    auto tf = tf_buffer_->lookupTransform(rear_axle_frame_, front_axle_frame_, tf2::TimePointZero);
    double dx = tf.transform.translation.x;
    double dy = tf.transform.translation.y;
    double dz = tf.transform.translation.z;
    double wb = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (wb > 0.1) {
      wheelbase_cached_ = wb;
      RCLCPP_INFO(get_logger(), "Wheelbase from TF: %.4f m", wb);
      return wb;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Cannot get wheelbase from TF (%s → %s): %s. Using fallback: %.4f m",
      rear_axle_frame_.c_str(),
      front_axle_frame_.c_str(),
      ex.what(),
      wheelbase_fallback_);
  }

  return wheelbase_fallback_;
}

void PurePursuitNode::bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg)
{
  bt_requested_behaviour = msg->behaviour;
}

void PurePursuitNode::controlCallback()
{
  std_msgs::msg::Bool idle_msg;

  // Check for stale or missing trajectory
  bool is_idle = !latest_trajectory_ || latest_trajectory_->points.empty() ||
                 (now() - last_trajectory_time_).seconds() > idle_timeout_sec_ || bt_requested_behaviour.empty();

  if (is_idle || (!disable_standby_ && bt_requested_behaviour == standby_msg_)) {
    idle_msg.data = true;
    idle_pub_->publish(idle_msg);
    publishAckermannMsg(base_frame_, standby_speed_, standby_steering_, invert_steering_);
    return;
  }

  idle_msg.data = false;
  idle_pub_->publish(idle_msg);

  const auto & traj = *latest_trajectory_;
  double wheelbase = getWheelbase();

  // Transform trajectory points into base_frame and find lookahead point
  double lookahead_x = 0.0;
  double lookahead_y = 0.0;
  double target_speed = max_speed_;
  bool found_lookahead = false;

  for (const auto & pt : traj.points) {
    geometry_msgs::msg::PoseStamped pose_in_traj;
    pose_in_traj.header = traj.header;
    pose_in_traj.pose = pt.pose;

    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      pose_in_base = tf_buffer_->transform(pose_in_traj, base_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform trajectory point to base frame: %s", ex.what());
      return;
    }

    double dx = pose_in_base.pose.position.x;
    double dy = pose_in_base.pose.position.y;
    double dist = std::hypot(dx, dy);

    // Only consider points ahead of the vehicle (positive x in base frame)
    if (dx > 0.0 && dist >= min_lookahead_distance_) {
      if (dist >= lookahead_distance_ || &pt == &traj.points.back()) {
        lookahead_x = dx;
        lookahead_y = dy;
        target_speed = pt.max_speed;
        found_lookahead = true;
        break;
      }
    }
  }

  if (!found_lookahead) {
    return;
  }

  // Pure pursuit math
  double ld_sq = lookahead_x * lookahead_x + lookahead_y * lookahead_y;
  double curvature = 2.0 * lookahead_y / ld_sq;
  double steering_angle = steering_angle_gain * std::atan(wheelbase * curvature);

  // Clamp steering
  steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

  // Reduce speed proportional to steering magnitude
  double steering_ratio = std::abs(steering_angle) / max_steering_angle_;
  double speed = target_speed * (1.0 - 0.5 * steering_ratio);
  speed = std::clamp(speed, min_speed_, max_speed_);

  // If target trajectory point says stop, stop
  if (target_speed <= 0.0) {
    speed = 0.0;
  }

  publishAckermannMsg(base_frame_, speed, steering_angle, invert_steering_);
}

void PurePursuitNode::publishAckermannMsg(
  std::string frame_id, double speed, double steering_angle, bool invert_steering)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.steering_angle = invert_steering ? -steering_angle : steering_angle;
  cmd.drive.speed = speed;

  ackermann_pub_->publish(cmd);
}

}  // namespace ackermann_pure_pursuit
