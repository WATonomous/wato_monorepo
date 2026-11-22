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
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ackermann_pure_pursuit
{

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pure_pursuit_node", options)
, last_trajectory_time_(0, 0, RCL_ROS_TIME)
{
  // Topics / frames
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("bt_topic", "execute_behaviour");
  declare_parameter("ackermann_topic", "/action/ackermann");
  declare_parameter("idle_topic", "/action/is_idle");
  declare_parameter("controller_status_topic", "controller_status");
  declare_parameter("odom_topic", "odom");
  declare_parameter("base_frame", "base_footprint");
  declare_parameter("rear_axle_frame", "rear_axle");
  declare_parameter("front_axle_frame", "front_axle");
  declare_parameter("standby_msg", "standby");

  // Lookahead (Phase 2)
  declare_parameter("lookahead_distance", 5.0);
  declare_parameter("min_lookahead_distance", 2.0);
  declare_parameter("lookahead_time", 1.5);
  declare_parameter("curvature_lookahead_gain", 2.0);
  declare_parameter("curvature_est_arc", 1.5);
  declare_parameter("speed_lookahead_distance", 1.0);
  declare_parameter("speed_lookahead_time", 2.0);

  // Steering
  declare_parameter("steering_angle_gain", 1.0);
  declare_parameter("max_steering_angle", 0.5);
  declare_parameter("max_steering_rate", 1.0);

  // Speed (Phase 3)
  declare_parameter("max_speed", 5.0);
  declare_parameter("min_speed", 0.5);
  declare_parameter("max_lateral_accel", 2.5);
  declare_parameter("max_accel", 1.5);
  declare_parameter("max_decel", 3.0);

  // Stanley low-speed blend (Phase 3d)
  declare_parameter("enable_stanley_blend", false);
  declare_parameter("stanley_gain", 0.5);
  declare_parameter("stanley_softening", 1.0);
  declare_parameter("stanley_speed_threshold", 2.0);

  // Disengage monitor (Phase 4)
  declare_parameter("enable_disengage", true);
  declare_parameter("max_cross_track_error", 1.5);
  declare_parameter("max_heading_error", 0.6);
  declare_parameter("disengage_debounce_sec", 0.5);
  declare_parameter("disengage_latch", false);
  declare_parameter("disengage_speed", 0.0);

  // Standby / misc
  declare_parameter("standby_speed", 0.0);
  declare_parameter("standby_steering", 0.0);
  declare_parameter("control_rate_hz", 20.0);
  declare_parameter("wheelbase_fallback", 2.5667);
  declare_parameter("idle_timeout_sec", 2.0);
  declare_parameter("invert_steering", false);
  declare_parameter("disable_standby", false);
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  ackermann_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic_, rclcpp::QoS(10));

  idle_pub_ = create_publisher<std_msgs::msg::Bool>(idle_topic_, rclcpp::QoS(10));

  status_pub_ =
    create_publisher<wato_trajectory_msgs::msg::ControllerStatus>(controller_status_topic_, rclcpp::QoS(10));

  trajectory_sub_ = create_subscription<wato_trajectory_msgs::msg::Trajectory>(
    trajectory_topic_, rclcpp::QoS(10), std::bind(&PurePursuitNode::trajectoryCallback, this, std::placeholders::_1));
  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    bt_topic_, rclcpp::QoS(10), std::bind(&PurePursuitNode::bt_callback, this, std::placeholders::_1));

  std::string odom_topic = get_parameter("odom_topic").as_string();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) {
      current_speed_ = msg->twist.twist.linear.x;
    });

  RCLCPP_INFO(
    get_logger(), "Configured: control at %.1f Hz, reference frame '%s'", control_rate_hz_, rear_axle_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

void PurePursuitNode::loadParameters()
{
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  bt_topic_ = get_parameter("bt_topic").as_string();
  ackermann_topic_ = get_parameter("ackermann_topic").as_string();
  idle_topic_ = get_parameter("idle_topic").as_string();
  controller_status_topic_ = get_parameter("controller_status_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  rear_axle_frame_ = get_parameter("rear_axle_frame").as_string();
  front_axle_frame_ = get_parameter("front_axle_frame").as_string();
  standby_msg_ = get_parameter("standby_msg").as_string();

  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
  lookahead_time_ = get_parameter("lookahead_time").as_double();
  curvature_lookahead_gain_ = get_parameter("curvature_lookahead_gain").as_double();
  curvature_est_arc_ = get_parameter("curvature_est_arc").as_double();
  speed_lookahead_distance_ = get_parameter("speed_lookahead_distance").as_double();
  speed_lookahead_time_ = get_parameter("speed_lookahead_time").as_double();

  steering_angle_gain_ = get_parameter("steering_angle_gain").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  max_steering_rate_ = get_parameter("max_steering_rate").as_double();

  max_speed_ = get_parameter("max_speed").as_double();
  min_speed_ = get_parameter("min_speed").as_double();
  max_lateral_accel_ = get_parameter("max_lateral_accel").as_double();
  max_accel_ = get_parameter("max_accel").as_double();
  max_decel_ = get_parameter("max_decel").as_double();

  enable_stanley_blend_ = get_parameter("enable_stanley_blend").as_bool();
  stanley_gain_ = get_parameter("stanley_gain").as_double();
  stanley_softening_ = get_parameter("stanley_softening").as_double();
  stanley_speed_threshold_ = get_parameter("stanley_speed_threshold").as_double();

  enable_disengage_ = get_parameter("enable_disengage").as_bool();
  max_cross_track_error_ = get_parameter("max_cross_track_error").as_double();
  max_heading_error_ = get_parameter("max_heading_error").as_double();
  disengage_debounce_sec_ = get_parameter("disengage_debounce_sec").as_double();
  disengage_latch_ = get_parameter("disengage_latch").as_bool();
  disengage_speed_ = get_parameter("disengage_speed").as_double();

  standby_speed_ = get_parameter("standby_speed").as_double();
  standby_steering_ = get_parameter("standby_steering").as_double();
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  wheelbase_fallback_ = get_parameter("wheelbase_fallback").as_double();
  idle_timeout_sec_ = get_parameter("idle_timeout_sec").as_double();
  invert_steering_ = get_parameter("invert_steering").as_bool();
  disable_standby_ = get_parameter("disable_standby").as_bool();
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  ackermann_pub_->on_activate();
  idle_pub_->on_activate();
  status_pub_->on_activate();

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
  status_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

PurePursuitNode::CallbackReturn PurePursuitNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  ackermann_pub_.reset();
  idle_pub_.reset();
  status_pub_.reset();
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
  status_pub_.reset();
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
  bt_requested_behaviour_ = msg->behaviour;
}

bool PurePursuitNode::buildPathInReferenceFrame(
  const wato_trajectory_msgs::msg::Trajectory & traj,
  std::vector<math::Vec2> & path_out,
  std::vector<double> & speeds_out)
{
  // One TF lookup per cycle (trajectory frame -> controller reference frame),
  // applied to every point. Fails atomically if the transform is unavailable.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(rear_axle_frame_, traj.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Cannot transform trajectory to %s: %s", rear_axle_frame_.c_str(), ex.what());
    return false;
  }

  path_out.clear();
  speeds_out.clear();
  path_out.reserve(traj.points.size());
  speeds_out.reserve(traj.points.size());
  for (const auto & pt : traj.points) {
    geometry_msgs::msg::Pose p;
    tf2::doTransform(pt.pose, p, tf);
    path_out.push_back(math::Vec2{p.position.x, p.position.y});
    speeds_out.push_back(pt.max_speed);
  }
  return true;
}

bool PurePursuitNode::updateDisengage(bool over, const std::string & reason, const rclcpp::Time & t_now)
{
  if (!enable_disengage_) {
    over_threshold_active_ = false;
    disengaged_latched_ = false;
    disengage_reason_.clear();
    return false;
  }

  if (over) {
    if (!over_threshold_active_) {
      over_threshold_active_ = true;
      over_threshold_since_ = t_now;
    }
    disengage_reason_ = reason;
    // Trip only after the error has been sustained past the debounce window.
    if ((t_now - over_threshold_since_).seconds() >= disengage_debounce_sec_) {
      disengaged_latched_ = true;
    }
  } else {
    over_threshold_active_ = false;
    if (!disengage_latch_) {
      disengaged_latched_ = false;
      disengage_reason_.clear();
    }
  }
  return disengaged_latched_;
}

void PurePursuitNode::publishStandby()
{
  std_msgs::msg::Bool idle_msg;
  idle_msg.data = true;
  idle_pub_->publish(idle_msg);
  publishAckermannMsg(base_frame_, standby_speed_, standby_steering_, invert_steering_);

  wato_trajectory_msgs::msg::ControllerStatus status;
  status.header.stamp = now();
  status.header.frame_id = rear_axle_frame_;
  status.commanded_speed = standby_speed_;
  status.commanded_steering = invert_steering_ ? -standby_steering_ : standby_steering_;
  status.disengaged = false;
  status.reason = "idle";
  status_pub_->publish(status);

  // Reset command-shaping state so re-engagement starts smoothly from standby.
  prev_speed_cmd_ = standby_speed_;
  prev_steering_cmd_ = standby_steering_;
  last_control_time_ = status.header.stamp;
  have_last_control_ = true;
}

void PurePursuitNode::controlCallback()
{
  // --- Idle / standby gate ---
  bool is_idle = !latest_trajectory_ || latest_trajectory_->points.empty() ||
                 (now() - last_trajectory_time_).seconds() > idle_timeout_sec_ || bt_requested_behaviour_.empty();

  if (is_idle || (!disable_standby_ && bt_requested_behaviour_ == standby_msg_)) {
    publishStandby();
    return;
  }

  const auto & traj = *latest_trajectory_;
  const double wheelbase = getWheelbase();

  // --- Transform trajectory into the controller reference frame (rear axle) once ---
  std::vector<math::Vec2> path;
  std::vector<double> speeds;
  if (!buildPathInReferenceFrame(traj, path, speeds) || path.size() < 2) {
    publishStandby();  // cannot evaluate this cycle: hold safe
    return;
  }

  // --- Cycle timing for slew / rate limiting ---
  const rclcpp::Time t_now = now();
  double dt = 1.0 / control_rate_hz_;
  if (have_last_control_) {
    double measured = (t_now - last_control_time_).seconds();
    if (measured > 1e-3 && measured < 1.0) {
      dt = measured;
    }
  }

  const double v = std::max(0.0, current_speed_);

  // --- Tracking error (Phase 1) ---
  math::TrackingError err = math::computeTrackingError(path);

  // Anchor forward searches at the nearest path segment so a self-crossing or
  // looping path cannot match a target on an earlier lobe.
  const std::size_t search_start = err.valid ? err.nearest_idx : 0;

  // --- Adaptive lookahead (Phase 2) ---
  // Time-headway schedule, clamped to [min, max].
  double ld = std::clamp(lookahead_time_ * v, min_lookahead_distance_, lookahead_distance_);
  // Estimate upcoming curvature at a preliminary lookahead, then shrink ld in curves.
  math::LookaheadResult prelim = math::findLookaheadPoint(path, ld, min_lookahead_distance_, search_start);
  double kappa_ahead = prelim.found ? math::curvatureByArc(path, prelim.segment_idx, curvature_est_arc_) : 0.0;
  double ld_eff = ld / (1.0 + curvature_lookahead_gain_ * std::abs(kappa_ahead));
  ld_eff = std::clamp(ld_eff, min_lookahead_distance_, lookahead_distance_);
  math::LookaheadResult look = math::findLookaheadPoint(path, ld_eff, min_lookahead_distance_, search_start);

  // --- Desired steering / speed for this cycle ---
  double desired_steering = 0.0;
  double desired_speed = 0.0;
  bool over = false;
  std::string reason = "ok";

  if (!look.found) {
    // Ran off the end of the path / nothing ahead: treat as a tracking failure.
    over = true;
    reason = "no_lookahead_point";
    desired_steering = 0.0;
    desired_speed = disengage_speed_;
  } else {
    kappa_ahead = math::curvatureByArc(path, look.segment_idx, curvature_est_arc_);

    // Pure pursuit steering about the rear axle.
    double pp_curv = math::pursuitCurvature(look.point);
    desired_steering = steering_angle_gain_ * std::atan(wheelbase * pp_curv);

    // Optional low-speed cross-track feedback (Phase 3d): ramps in below the threshold speed.
    if (enable_stanley_blend_ && err.valid) {
      double blend = std::clamp(1.0 - v / std::max(1e-3, stanley_speed_threshold_), 0.0, 1.0);
      desired_steering +=
        blend * math::crossTrackSteerCorrection(err.cross_track, v, stanley_gain_, stanley_softening_);
    }
    desired_steering = std::clamp(desired_steering, -max_steering_angle_, max_steering_angle_);

    // Target speed: the minimum commanded speed over a decoupled, speed-scaled
    // horizon (Phase 2d), so a stop inside the horizon (or at the path end) is
    // never skipped over.
    double speed_horizon = std::max(speed_lookahead_distance_, speed_lookahead_time_ * v);
    double path_speed = math::minSpeedWithinHorizon(path, speeds, speed_horizon, max_speed_, search_start);

    // Curvature-limited speed (Phase 3a): keep lateral accel within budget.
    double v_curve = math::curvatureLimitedSpeed(max_lateral_accel_, kappa_ahead, max_speed_);
    if (path_speed <= 0.0) {
      desired_speed = 0.0;  // path commands a stop
    } else {
      desired_speed = std::clamp(std::min(path_speed, v_curve), min_speed_, max_speed_);
    }

    // Tracking-threshold checks feed the disengage monitor.
    if (err.valid && std::abs(err.cross_track) > max_cross_track_error_) {
      over = true;
      reason = "cte_exceeded";
    } else if (err.valid && std::abs(err.heading) > max_heading_error_) {
      over = true;
      reason = "heading_exceeded";
    }
  }

  // --- Disengage monitor (Phase 4): advisory only ---
  // Detects (debounced) that tracking has degraded and reports it via
  // ControllerStatus.disengaged. It deliberately does NOT alter the command or mask
  // the controller out of the mux — acting on the recommendation (driver takeover /
  // OSCC disable) is left to a higher-level supervisor.
  bool disengaged = updateDisengage(over, reason, t_now);

  // --- Command shaping (Phase 3b/3c): accel/decel and steering-rate slew limits ---
  double steering = desired_steering;
  double speed = desired_speed;
  if (have_last_control_) {
    steering = math::slewLimit(prev_steering_cmd_, desired_steering, max_steering_rate_ * dt);
    if (desired_speed >= prev_speed_cmd_) {
      speed = std::min(desired_speed, prev_speed_cmd_ + max_accel_ * dt);
    } else {
      speed = std::max(desired_speed, prev_speed_cmd_ - max_decel_ * dt);
    }
  }

  // --- Publish command + idle + status ---
  // Actively tracking: not idle. The disengage recommendation is advisory (see below)
  // and must not mask the controller, so is_idle stays false here regardless.
  std_msgs::msg::Bool idle_msg;
  idle_msg.data = false;
  idle_pub_->publish(idle_msg);

  publishAckermannMsg(base_frame_, speed, steering, invert_steering_);

  wato_trajectory_msgs::msg::ControllerStatus status;
  status.header.stamp = t_now;
  status.header.frame_id = rear_axle_frame_;
  status.cross_track_error = err.valid ? err.cross_track : 0.0;
  status.heading_error = err.valid ? err.heading : 0.0;
  status.lookahead_distance = ld_eff;
  status.path_curvature = kappa_ahead;
  status.commanded_speed = speed;
  status.commanded_steering = invert_steering_ ? -steering : steering;
  status.disengaged = disengaged;
  status.reason = disengaged ? disengage_reason_ : reason;
  status_pub_->publish(status);

  // Save state for next cycle.
  prev_speed_cmd_ = speed;
  prev_steering_cmd_ = steering;
  last_control_time_ = t_now;
  have_last_control_ = true;
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
