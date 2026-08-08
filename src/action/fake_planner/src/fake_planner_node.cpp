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

#include "fake_planner/fake_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace fake_planner
{

FakePlannerNode::FakePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fake_planner_node", options)
, release_time_(now())
{
  // Seeded from now() above rather than left default-constructed: a default rclcpp::Time carries
  // the system clock, and subtracting it from a sim-time now() throws.

  // Defaults only: config/params.yaml is where these are tuned and documented.
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("behaviour_topic", "execute_behaviour");
  declare_parameter("odom_topic", "odom");
  declare_parameter("frame_id", "odom");
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("publish_behaviour", true);
  declare_parameter("behaviour", "lane_follow");

  // "auto" | "relative" | "absolute", resolved against the loaded maneuver in on_configure. Not
  // "on"/"off": YAML reads those as booleans, and a bool override on a string parameter aborts
  // the node at construction.
  declare_parameter("anchoring", "auto");

  declare_parameter("horizon_m", 35.0);
  declare_parameter("trail_m", 2.0);
  declare_parameter("search_ahead_m", 400.0);
  declare_parameter("respawn_jump_m", 5.0);
  declare_parameter("start_on_activate", true);
  declare_parameter("finish_distance_m", 3.0);
  declare_parameter("finish_speed_mps", 0.25);
  declare_parameter("max_path_deviation_m", 5.0);
  declare_parameter("release_ramp_s", 3.0);
  declare_parameter("maneuver_file", std::string(""));

  // Same param name and default topic as the real planner.
  declare_parameter("marker_pub_topic", "trajectory_markers");
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  behaviour_topic_ = get_parameter("behaviour_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  publish_behaviour_ = get_parameter("publish_behaviour").as_bool();
  behaviour_ = get_parameter("behaviour").as_string();
  anchoring_ = get_parameter("anchoring").as_string();
  start_on_activate_ = get_parameter("start_on_activate").as_bool();
  maneuver_file_ = get_parameter("maneuver_file").as_string();
  marker_topic_ = get_parameter("marker_pub_topic").as_string();
  horizon_m_ = get_parameter("horizon_m").as_double();
  trail_m_ = get_parameter("trail_m").as_double();
  search_ahead_m_ = get_parameter("search_ahead_m").as_double();
  respawn_jump_m_ = get_parameter("respawn_jump_m").as_double();
  finish_distance_m_ = get_parameter("finish_distance_m").as_double();
  finish_speed_mps_ = get_parameter("finish_speed_mps").as_double();
  max_path_deviation_m_ = get_parameter("max_path_deviation_m").as_double();
  release_ramp_s_ = get_parameter("release_ramp_s").as_double();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "publish_rate_hz must be > 0 (got %.3f)", publish_rate_hz_);
    return CallbackReturn::FAILURE;
  }
  // The window geometry feeds a bounded walk over the path, and the thresholds are compared
  // against distances and speeds, so a negative value is never meaningful -- fail rather than
  // publish a one-point window or a maneuver that can never finish.
  if (horizon_m_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "horizon_m must be > 0 (got %.3f)", horizon_m_);
    return CallbackReturn::FAILURE;
  }
  if (search_ahead_m_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "search_ahead_m must be > 0 (got %.3f)", search_ahead_m_);
    return CallbackReturn::FAILURE;
  }
  const std::pair<const char *, double> non_negative[] = {
    {"trail_m", trail_m_},
    {"respawn_jump_m", respawn_jump_m_},
    {"finish_distance_m", finish_distance_m_},
    {"finish_speed_mps", finish_speed_mps_},
    {"max_path_deviation_m", max_path_deviation_m_},
    {"release_ramp_s", release_ramp_s_},
  };
  for (const auto & [name, value] : non_negative) {
    if (value < 0.0) {
      RCLCPP_ERROR(get_logger(), "%s must be >= 0 (got %.3f)", name, value);
      return CallbackReturn::FAILURE;
    }
  }
  if (maneuver_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "maneuver_file is empty; point it at a maneuver JSON (segment list).");
    return CallbackReturn::FAILURE;
  }
  if (anchoring_ != "auto" && anchoring_ != "relative" && anchoring_ != "absolute") {
    RCLCPP_ERROR(get_logger(), "anchoring must be one of auto|relative|absolute (got '%s').", anchoring_.c_str());
    return CallbackReturn::FAILURE;
  }

  FakePlannerConfig config;
  config.horizon_m = horizon_m_;
  config.trail_m = trail_m_;
  config.search_ahead_m = search_ahead_m_;
  config.frame_id = frame_id_;
  core_ = std::make_unique<FakePlannerCore>(config);

  std::string load_error;
  if (!core_->loadManeuver(maneuver_file_, load_error)) {
    RCLCPP_ERROR(get_logger(), "Failed to load maneuver '%s': %s", maneuver_file_.c_str(), load_error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (core_->waypointCount() < 2) {
    RCLCPP_ERROR(
      get_logger(), "Maneuver '%s' has %zu waypoint(s); need >= 2.", maneuver_file_.c_str(), core_->waypointCount());
    return CallbackReturn::FAILURE;
  }

  // The finish check compares distance-to-stop-line against finish_distance_m, so a maneuver that
  // short is inside the finish band before it starts. moved_ keeps it from latching immediately,
  // but the run still ends the moment the car slows down anywhere on it.
  if (!core_->closed() && core_->pathLength() <= finish_distance_m_) {
    RCLCPP_WARN(
      get_logger(),
      "Maneuver '%s' is only %.2f m long, within finish_distance_m (%.2f): it will count as "
      "finished as soon as the vehicle slows. Shorten finish_distance_m or lengthen the maneuver.",
      maneuver_file_.c_str(),
      core_->pathLength(),
      finish_distance_m_);
  }

  anchor_to_first_pose_ = (anchoring_ == "auto") ? !core_->hasAbsoluteStart() : (anchoring_ == "relative");

  // An absolute maneuver keeps its own geometry across a reset, so reset only re-acquires the
  // nearest point -- which, for a car parked on the stop line, is the stop line. Closed maneuvers
  // have no stop line and relative ones are re-laid from the current pose, so both are fine.
  if (!anchor_to_first_pose_ && !core_->closed()) {
    RCLCPP_WARN(
      get_logger(),
      "Maneuver '%s' declares an absolute 'start' and is not closed: the reset service cannot "
      "re-run it in place, because the path stays where it was drawn and the vehicle is left "
      "parked at its end. Move the vehicle back to the start, or relaunch, to run it again.",
      maneuver_file_.c_str());
  }

  // Forcing "relative" on a maneuver that declares an absolute start is legal (replay the oval's
  // shape from wherever the car is parked) but is never what you want on the track it was drawn
  // for, so warn rather than silently putting the path through a hedge.
  if (anchoring_ == "relative" && core_->hasAbsoluteStart()) {
    RCLCPP_WARN(
      get_logger(),
      "anchoring:=relative with a maneuver that declares an absolute 'start': the path will be laid "
      "from the vehicle's pose rather than on its authored geometry. Use anchoring:=auto to publish "
      "it where it was drawn.");
  }

  trajectory_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>(trajectory_topic_, rclcpp::QoS(10));
  if (publish_behaviour_) {
    behaviour_pub_ = create_publisher<behaviour_msgs::msg::ExecuteBehaviour>(behaviour_topic_, rclcpp::QoS(10));
  }
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(10));

  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_trajectory",
    std::bind(&FakePlannerNode::startTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop_trajectory",
    std::bind(&FakePlannerNode::stopTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&FakePlannerNode::resetTrajectory, this, std::placeholders::_1, std::placeholders::_2));

  have_pose_ = false;
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10), std::bind(&FakePlannerNode::odomCallback, this, std::placeholders::_1));

  // Anchoring waits for the first odom to lay the waypoints out from the vehicle's pose, so the
  // same file works regardless of where odom's origin is. Without it they publish verbatim.
  if (anchor_to_first_pose_) {
    anchored_ = false;
  } else {
    core_->anchor(0.0, 0.0, 0.0);
    anchored_ = true;
  }

  RCLCPP_INFO(
    get_logger(),
    "Configured: %zu waypoints from '%s', frame '%s', %.1f Hz, anchor=%s, behaviour=%s, start_on_activate=%s",
    core_->waypointCount(),
    maneuver_file_.c_str(),
    frame_id_.c_str(),
    publish_rate_hz_,
    anchor_to_first_pose_ ? (anchoring_ == "auto" ? "true (auto: relative maneuver)" : "true")
                          : (anchoring_ == "auto" ? "false (auto: absolute 'start')" : "false"),
    publish_behaviour_ ? behaviour_.c_str() : "(disabled)",
    start_on_activate_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

void FakePlannerNode::releaseTrajectory()
{
  if (!trajectory_started_) {
    release_time_ = now();
  }
  trajectory_started_ = true;
}

double FakePlannerNode::releaseRampScale()
{
  if (release_ramp_s_ <= 0.0) {
    return 1.0;
  }
  const double elapsed = (now() - release_time_).seconds();
  // A negative elapsed means the clock went backwards under us -- sim time restarting on a world
  // reload. Re-seed rather than hand back a nonsense scale.
  if (elapsed < 0.0) {
    release_time_ = now();
    return 0.0;
  }
  return std::min(1.0, elapsed / release_ramp_s_);
}

void FakePlannerNode::startTrajectory(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // A finished run does not restart here: the car is parked on the stop line, so republishing
  // would either do nothing (all-zero speeds) or drive off the end. Re-running is reset's job.
  if (finished_) {
    response->success = false;
    response->message = "Maneuver already finished; call reset to run it again";
    RCLCPP_WARN(get_logger(), "Start ignored: %s", response->message.c_str());
    return;
  }

  releaseTrajectory();
  response->success = true;
  response->message = "Trajectory started";
  RCLCPP_INFO(get_logger(), "Trajectory started via service");
}

void FakePlannerNode::stopTrajectory(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  trajectory_started_ = false;
  response->success = true;
  response->message = "Trajectory stopped; controller will fall back to standby";
  RCLCPP_INFO(get_logger(), "Trajectory stopped via service");
}

void FakePlannerNode::resetTrajectory(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // When anchoring, drop the anchored trajectory so the next odom re-lays the maneuver from
  // wherever the car ended up. Absolute maneuvers keep their geometry and re-acquire the nearest
  // point.
  {
    const std::lock_guard<std::mutex> lock(core_mutex_);
    if (anchor_to_first_pose_) {
      anchored_ = false;
      core_->clear();
    } else {
      core_->rewind();
    }
  }

  // Match a fresh activate: clear the latches, then resume or hold per start_on_activate.
  finished_ = false;
  moved_ = false;
  trajectory_started_ = false;
  if (start_on_activate_) {
    releaseTrajectory();
  }

  response->success = true;
  response->message = anchor_to_first_pose_ ? "Reset; re-anchoring on next odom" : "Reset to nearest point";
  RCLCPP_INFO(get_logger(), "Reset via service (%s)", response->message.c_str());
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_pub_->on_activate();
  if (behaviour_pub_) {
    behaviour_pub_->on_activate();
  }
  marker_pub_->on_activate();

  trajectory_started_ = false;
  finished_ = false;
  moved_ = false;
  if (start_on_activate_) {
    releaseTrajectory();
  }

  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, std::bind(&FakePlannerNode::timerCallback, this));

  if (trajectory_started_) {
    RCLCPP_INFO(get_logger(), "Activated: publishing trajectory at %.1f Hz", publish_rate_hz_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Activated: holding in standby. Call the start service to drive:\n"
      "  ros2 service call %s/start_trajectory std_srvs/srv/Trigger",
      get_fully_qualified_name());
  }
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_->on_deactivate();
  if (behaviour_pub_) {
    behaviour_pub_->on_deactivate();
  }
  marker_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_.reset();
  behaviour_pub_.reset();
  marker_pub_.reset();
  odom_sub_.reset();
  start_srv_.reset();
  stop_srv_.reset();
  reset_srv_.reset();
  {
    const std::lock_guard<std::mutex> lock(core_mutex_);
    core_.reset();
  }
  anchored_ = false;
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_.reset();
  behaviour_pub_.reset();
  marker_pub_.reset();
  odom_sub_.reset();
  start_srv_.reset();
  stop_srv_.reset();
  reset_srv_.reset();
  {
    const std::lock_guard<std::mutex> lock(core_mutex_);
    core_.reset();
  }
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

// Same marker scheme as trajectory_planner's visualization -- wipe, then a sphere per point sized
// by speed with a text label on every 4th point -- so the two look identical in Foxglove. Kept as
// its own copy rather than factored into a shared helper, deliberately: sharing it would mean
// editing a production planner to serve a test rig.
//
// Differs from trajectory_planner in what saturates the sphere size (the real planner normalizes by
// the lane speed limit, which we don't have, so the maneuver's top speed stands in) and in taking
// the trajectory by argument, so the labels show the speeds actually published rather than the
// unscaled ones sitting in the core.
void FakePlannerNode::publishMarkers(const wato_trajectory_msgs::msg::Trajectory & trajectory)
{
  if (!marker_pub_ || !marker_pub_->is_activated()) {
    return;
  }
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  double limit_speed = 0.0;
  for (const auto & point : trajectory.points) {
    limit_speed = std::max(limit_speed, static_cast<double>(point.max_speed));
  }
  if (limit_speed <= 0.0) {
    limit_speed = 1.0;
  }

  auto header = trajectory.header;
  header.stamp = now();

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "trajectory_velocity";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = point.pose;

    // Size correlates with speed: base 0.1 m, up to 0.5 m at the top speed.
    const double speed_ratio = std::max(0.0, std::min(1.0, point.max_speed / limit_speed));
    const double diameter = 0.1 + (0.4 * speed_ratio);

    // Label every 4th point to avoid clutter. Keyed off the point index, not a shared running id:
    // incrementing one counter for both namespaces makes the labels land irregularly, since a
    // labelled point advances it twice.
    if (i % 4 == 0) {
      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "trajectory_speed_labels";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = point.pose;
      label.pose.position.y += 0.8;
      label.pose.position.z += 0.5;
      label.scale.z = 0.4;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;

      std::ostringstream speed_text;
      speed_text << std::fixed << std::setprecision(1) << point.max_speed << " m/s";
      label.text = speed_text.str();
      markers.markers.push_back(label);
    }

    sphere.scale.x = diameter;
    sphere.scale.y = diameter;
    sphere.scale.z = diameter;

    sphere.color.r = 0.6f;
    sphere.color.g = 0.2f;
    sphere.color.b = 0.8f;
    sphere.color.a = 0.8f;

    markers.markers.push_back(sphere);
  }

  marker_pub_->publish(markers);
}

void FakePlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const double new_x = msg->pose.pose.position.x;
  const double new_y = msg->pose.pose.position.y;

  veh_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  if (veh_speed_ > finish_speed_mps_) {
    moved_ = true;
  }

  // A teleport between consecutive odom messages means the vehicle was respawned underneath us.
  // Re-anchor instead of steering toward a path laid out from a pose that no longer exists.
  const bool teleported =
    have_pose_ && anchored_ && respawn_jump_m_ > 0.0 && std::hypot(new_x - veh_x_, new_y - veh_y_) > respawn_jump_m_;

  veh_x_ = new_x;
  veh_y_ = new_y;
  have_pose_ = true;

  const std::lock_guard<std::mutex> lock(core_mutex_);

  if (teleported) {
    RCLCPP_INFO(get_logger(), "Pose jumped to (%.2f, %.2f) -- treating as a respawn and re-anchoring", new_x, new_y);
    core_->rewind();
    // Treated exactly like a fresh activate: the respawn itself never releases the car.
    finished_ = false;
    moved_ = false;
    trajectory_started_ = false;
    if (start_on_activate_) {
      releaseTrajectory();
    }
    if (anchor_to_first_pose_) {
      anchored_ = false;  // falls through to the anchoring path below
    }
  }

  if (anchored_) {
    core_->updateWindow(veh_x_, veh_y_);
    if (core_->distanceToPath() > max_path_deviation_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Vehicle is %.2f m from the maneuver (max_path_deviation_m %.2f): the published window is "
        "the nearest stretch of path, which may be behind or beside the car.",
        core_->distanceToPath(),
        max_path_deviation_m_);
    }
    return;
  }

  const auto & q = msg->pose.pose.orientation;
  const double anchor_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  core_->anchor(new_x, new_y, anchor_yaw);
  core_->updateWindow(veh_x_, veh_y_);
  anchored_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Anchored trajectory to pose (%.2f, %.2f, yaw=%.3f) in frame '%s'",
    new_x,
    new_y,
    anchor_yaw,
    frame_id_.c_str());
}

void FakePlannerNode::timerCallback()
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  if (!core_) {
    return;
  }

  // Publishing nothing lets the controller time out into standby so the car holds where it stopped.
  // Markers stay up so the driven path is still visible.
  if (finished_) {
    publishMarkers(core_->window());
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000, "Maneuver finished; in standby. Call the reset service to run it again.");
    return;
  }

  // Markers only until started, so the maneuver can be inspected before releasing the car: with no
  // trajectory and no behaviour heartbeat the controller stays in standby.
  if (!trajectory_started_) {
    publishMarkers(core_->window());
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Holding: waiting for the start_trajectory service.");
    return;
  }

  if (!core_->ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for first odom on '%s' to anchor trajectory...", odom_topic_.c_str());
    return;
  }

  // End of an open maneuver. Latch rather than sitting on a zero-speed window forever: an engaged
  // controller parked on the end of the path is one tracking wobble away from creeping. On a closed
  // circuit distanceToEnd is infinite, so laps are untouched. moved_ gates it because a car that
  // has not been anywhere yet also satisfies "close to the stop line and stationary".
  if (moved_ && have_pose_ && core_->distanceToEnd() <= finish_distance_m_ && veh_speed_ <= finish_speed_mps_) {
    finished_ = true;
    trajectory_started_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Reached the end of the maneuver (%.2f m short of the stop line at %.2f m/s) -- back to standby.\n"
      "  To run it again: ros2 service call %s/reset std_srvs/srv/Trigger",
      core_->distanceToEnd(),
      veh_speed_,
      get_fully_qualified_name());
    publishMarkers(core_->window());
    return;
  }

  auto trajectory = core_->window();
  trajectory.header.stamp = now();

  // Ease the car off the line: a closed maneuver skips the core's geometric launch ramp (its window
  // wraps at the seam, so a ramp there would brake the car once a lap), which would otherwise leave
  // it commanding cruise speed from a standstill the moment it is released. Scaling the published
  // window instead is time-based and position-independent, so it works for closed and open alike --
  // on an open maneuver it simply composes with the geometric ramp, whichever is lower winning.
  const double ramp_scale = releaseRampScale();
  if (ramp_scale < 1.0) {
    for (auto & point : trajectory.points) {
      point.max_speed = static_cast<float>(point.max_speed * ramp_scale);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "Release ramp: publishing %.0f%% of authored speed", ramp_scale * 100.0);
  }

  trajectory_pub_->publish(trajectory);
  publishMarkers(trajectory);

  if (behaviour_pub_) {
    behaviour_msgs::msg::ExecuteBehaviour beh;
    beh.behaviour = behaviour_;
    behaviour_pub_->publish(beh);
  }
}

}  // namespace fake_planner

RCLCPP_COMPONENTS_REGISTER_NODE(fake_planner::FakePlannerNode)
