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
#include <exception>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

// A moving frame: world position (x, y) and heading theta. Segments are authored in this local
// frame (u forward along theta, v to the left) and transformed into the world as we walk them,
// so each segment simply continues from where the previous one ended.
struct Cursor
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// A finely-sampled polyline (parallel x/y/speed) built up before uniform resampling.
struct Path
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> speed;
};

double lerp(double a, double b, double t)
{
  return a + (b - a) * t;
}

// Append the world point for a local (u forward, v left) offset from the cursor.
void appendLocal(Path & path, const Cursor & c, double u, double v, double speed)
{
  const double cos_t = std::cos(c.theta);
  const double sin_t = std::sin(c.theta);
  path.x.push_back(c.x + u * cos_t - v * sin_t);
  path.y.push_back(c.y + u * sin_t + v * cos_t);
  path.speed.push_back(speed);
}

// Number of fine sub-steps for a segment of the given local length.
int subSteps(double length, double fine)
{
  return std::max(1, static_cast<int>(std::ceil(std::abs(length) / fine)));
}

// --- Primitive builders. Each appends samples for the segment interior (t in (0, L]) and moves
// the cursor to the segment end; the caller seeds the very first point. Speed ramps s0 -> s1. ---

// Straight / dwell: hold the current lateral offset, advance along the heading.
void buildStraight(Path & path, Cursor & c, double length, double s0, double s1, double fine)
{
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    appendLocal(path, c, t, 0.0, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Shift: smooth (raised-cosine) lateral move of `lateral`, tangent to the heading at both ends.
void buildShift(Path & path, Cursor & c, double lateral, double length, double s0, double s1, double fine)
{
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    const double v = lateral * (1.0 - std::cos(kPi * t / length)) / 2.0;
    appendLocal(path, c, t, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Slalom: sinusoidal weave. The amplitude is tapered over the first/last half wavelength so the
// path starts and ends on the centreline tangent to the heading, while the middle stays a pure
// sine (the steady-state region the frequency-response test cares about).
void buildSlalom(
  Path & path, Cursor & c, double amp, double wavelength, double cycles, double s0, double s1, double fine)
{
  const double length = wavelength * cycles;
  const double taper = std::min(wavelength / 2.0, length / 2.0);
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    double env = amp;
    if (t < taper) {
      env = amp * (1.0 - std::cos(kPi * t / taper)) / 2.0;
    } else if (t > length - taper) {
      env = amp * (1.0 - std::cos(kPi * (length - t) / taper)) / 2.0;
    }
    const double v = env * std::sin(2.0 * kPi * t / wavelength);
    appendLocal(path, c, t, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Arc: constant-radius turn (dir_sign +1 = left/CCW, -1 = right/CW); also rotates the heading.
void buildArc(
  Path & path, Cursor & c, double radius, double angle_rad, double dir_sign, double s0, double s1, double fine)
{
  const double length = radius * angle_rad;  // arc length
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    const double phi = t / radius;  // angle swept so far
    const double u = radius * std::sin(phi);
    const double v = dir_sign * radius * (1.0 - std::cos(phi));
    appendLocal(path, c, u, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
  c.theta += dir_sign * angle_rad;
}

// Resample a fine polyline to uniform arc-length spacing (speed linearly interpolated). This is
// what makes the output indistinguishable from a real planner, which emits uniformly-spaced
// points (trajectory_planner's interpolation_resolution).
Path resampleUniform(const Path & in, double spacing)
{
  if (in.x.size() < 2) {
    return in;
  }
  std::vector<double> cum(in.x.size(), 0.0);
  for (std::size_t i = 1; i < in.x.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(in.x[i] - in.x[i - 1], in.y[i] - in.y[i - 1]);
  }
  const double total = cum.back();

  Path out;
  std::size_t j = 0;
  for (double s = 0.0; s < total; s += spacing) {
    while (j + 1 < cum.size() && cum[j + 1] < s) {
      ++j;
    }
    const double seg = cum[j + 1] - cum[j];
    const double f = seg > 1e-9 ? (s - cum[j]) / seg : 0.0;
    out.x.push_back(lerp(in.x[j], in.x[j + 1], f));
    out.y.push_back(lerp(in.y[j], in.y[j + 1], f));
    out.speed.push_back(lerp(in.speed[j], in.speed[j + 1], f));
  }
  // Always finish exactly on the last authored point (unless we already landed there).
  if (out.x.empty() || std::hypot(in.x.back() - out.x.back(), in.y.back() - out.y.back()) > 1e-6) {
    out.x.push_back(in.x.back());
    out.y.push_back(in.y.back());
    out.speed.push_back(in.speed.back());
  }
  return out;
}

// Deceleration used to ramp an open maneuver down to a standstill. Matches
// trajectory_planner's max_tangential_accel so the fake stop feels like the real one.
constexpr double kMaxDecel = 1.0;

// How far past the last authored point the zero-speed pad extends. Pure pursuit only steers to
// points *ahead* of the vehicle and simply returns (holding its last command) when it runs out,
// so the path has to stay populated past the stop line or the car drives off the end. Comfortably
// longer than the controller's max lookahead.
constexpr double kStopPadM = 10.0;

// Terminate an open maneuver in a standstill: extend the final heading with zero-speed points,
// then propagate that zero backwards at kMaxDecel so the speed profile brakes into it. Same
// backward pass trajectory_planner runs, just with a known terminal speed instead of an obstacle.
void appendStopPad(Path & path, double spacing)
{
  if (path.x.size() < 2) {
    return;
  }
  const double dx = path.x.back() - path.x[path.x.size() - 2];
  const double dy = path.y.back() - path.y[path.y.size() - 2];
  const double norm = std::hypot(dx, dy);
  if (norm < 1e-9) {
    return;
  }

  const double end_x = path.x.back();
  const double end_y = path.y.back();
  const int pad_points = static_cast<int>(std::ceil(kStopPadM / spacing));
  for (int k = 1; k <= pad_points; ++k) {
    const double t = spacing * k;
    path.x.push_back(end_x + dx / norm * t);
    path.y.push_back(end_y + dy / norm * t);
    path.speed.push_back(0.0);
  }
  // The last authored point is the stop line itself.
  path.speed.back() = 0.0;
  path.speed[path.speed.size() - pad_points - 1] = 0.0;

  for (int i = static_cast<int>(path.speed.size()) - 2; i >= 0; --i) {
    const double seg = std::hypot(path.x[i + 1] - path.x[i], path.y[i + 1] - path.y[i]);
    const double v_max = std::sqrt(path.speed[i + 1] * path.speed[i + 1] + 2.0 * kMaxDecel * seg);
    path.speed[i] = std::min(path.speed[i], v_max);
  }
}

}  // namespace

namespace fake_planner
{

FakePlannerNode::FakePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fake_planner_node", options)
{
  // Topics / frame
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("behaviour_topic", "execute_behaviour");
  declare_parameter("odom_topic", "odom");
  declare_parameter("frame_id", "odom");

  // Behaviour
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("publish_behaviour", true);
  declare_parameter("behaviour", "lane_follow");
  declare_parameter("anchor_to_first_pose", true);

  // Rolling window. The real planner never publishes the whole route: the lattice planner emits
  // a ~30 m horizon starting 2 m behind the vehicle, replanned as it drives. Publishing the full
  // maneuver instead makes both the markers and the controller's search look nothing like the
  // real thing, so the fake planner slides the same window along its maneuver.
  declare_parameter("horizon_m", 35.0);
  declare_parameter("trail_m", 2.0);

  // Respawn detection. A pose that jumps this far between two odom messages is a teleport, not
  // driving -- in sim that means the ego was destroyed and respawned. Continuing to publish a
  // path anchored to the old pose after that is never right, so the node re-anchors itself and
  // the demo resets with a single "respawn ego" click rather than a click-in-the-right-order
  // dance. Set 0 to disable (e.g. if a localization jump on the real car would trip it).
  declare_parameter("respawn_jump_m", 5.0);

  // Whether activation starts the trajectory immediately. Set false to bring the stack up
  // with the vehicle held in standby and release it later with the start_trajectory service.
  declare_parameter("start_on_activate", true);

  // Maneuver JSON (a segment list, expanded + resampled into waypoints at load; authored in
  // frame_id relative to the anchor pose, or from an absolute "start" pose when the file
  // supplies one). One maneuver per file, selected from the package's maneuvers/ folder via
  // the launch.
  declare_parameter("maneuver_file", std::string(""));

  // Trajectory visualization, mirroring trajectory_planner: a MarkerArray of speed-sized
  // spheres + speed labels, published with the trajectory whenever someone subscribes.
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
  anchor_to_first_pose_ = get_parameter("anchor_to_first_pose").as_bool();
  start_on_activate_ = get_parameter("start_on_activate").as_bool();
  maneuver_file_ = get_parameter("maneuver_file").as_string();
  marker_topic_ = get_parameter("marker_pub_topic").as_string();
  horizon_m_ = get_parameter("horizon_m").as_double();
  trail_m_ = get_parameter("trail_m").as_double();
  respawn_jump_m_ = get_parameter("respawn_jump_m").as_double();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "publish_rate_hz must be > 0 (got %.3f)", publish_rate_hz_);
    return CallbackReturn::FAILURE;
  }
  if (maneuver_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "maneuver_file is empty; point it at a maneuver JSON (segment list).");
    return CallbackReturn::FAILURE;
  }
  std::string load_error;
  if (!loadManeuver(maneuver_file_, load_error)) {
    RCLCPP_ERROR(get_logger(), "Failed to load maneuver '%s': %s", maneuver_file_.c_str(), load_error.c_str());
    return CallbackReturn::FAILURE;
  }
  // loadManeuver fills x/y/speed together, so the arrays are equal-length by construction.
  if (wp_x_.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Maneuver '%s' has %zu waypoint(s); need >= 2.", maneuver_file_.c_str(), wp_x_.size());
    return CallbackReturn::FAILURE;
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

  // Odom is needed either way: to anchor the layout (optionally) and to slide the rolling window
  // as the vehicle drives.
  have_pose_ = false;
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10), std::bind(&FakePlannerNode::odomCallback, this, std::placeholders::_1));

  // Anchoring lays the waypoints out from the vehicle's pose at launch, so the same file
  // works regardless of where odom's origin is (sim spawn vs. accumulated odom on the car).
  // Without it, waypoints are published verbatim in frame_id.
  if (anchor_to_first_pose_) {
    anchored_ = false;
    trajectory_ready_ = false;
  } else {
    anchor_x_ = 0.0;
    anchor_y_ = 0.0;
    anchor_yaw_ = 0.0;
    anchored_ = true;
    buildTrajectory();
  }

  RCLCPP_INFO(
    get_logger(),
    "Configured: %zu waypoints from '%s', frame '%s', %.1f Hz, anchor=%s, behaviour=%s, start_on_activate=%s",
    wp_x_.size(),
    maneuver_file_.c_str(),
    frame_id_.c_str(),
    publish_rate_hz_,
    anchor_to_first_pose_ ? "true" : "false",
    publish_behaviour_ ? behaviour_.c_str() : "(disabled)",
    start_on_activate_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

void FakePlannerNode::startTrajectory(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  trajectory_started_ = true;
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
  // Rewind the window. When anchoring, drop the anchor too so the next odom re-lays the whole
  // maneuver from wherever the car ended up -- that is what makes a re-run possible after the
  // car has driven to the end and stopped. Absolute maneuvers keep their fixed geometry and
  // just re-acquire the nearest point.
  window_start_ = 0;
  if (anchor_to_first_pose_) {
    anchored_ = false;
    trajectory_ready_ = false;
  }

  // Match a fresh activate: resume immediately or hold, per start_on_activate.
  trajectory_started_ = start_on_activate_;

  response->success = true;
  response->message = anchor_to_first_pose_ ? "Reset; re-anchoring on next odom" : "Reset to nearest point";
  RCLCPP_INFO(get_logger(), "Reset via service (%s)", response->message.c_str());
}

bool FakePlannerNode::loadManeuver(const std::string & path, std::string & error)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    error = "cannot open file";
    return false;
  }

  nlohmann::json doc;
  try {
    file >> doc;
  } catch (const std::exception & e) {
    error = std::string("invalid JSON: ") + e.what();
    return false;
  }
  if (!doc.is_object()) {
    error = "top-level JSON must be an object";
    return false;
  }

  // Spacing is a property of the MAP, not of the planner. For lane following nothing in the
  // real pipeline resamples: lattice_planning walks the lanelet centreline out to the horizon
  // and hands those points over verbatim (centreline_to_path_points), and trajectory_planner
  // emits one TrajectoryPoint per input pose. So the trajectory's waypoint spacing is just the
  // lanelet centreline node spacing of whatever map is loaded. Measured from maps/osm:
  //   oval_track_4_lane.osm  2.00 m   (generated at NODE_SPACING by tools/generate_oval_track.py)
  //   Town10HD.osm           1.00 m
  // path_gen.path_steps does NOT set this -- that only discretises the cubic spirals used for
  // lane CHANGES, which lane-following never goes through.
  const double spacing = doc.value("sample_spacing_m", 1.0);
  const double default_speed = doc.value("default_speed", 2.0);
  closed_ = doc.value("closed", false);
  if (spacing <= 0.0) {
    error = "sample_spacing_m must be > 0";
    return false;
  }
  if (!doc.contains("segments") || !doc["segments"].is_array() || doc["segments"].empty()) {
    error = "missing or empty 'segments' array";
    return false;
  }

  // Walk the segments as a moving frame, sampling each finely, then resample to uniform spacing.
  const double fine = std::min(0.05, spacing);
  Path fine_path;
  Cursor cursor;

  // Optional absolute start pose: a maneuver on fixed geometry (e.g. an oval lane) sets
  // "start" and runs with anchor_to_first_pose=false, publishing verbatim in frame_id.
  if (doc.contains("start")) {
    const nlohmann::json & start = doc["start"];
    if (!start.is_object()) {
      error = "'start' must be an object with x/y/yaw";
      return false;
    }
    cursor.x = start.value("x", 0.0);
    cursor.y = start.value("y", 0.0);
    cursor.theta = start.value("yaw", 0.0);
  }

  double current_speed = default_speed;  // carried between segments for speed continuity

  std::size_t idx = 0;
  for (const auto & seg : doc["segments"]) {
    ++idx;
    if (!seg.is_object() || seg.size() != 1) {
      error = "segment " + std::to_string(idx) + ": expected a single {type: {...}} object";
      return false;
    }
    const auto it = seg.begin();
    const std::string type = it.key();
    const nlohmann::json & p = it.value();
    if (!p.is_object()) {
      error = "segment " + std::to_string(idx) + " (" + type + "): parameters must be an object";
      return false;
    }

    const double s0 = p.value("speed", current_speed);
    const double s1 = p.value("end_speed", s0);

    // Seed the very first path point at the start pose and speed.
    if (fine_path.x.empty()) {
      appendLocal(fine_path, cursor, 0.0, 0.0, s0);
    }

    try {
      if (type == "straight" || type == "dwell") {
        buildStraight(fine_path, cursor, p.at("length").get<double>(), s0, s1, fine);
      } else if (type == "shift") {
        buildShift(fine_path, cursor, p.at("lateral").get<double>(), p.at("length").get<double>(), s0, s1, fine);
      } else if (type == "slalom") {
        buildSlalom(
          fine_path,
          cursor,
          p.at("amplitude").get<double>(),
          p.at("wavelength").get<double>(),
          p.at("cycles").get<double>(),
          s0,
          s1,
          fine);
      } else if (type == "arc") {
        const std::string dir = p.value("dir", std::string("left"));
        if (dir != "left" && dir != "right") {
          error = "segment " + std::to_string(idx) + " (arc): dir must be 'left' or 'right'";
          return false;
        }
        const double dir_sign = (dir == "left") ? 1.0 : -1.0;
        buildArc(
          fine_path,
          cursor,
          p.at("radius").get<double>(),
          p.at("angle").get<double>() * kPi / 180.0,
          dir_sign,
          s0,
          s1,
          fine);
      } else {
        error = "segment " + std::to_string(idx) + ": unknown type '" + type + "'";
        return false;
      }
    } catch (const std::exception & e) {
      error = "segment " + std::to_string(idx) + " (" + type + "): " + e.what();
      return false;
    }

    current_speed = s1;
  }

  Path resampled = resampleUniform(fine_path, spacing);
  // A closed circuit laps forever, so it has no end to stop at; everything else does.
  if (!closed_) {
    appendStopPad(resampled, spacing);
  }
  wp_x_ = resampled.x;
  wp_y_ = resampled.y;
  wp_speed_ = resampled.speed;
  return true;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_pub_->on_activate();
  if (behaviour_pub_) {
    behaviour_pub_->on_activate();
  }
  marker_pub_->on_activate();

  trajectory_started_ = start_on_activate_;

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
  trajectory_ready_ = false;
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
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void FakePlannerNode::buildTrajectory()
{
  const double c = std::cos(anchor_yaw_);
  const double s = std::sin(anchor_yaw_);

  // Apply the SE(2) anchor transform to every waypoint up front so the timer callback only
  // has to restamp and publish.
  std::vector<double> xs(wp_x_.size());
  std::vector<double> ys(wp_x_.size());
  for (std::size_t i = 0; i < wp_x_.size(); ++i) {
    xs[i] = anchor_x_ + wp_x_[i] * c - wp_y_[i] * s;
    ys[i] = anchor_y_ + wp_x_[i] * s + wp_y_[i] * c;
  }

  wato_trajectory_msgs::msg::Trajectory traj;
  traj.header.frame_id = frame_id_;
  traj.points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = xs[i];
    pt.pose.position.y = ys[i];
    pt.pose.position.z = 0.0;

    // Yaw from consecutive points (last point reuses the previous heading).
    double yaw = 0.0;
    if (i + 1 < xs.size()) {
      yaw = std::atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i]);
    } else if (i > 0) {
      yaw = std::atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]);
    }
    pt.pose.orientation.z = std::sin(yaw / 2.0);
    pt.pose.orientation.w = std::cos(yaw / 2.0);

    pt.max_speed = wp_speed_[i];
    traj.points.push_back(pt);
  }

  full_traj_ = traj;
  window_start_ = 0;
  trajectory_ready_ = true;
  updateWindow();
}

std::size_t FakePlannerNode::nearestIndex() const
{
  const std::size_t n = full_traj_.points.size();
  // Search a bounded stretch ahead of the last match rather than the whole path: a slalom or a
  // closed lap passes near its own earlier points, and a global search would jump back to them.
  constexpr std::size_t kSearchAhead = 400;
  const std::size_t span = std::min(kSearchAhead, n);

  std::size_t best = window_start_;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t k = 0; k < span; ++k) {
    const std::size_t i = closed_ ? (window_start_ + k) % n : std::min(window_start_ + k, n - 1);
    const auto & p = full_traj_.points[i].pose.position;
    const double d = std::hypot(p.x - veh_x_, p.y - veh_y_);
    if (d < best_dist) {
      best_dist = d;
      best = i;
    }
  }
  return best;
}

// Slice the published horizon out of the full maneuver: trail_m_ behind the vehicle (so pure
// pursuit always has path starting behind it and doesn't creep) through horizon_m_ ahead. On a
// closed circuit the window wraps and the car laps; on an open one it runs into the zero-speed
// stop pad and holds there.
void FakePlannerNode::updateWindow()
{
  const std::size_t n = full_traj_.points.size();
  if (n == 0) {
    return;
  }
  if (!have_pose_) {
    trajectory_ = full_traj_;
    return;
  }

  const std::size_t nearest = nearestIndex();
  window_start_ = nearest;

  wato_trajectory_msgs::msg::Trajectory window;
  window.header = full_traj_.header;

  // Walk backwards from the nearest point to lay down the trailing stub.
  std::size_t first = nearest;
  double trailed = 0.0;
  while (trailed < trail_m_) {
    const std::size_t prev = (first == 0) ? (closed_ ? n - 1 : 0) : first - 1;
    if (prev == first) {
      break;  // open path, already at the start
    }
    const auto & a = full_traj_.points[prev].pose.position;
    const auto & b = full_traj_.points[first].pose.position;
    trailed += std::hypot(b.x - a.x, b.y - a.y);
    first = prev;
  }

  double covered = 0.0;
  std::size_t i = first;
  for (std::size_t k = 0; k < n; ++k) {
    window.points.push_back(full_traj_.points[i]);
    const std::size_t next = (i + 1 == n) ? (closed_ ? 0 : i) : i + 1;
    if (next == i) {
      break;  // open path, ran off the end
    }
    const auto & a = full_traj_.points[i].pose.position;
    const auto & b = full_traj_.points[next].pose.position;
    covered += std::hypot(b.x - a.x, b.y - a.y);
    i = next;
    if (covered >= trailed + horizon_m_) {
      break;
    }
  }

  trajectory_ = window;
}

// Same marker scheme as trajectory_planner's visualization: wipe, then a sphere per point
// sized by speed with a text label on every 4th marker. The only difference is the speed
// that saturates the sphere size: the real planner normalizes by the lane speed limit,
// which the fake planner doesn't have, so the maneuver's top speed stands in for it.
void FakePlannerNode::publishMarkers()
{
  if (!marker_pub_ || !marker_pub_->is_activated() || !trajectory_ready_) {
    return;
  }
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  double limit_speed = 0.0;
  for (const auto & point : trajectory_.points) {
    limit_speed = std::max(limit_speed, static_cast<double>(point.max_speed));
  }
  if (limit_speed <= 0.0) {
    limit_speed = 1.0;
  }

  auto header = trajectory_.header;
  header.stamp = now();

  visualization_msgs::msg::MarkerArray markers;

  // Delete all previous markers first
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  // Create a sphere marker for each point
  int id = 0;
  for (const auto & point : trajectory_.points) {
    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "trajectory_velocity";
    sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = point.pose;

    // Size correlates with speed: base 0.1 m, up to 0.5 m at the top speed.
    const double speed_ratio = std::max(0.0, std::min(1.0, point.max_speed / limit_speed));
    const double diameter = 0.1 + (0.4 * speed_ratio);

    // Only add labels for every 4th point to avoid clutter
    if (id % 4 == 0) {
      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "trajectory_speed_labels";
      label.id = id++;
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

      const std::string speed_str = std::to_string(point.max_speed);
      label.text = speed_str.substr(0, speed_str.find(".") + 2) + " m/s";
      markers.markers.push_back(label);
    }

    sphere.scale.x = diameter;
    sphere.scale.y = diameter;
    sphere.scale.z = diameter;

    // Uniform color (Purple)
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

  // A teleport between consecutive odom messages means the vehicle was respawned underneath us.
  // Re-anchor instead of steering toward a path laid out from a pose that no longer exists.
  const bool teleported =
    have_pose_ && anchored_ && respawn_jump_m_ > 0.0 && std::hypot(new_x - veh_x_, new_y - veh_y_) > respawn_jump_m_;

  veh_x_ = new_x;
  veh_y_ = new_y;
  have_pose_ = true;

  if (teleported) {
    RCLCPP_INFO(get_logger(), "Pose jumped to (%.2f, %.2f) -- treating as a respawn and re-anchoring", new_x, new_y);
    window_start_ = 0;
    trajectory_started_ = start_on_activate_;
    if (anchor_to_first_pose_) {
      anchored_ = false;  // falls through to the anchoring path below
    }
  }

  if (anchored_) {
    updateWindow();
    return;
  }
  anchor_x_ = msg->pose.pose.position.x;
  anchor_y_ = msg->pose.pose.position.y;
  const auto & q = msg->pose.pose.orientation;
  // Yaw from quaternion (z-up): atan2(2(wz + xy), 1 - 2(yy + zz)).
  anchor_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  anchored_ = true;
  buildTrajectory();

  RCLCPP_INFO(
    get_logger(),
    "Anchored trajectory to pose (%.2f, %.2f, yaw=%.3f) in frame '%s'",
    anchor_x_,
    anchor_y_,
    anchor_yaw_,
    frame_id_.c_str());
}

void FakePlannerNode::timerCallback()
{
  // Publish nothing until started (except the marker preview, so the maneuver can be
  // inspected before releasing the car): with no trajectory and no behaviour heartbeat
  // the controller stays in standby rather than driving off on bringup.
  if (!trajectory_started_) {
    publishMarkers();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Holding: waiting for the start_trajectory service.");
    return;
  }

  if (!trajectory_ready_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for first odom on '%s' to anchor trajectory...", odom_topic_.c_str());
    return;
  }

  trajectory_.header.stamp = now();
  trajectory_pub_->publish(trajectory_);
  publishMarkers();

  if (behaviour_pub_) {
    behaviour_msgs::msg::ExecuteBehaviour beh;
    beh.behaviour = behaviour_;
    behaviour_pub_->publish(beh);
  }
}

}  // namespace fake_planner

RCLCPP_COMPONENTS_REGISTER_NODE(fake_planner::FakePlannerNode)
