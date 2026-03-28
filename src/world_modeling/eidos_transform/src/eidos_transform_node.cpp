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

#include "eidos_transform/eidos_transform_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace eidos_transform
{

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

EidosTransformNode::EidosTransformNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("eidos_transform_node", options)
, ekf_loader_("eidos_transform", "eidos_transform::EKFModelPlugin")
{
  RCLCPP_INFO(get_logger(), "EidosTransformNode created.");
}

EidosTransformNode::~EidosTransformNode() = default;

// ---------------------------------------------------------------------------
// Helper: parse a list of MeasurementSource entries from parameters
// ---------------------------------------------------------------------------

static void readNoiseParam(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & param,
  gtsam::Vector6 & out, const std::vector<double> & defaults)
{
  node->declare_parameter<std::vector<double>>(param, defaults);
  std::vector<double> v;
  node->get_parameter(param, v);
  for (size_t i = 0; i < 6 && i < v.size(); ++i) out(static_cast<int>(i)) = v[i];
}

static void parseSources(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & list_param,
  std::vector<MeasurementSource> & out,
  std::mutex & mtx,
  const std::string & label)
{
  node->declare_parameter<std::vector<std::string>>(list_param, std::vector<std::string>{});
  std::vector<std::string> names;
  node->get_parameter(list_param, names);

  for (const auto & src_name : names) {
    MeasurementSource src;
    src.name = src_name;

    node->declare_parameter<std::string>(src_name + ".type", "odom");
    node->declare_parameter<std::string>(src_name + ".topic", "");
    node->get_parameter(src_name + ".type", src.type);
    node->get_parameter(src_name + ".topic", src.topic);

    if (src.topic.empty()) {
      RCLCPP_WARN(node->get_logger(), "%s source '%s' has no topic, skipping.", label.c_str(), src_name.c_str());
      continue;
    }

    size_t idx = out.size();

    if (src.type == "imu") {
      // IMU-specific params
      node->declare_parameter<std::string>(src_name + ".imu_frame", "imu_link");
      node->declare_parameter<bool>(src_name + ".use_orientation", true);
      node->declare_parameter<bool>(src_name + ".use_angular_velocity", true);
      node->declare_parameter<bool>(src_name + ".use_linear_acceleration", false);
      node->declare_parameter<double>(src_name + ".gravity", 9.80511);
      node->declare_parameter<bool>(src_name + ".gravity_compensated", false);

      node->get_parameter(src_name + ".imu_frame", src.imu_frame);
      node->get_parameter(src_name + ".use_orientation", src.use_orientation);
      node->get_parameter(src_name + ".use_angular_velocity", src.use_angular_velocity);
      node->get_parameter(src_name + ".use_linear_acceleration", src.use_linear_acceleration);
      node->get_parameter(src_name + ".gravity", src.gravity);
      node->get_parameter(src_name + ".gravity_compensated", src.gravity_compensated);

      readNoiseParam(node, src_name + ".orientation_noise", src.orientation_noise,
        {1e-2, 1e-2, 1e-2, 1.0, 1.0, 1.0});
      readNoiseParam(node, src_name + ".angular_velocity_noise", src.angular_velocity_noise,
        {1e-4, 1e-4, 1e-4, 1.0, 1.0, 1.0});
      readNoiseParam(node, src_name + ".linear_velocity_noise", src.linear_velocity_noise,
        {1.0, 1.0, 1.0, 1e-1, 1e-1, 1e-1});

      src.imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        src.topic, rclcpp::SensorDataQoS(),
        [&out, &mtx, idx](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx);
          if (idx < out.size()) {
            out[idx].latest_imu = msg;
            out[idx].has_new_data = true;
          }
        });

      RCLCPP_INFO(node->get_logger(), "%s source '%s' [imu] on '%s' frame '%s'",
        label.c_str(), src_name.c_str(), src.topic.c_str(), src.imu_frame.c_str());

    } else {
      // Odom-type params
      node->declare_parameter<std::vector<bool>>(
        src_name + ".pose_mask", {false, false, false, false, false, false});
      node->declare_parameter<std::vector<bool>>(
        src_name + ".twist_mask", {false, false, false, false, false, false});

      std::vector<bool> pm, tm;
      node->get_parameter(src_name + ".pose_mask", pm);
      node->get_parameter(src_name + ".twist_mask", tm);
      for (size_t i = 0; i < 6 && i < pm.size(); ++i) src.pose_mask[i] = pm[i];
      for (size_t i = 0; i < 6 && i < tm.size(); ++i) src.twist_mask[i] = tm[i];

      readNoiseParam(node, src_name + ".pose_noise", src.pose_noise,
        {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
      readNoiseParam(node, src_name + ".twist_noise", src.twist_noise,
        {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});

      src.odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        src.topic, rclcpp::SensorDataQoS(),
        [&out, &mtx, idx](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx);
          if (idx < out.size()) {
            out[idx].latest_odom = msg;
            out[idx].has_new_data = true;
          }
        });

      RCLCPP_INFO(node->get_logger(), "%s source '%s' [odom] on '%s'",
        label.c_str(), src_name.c_str(), src.topic.c_str());
    }

    out.push_back(std::move(src));
  }
}

// ---------------------------------------------------------------------------
// Lifecycle: on_configure
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // Parameters
  declare_parameter<double>("tick_rate", 200.0);
  declare_parameter<std::string>("frames.odom", "odom");
  declare_parameter<std::string>("frames.base_link", "base_footprint");
  declare_parameter<std::string>("frames.map", "map");
  declare_parameter<std::string>("frames.utm", "utm");
  declare_parameter<std::string>("topics.odom", "transform/odometry");
  declare_parameter<std::string>("topics.utm_to_map", "");
  declare_parameter<std::string>("topics.predict_service", "transform/predict_relative");
  declare_parameter<std::string>("ekf.plugin", "eidos_transform::HolonomicEKF");
  declare_parameter<std::string>("ekf.name", "holonomic_ekf");

  get_parameter("tick_rate", tick_rate_);
  get_parameter("frames.odom", odom_frame_);
  get_parameter("frames.base_link", base_link_frame_);
  get_parameter("frames.map", map_frame_);
  get_parameter("frames.utm", utm_frame_);
  get_parameter("topics.odom", odom_topic_);
  get_parameter("topics.utm_to_map", utm_to_map_topic_);

  std::string predict_service_topic;
  get_parameter("topics.predict_service", predict_service_topic);
  get_parameter("ekf.plugin", ekf_plugin_name_);

  std::string ekf_name;
  get_parameter("ekf.name", ekf_name);

  // Load dual EKF plugins (same type, separate instances)
  try {
    local_ekf_ = ekf_loader_.createSharedInstance(ekf_plugin_name_);
    local_ekf_->initialize(ekf_name, shared_from_this());

    global_ekf_ = ekf_loader_.createSharedInstance(ekf_plugin_name_);
    global_ekf_->initialize(ekf_name + "_global", shared_from_this());

    RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m Loaded dual EKF: %s (local + global)", ekf_plugin_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load EKF plugin '%s': %s", ekf_plugin_name_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  // Parse odom_sources (fed to BOTH EKFs)
  parseSources(this, "odom_sources", odom_sources_, sources_mutex_, "Odom");

  // Parse map_sources (fed to global EKF ONLY)
  parseSources(this, "map_sources", map_sources_, sources_mutex_, "Map");


  // TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Odom publisher
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  // UTM to map subscriber
  if (!utm_to_map_topic_.empty()) {
    utm_to_map_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      utm_to_map_topic_,
      rclcpp::QoS(1).transient_local(),
      std::bind(&EidosTransformNode::utmToMapCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "UTM-to-map on topic '%s'", utm_to_map_topic_.c_str());
  }

  // PredictRelativeTransform service
  predict_srv_ = create_service<eidos_msgs::srv::PredictRelativeTransform>(
    predict_service_topic,
    std::bind(
      &EidosTransformNode::predictRelativeTransformCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Tick callback group
  tick_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(get_logger(), "Configuration complete.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate / on_deactivate / on_cleanup / on_shutdown
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  odom_pub_->on_activate();
  local_ekf_->activate();
  global_ekf_->activate();

  first_tick_ = true;

  auto period = std::chrono::duration<double>(1.0 / tick_rate_);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EidosTransformNode::tick, this),
    tick_callback_group_);

  RCLCPP_INFO(get_logger(), "Active. Tick rate: %.1f Hz", tick_rate_);
  return CallbackReturn::SUCCESS;
}

EidosTransformNode::CallbackReturn EidosTransformNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  if (tick_timer_) { tick_timer_->cancel(); tick_timer_.reset(); }
  local_ekf_->deactivate();
  global_ekf_->deactivate();
  odom_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

EidosTransformNode::CallbackReturn EidosTransformNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  tick_timer_.reset();
  local_ekf_.reset();
  global_ekf_.reset();
  odom_sources_.clear();
  map_sources_.clear();
  odom_pub_.reset();
  utm_to_map_sub_.reset();
  predict_srv_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
  tf_listener_.reset();
  return CallbackReturn::SUCCESS;
}

EidosTransformNode::CallbackReturn EidosTransformNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Tick loop
// ---------------------------------------------------------------------------

void EidosTransformNode::tick()
{
  rclcpp::Time now = this->now();
  if (now.nanoseconds() == 0) return;

  double dt = 0.0;
  if (first_tick_) {
    first_tick_ = false;
    last_tick_time_ = now;
    global_ekf_time_ = now.seconds();
    RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First tick, broadcasting identity TF");
    broadcastOdomToBaseTF(now);
    return;
  }
  dt = (now - last_tick_time_).seconds();
  if (dt <= 0.0) return;
  if (dt > 1.0) {
    RCLCPP_WARN(get_logger(), "\033[34m[Transform]\033[0m Large dt=%.4f, skipping", dt);
    last_tick_time_ = now;
    return;
  }
  last_tick_time_ = now;
  double now_sec = now.seconds();

  // ---- Local EKF: immediate fusion (no delay handling needed) ----
  local_ekf_->predict(dt);

  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (auto & src : odom_sources_) {
      if (!src.has_new_data) continue;
      src.has_new_data = false;
      fuseSource(local_ekf_, src);
      if (!src.logged_first_msg) {
        RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First odom source '%s' on '%s'",
          src.name.c_str(), src.topic.c_str());
        src.logged_first_msg = true;
      }
    }
  }

  // ---- Global EKF: delayed measurement handling (rewind-replay) ----
  global_ekf_->predict(dt);
  global_ekf_time_ = now_sec;

  // Save state snapshot after predict (before any measurements this tick)
  global_state_history_.push_back(global_ekf_->snapshot(now_sec));
  while (global_state_history_.size() > kMaxHistory) global_state_history_.pop_front();

  // Collect new map source measurements with their timestamps
  std::vector<MeasurementRecord> new_measurements;
  bool needs_rewind = false;
  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (auto & src : map_sources_) {
      if (!src.has_new_data) continue;
      src.has_new_data = false;
      has_map_source_data_ = true;

      MeasurementRecord rec;
      rec.source_name = src.name;
      rec.source_type = src.type;
      rec.target = MeasurementRecord::Target::GLOBAL;

      if (src.type == "odom" && src.latest_odom) {
        rec.time = rclcpp::Time(src.latest_odom->header.stamp).seconds();
        rec.pose = odomMsgToPose3(*src.latest_odom);
        rec.twist = odomMsgToTwist(*src.latest_odom);
        rec.pose_mask = src.pose_mask;
        rec.twist_mask = src.twist_mask;
        rec.pose_noise = src.pose_noise;
        rec.twist_noise = src.twist_noise;
      } else if (src.type == "imu" && src.latest_imu) {
        rec.time = rclcpp::Time(src.latest_imu->header.stamp).seconds();
      } else {
        continue;
      }

      // Check if this measurement is delayed (older than the previous tick time)
      if (rec.time < global_ekf_time_ - dt - 0.01) {
        needs_rewind = true;
      }

      new_measurements.push_back(rec);

      if (!src.logged_first_msg) {
        RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First map source '%s' on '%s'",
          src.name.c_str(), src.topic.c_str());
        src.logged_first_msg = true;
      }
    }
  }

  // Add new measurements to history
  for (auto & rec : new_measurements) {
    global_measurement_history_.push_back(rec);
  }
  while (global_measurement_history_.size() > kMaxHistory) {
    global_measurement_history_.pop_front();
  }

  if (needs_rewind) {
    // Find the oldest delayed measurement
    double oldest_delayed = now_sec;
    for (const auto & rec : new_measurements) {
      if (rec.time < oldest_delayed) oldest_delayed = rec.time;
    }
    rewindAndReplay(oldest_delayed);
  } else {
    // No delayed measurements — fuse new ones immediately at current time
    for (auto & rec : new_measurements) {
      applyMeasurement(global_ekf_, rec);
    }
  }

  // Broadcast TF and publish odom
  broadcastOdomToBaseTF(now);
  broadcastMapToOdomTF(now);
  publishOdometry(now);

}

// ---------------------------------------------------------------------------
// Rewind-Replay for delayed measurements
// ---------------------------------------------------------------------------

void EidosTransformNode::rewindAndReplay(double delayed_time)
{
  // Find the latest state snapshot before the delayed measurement
  StateSnapshot restore_point;
  bool found = false;
  for (auto it = global_state_history_.rbegin(); it != global_state_history_.rend(); ++it) {
    if (it->time <= delayed_time) {
      restore_point = *it;
      found = true;
      break;
    }
  }

  if (!found) {
    // No snapshot old enough — can't rewind, just apply at current time
    return;
  }

  // Restore EKF to that snapshot
  global_ekf_->restore(restore_point);

  // Collect all measurements from restore_point time to now, sorted by time
  std::vector<MeasurementRecord> replay;
  for (const auto & rec : global_measurement_history_) {
    if (rec.time > restore_point.time) {
      replay.push_back(rec);
    }
  }
  std::sort(replay.begin(), replay.end(), [](const auto & a, const auto & b) { return a.time < b.time; });

  // Replay: predict between measurements, apply each one
  double last_time = restore_point.time;
  for (const auto & rec : replay) {
    double replay_dt = rec.time - last_time;
    if (replay_dt > 0.0 && replay_dt < 1.0) {
      global_ekf_->predict(replay_dt);
    }
    applyMeasurement(global_ekf_, rec);
    last_time = rec.time;
  }

  // Predict to current time
  double final_dt = global_ekf_time_ - last_time;
  if (final_dt > 0.0 && final_dt < 1.0) {
    global_ekf_->predict(final_dt);
  }

  // Clear old state history and re-snapshot
  global_state_history_.clear();
  global_state_history_.push_back(global_ekf_->snapshot(global_ekf_time_));
}

// ---------------------------------------------------------------------------
// Apply a measurement record to an EKF
// ---------------------------------------------------------------------------

void EidosTransformNode::applyMeasurement(
  std::shared_ptr<EKFModelPlugin> & ekf, const MeasurementRecord & rec)
{
  if (rec.source_type == "odom") {
    bool any_pose = false;
    for (int i = 0; i < 6; ++i) {
      if (rec.pose_mask[static_cast<size_t>(i)]) { any_pose = true; break; }
    }
    if (any_pose) {
      ekf->updatePose(rec.pose, rec.pose_mask, rec.pose_noise);
    }

    bool any_twist = false;
    for (int i = 0; i < 6; ++i) {
      if (rec.twist_mask[static_cast<size_t>(i)]) { any_twist = true; break; }
    }
    if (any_twist) {
      ekf->updateTwist(rec.twist, rec.twist_mask, rec.twist_noise);
    }
  }
  // IMU measurement replay would go here if map_sources ever include IMU type
}

// ---------------------------------------------------------------------------
// Fuse a measurement source into an EKF
// ---------------------------------------------------------------------------

void EidosTransformNode::fuseSource(
  std::shared_ptr<EKFModelPlugin> & ekf,
  MeasurementSource & src)
{
  if (src.type == "imu") {
    // IMU source: extract gyro, orientation, acceleration
    if (!src.latest_imu) return;
    const auto & msg = *src.latest_imu;

    // Resolve imu_frame → base_link rotation (once)
    if (!src.has_imu_tf) {
      try {
        auto tf = tf_buffer_->lookupTransform(base_link_frame_, src.imu_frame, tf2::TimePointZero);
        const auto & r = tf.transform.rotation;
        src.R_base_imu = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
        src.has_imu_tf = true;
      } catch (const tf2::TransformException &) {
        return;
      }
    }

    Eigen::Vector3d gyr_imu(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    Eigen::Vector3d acc_imu(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Eigen::Vector3d gyr_base = src.R_base_imu * gyr_imu;
    Eigen::Vector3d acc_base = src.R_base_imu * acc_imu;

    // Angular velocity → twist update
    if (src.use_angular_velocity) {
      gtsam::Vector6 twist = gtsam::Vector6::Zero();
      twist(0) = gyr_base.x();
      twist(1) = gyr_base.y();
      twist(2) = gyr_base.z();
      std::array<bool, 6> mask = {true, true, true, false, false, false};
      ekf->updateTwist(twist, mask, src.angular_velocity_noise);
    }

    // Orientation → pose update (rotation only)
    if (src.use_orientation && msg.orientation_covariance[0] >= 0.0) {
      Eigen::Quaterniond q_imu(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
      if (q_imu.squaredNorm() > 0.5) {
        q_imu.normalize();
        Eigen::Matrix3d R_world_base = q_imu.toRotationMatrix() * src.R_base_imu.transpose();
        // Use EKF's current translation so Logmap only sees rotation difference
        gtsam::Pose3 orientation_pose(gtsam::Rot3(R_world_base), ekf->pose().translation());
        std::array<bool, 6> mask = {true, true, true, false, false, false};
        ekf->updatePose(orientation_pose, mask, src.orientation_noise);
      }
    }

    // Linear acceleration → gravity compensation, then EKF handles bias + integration
    if (src.use_linear_acceleration) {
      double imu_time = rclcpp::Time(msg.header.stamp).seconds();
      Eigen::Vector3d acc_compensated = acc_base;

      // Remove gravity if not already compensated by the sensor
      if (!src.gravity_compensated) {
        Eigen::Matrix3d R_world_body = ekf->pose().rotation().matrix();
        Eigen::Vector3d g_world(0.0, 0.0, -src.gravity);
        Eigen::Vector3d g_body = R_world_body.transpose() * g_world;
        acc_compensated = acc_base - g_body;
      }

      // Feed directly to EKF — it handles bias subtraction and velocity update internally
      if (src.last_imu_time > 0.0) {
        double imu_dt = imu_time - src.last_imu_time;
        if (imu_dt > 0.0 && imu_dt < 0.1) {
          Eigen::Vector3d accel_noise(
            src.linear_velocity_noise(3), src.linear_velocity_noise(4), src.linear_velocity_noise(5));
          ekf->updateAcceleration(acc_compensated, accel_noise, imu_dt);
        }
      }
      src.last_imu_time = imu_time;
    }
    return;
  }

  // Odom source: extract pose and twist
  if (!src.latest_odom) return;
  gtsam::Pose3 meas_pose = odomMsgToPose3(*src.latest_odom);
  gtsam::Vector6 meas_twist = odomMsgToTwist(*src.latest_odom);

  bool any_pose = false;
  for (int i = 0; i < 6; ++i) {
    if (src.pose_mask[static_cast<size_t>(i)]) { any_pose = true; break; }
  }
  if (any_pose) {
    ekf->updatePose(meas_pose, src.pose_mask, src.pose_noise);
  }

  bool any_twist = false;
  for (int i = 0; i < 6; ++i) {
    if (src.twist_mask[static_cast<size_t>(i)]) { any_twist = true; break; }
  }
  if (any_twist) {
    ekf->updateTwist(meas_twist, src.twist_mask, src.twist_noise);
  }
}

// ---------------------------------------------------------------------------
// TF broadcasting
// ---------------------------------------------------------------------------

void EidosTransformNode::broadcastOdomToBaseTF(const rclcpp::Time & stamp)
{
  gtsam::Pose3 p = local_ekf_->pose();
  gtsam::Quaternion q = p.rotation().toQuaternion();
  gtsam::Point3 t = p.translation();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_link_frame_;
  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();
  tf_msg.transform.rotation.w = q.w();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();

  tf_broadcaster_->sendTransform(tf_msg);
}

void EidosTransformNode::broadcastMapToOdomTF(const rclcpp::Time & stamp)
{
  if (!has_map_source_data_) return;

  // Use local EKF pose directly — both EKFs were just updated in the same tick,
  // so this is exactly the odom→base_link we just broadcast. No TF lookup needed.
  gtsam::Pose3 odom_to_base = local_ekf_->pose();

  // map_to_odom = map_to_base (global EKF) * inv(odom_to_base)
  gtsam::Pose3 map_to_odom = global_ekf_->pose().compose(odom_to_base.inverse());

  gtsam::Quaternion q = map_to_odom.rotation().toQuaternion();
  gtsam::Point3 t = map_to_odom.translation();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;
  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();
  tf_msg.transform.rotation.w = q.w();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();

  tf_broadcaster_->sendTransform(tf_msg);
}

// ---------------------------------------------------------------------------
// Publish fused odometry (from local EKF)
// ---------------------------------------------------------------------------

void EidosTransformNode::publishOdometry(const rclcpp::Time & stamp)
{
  if (!odom_pub_->is_activated()) return;

  gtsam::Pose3 p = local_ekf_->pose();
  gtsam::Vector6 v = local_ekf_->velocity();
  gtsam::Quaternion q = p.rotation().toQuaternion();
  gtsam::Point3 t = p.translation();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;

  odom_msg.pose.pose.position.x = t.x();
  odom_msg.pose.pose.position.y = t.y();
  odom_msg.pose.pose.position.z = t.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();

  odom_msg.twist.twist.angular.x = v(0);
  odom_msg.twist.twist.angular.y = v(1);
  odom_msg.twist.twist.angular.z = v(2);
  odom_msg.twist.twist.linear.x = v(3);
  odom_msg.twist.twist.linear.y = v(4);
  odom_msg.twist.twist.linear.z = v(5);

  odom_pub_->publish(odom_msg);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void EidosTransformNode::utmToMapCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  cached_utm_to_map_ = *msg;
  cached_utm_to_map_.header.frame_id = utm_frame_;
  cached_utm_to_map_.child_frame_id = map_frame_;
  has_utm_to_map_ = true;

  tf_broadcaster_->sendTransform(cached_utm_to_map_);
  RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m Published static utm->map transform.");
}

void EidosTransformNode::predictRelativeTransformCallback(
  const std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Request> request,
  std::shared_ptr<eidos_msgs::srv::PredictRelativeTransform::Response> response)
{
  double dt = request->timestamp_to - request->timestamp_from;
  if (dt < 0.0) {
    RCLCPP_WARN(get_logger(), "PredictRelativeTransform: negative dt (%.4f), rejecting.", dt);
    response->success = false;
    return;
  }

  gtsam::Vector6 vel;
  {
    std::lock_guard lock(sources_mutex_);
    vel = local_ekf_->velocity();
  }
  gtsam::Vector6 delta = vel * dt;
  gtsam::Pose3 relative = gtsam::Pose3::Expmap(delta);

  gtsam::Quaternion q = relative.rotation().toQuaternion();
  gtsam::Point3 t = relative.translation();

  response->relative_pose.position.x = t.x();
  response->relative_pose.position.y = t.y();
  response->relative_pose.position.z = t.z();
  response->relative_pose.orientation.w = q.w();
  response->relative_pose.orientation.x = q.x();
  response->relative_pose.orientation.y = q.y();
  response->relative_pose.orientation.z = q.z();
  response->success = true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

gtsam::Pose3 EidosTransformNode::odomMsgToPose3(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
}

gtsam::Vector6 EidosTransformNode::odomMsgToTwist(const nav_msgs::msg::Odometry & msg)
{
  const auto & a = msg.twist.twist.angular;
  const auto & l = msg.twist.twist.linear;
  gtsam::Vector6 twist;
  twist << a.x, a.y, a.z, l.x, l.y, l.z;
  return twist;
}

}  // namespace eidos_transform
