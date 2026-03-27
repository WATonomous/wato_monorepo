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
// Lifecycle: on_configure
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // ---- Parameters ----
  declare_parameter<double>("tick_rate", 200.0);
  declare_parameter<std::string>("frames.odom", "odom");
  declare_parameter<std::string>("frames.base_link", "base_footprint");
  declare_parameter<std::string>("frames.map", "map");
  declare_parameter<std::string>("frames.utm", "utm");
  declare_parameter<std::string>("topics.odom", "transform/odometry");
  declare_parameter<std::string>("topics.map_source", "slam/pose");
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
  get_parameter("topics.map_source", map_source_topic_);
  get_parameter("topics.utm_to_map", utm_to_map_topic_);

  std::string predict_service_topic;
  get_parameter("topics.predict_service", predict_service_topic);
  get_parameter("ekf.plugin", ekf_plugin_name_);

  std::string ekf_name;
  get_parameter("ekf.name", ekf_name);

  // ---- Load EKF model plugin ----
  try {
    ekf_model_ = ekf_loader_.createSharedInstance(ekf_plugin_name_);
    ekf_model_->initialize(ekf_name, shared_from_this());
    RCLCPP_INFO(get_logger(), "Loaded EKF plugin: %s", ekf_plugin_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load EKF plugin '%s': %s", ekf_plugin_name_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  // ---- Parse odom_sources ----
  declare_parameter<std::vector<std::string>>("odom_sources", std::vector<std::string>{});
  std::vector<std::string> source_names;
  get_parameter("odom_sources", source_names);

  for (const auto & src_name : source_names) {
    MeasurementSource src;
    src.name = src_name;

    declare_parameter<std::string>(src_name + ".odom_topic", "");
    get_parameter(src_name + ".odom_topic", src.odom_topic);

    if (src.odom_topic.empty()) {
      RCLCPP_WARN(get_logger(), "Odom source '%s' has no topic, skipping.", src_name.c_str());
      continue;
    }

    // Pose mask: 6 bools [rx, ry, rz, tx, ty, tz]
    declare_parameter<std::vector<bool>>(src_name + ".pose_mask", {false, false, false, false, false, false});
    std::vector<bool> pm;
    get_parameter(src_name + ".pose_mask", pm);
    for (size_t i = 0; i < 6 && i < pm.size(); ++i) {
      src.pose_mask[i] = pm[i];
    }

    // Twist mask
    declare_parameter<std::vector<bool>>(src_name + ".twist_mask", {false, false, false, false, false, false});
    std::vector<bool> tm;
    get_parameter(src_name + ".twist_mask", tm);
    for (size_t i = 0; i < 6 && i < tm.size(); ++i) {
      src.twist_mask[i] = tm[i];
    }

    // Pose noise (std devs)
    declare_parameter<std::vector<double>>(src_name + ".pose_noise", {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
    std::vector<double> pn;
    get_parameter(src_name + ".pose_noise", pn);
    for (size_t i = 0; i < 6 && i < pn.size(); ++i) {
      src.pose_noise(static_cast<int>(i)) = pn[i];
    }

    // Twist noise (std devs)
    declare_parameter<std::vector<double>>(src_name + ".twist_noise", {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
    std::vector<double> tn;
    get_parameter(src_name + ".twist_noise", tn);
    for (size_t i = 0; i < 6 && i < tn.size(); ++i) {
      src.twist_noise(static_cast<int>(i)) = tn[i];
    }

    // Create subscriber
    src.subscriber = create_subscription<nav_msgs::msg::Odometry>(
      src.odom_topic,
      rclcpp::SensorDataQoS(),
      [this, idx = sources_.size()](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(sources_mutex_);
        if (idx < sources_.size()) {
          sources_[idx].latest_msg = msg;
          sources_[idx].has_new_data = true;
        }
      });

    RCLCPP_INFO(get_logger(), "Odom source '%s' on topic '%s'", src_name.c_str(), src.odom_topic.c_str());

    sources_.push_back(std::move(src));
  }

  // ---- TF broadcasters ----
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  // ---- Odom publisher ----
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  // ---- Map source subscriber ----
  if (!map_source_topic_.empty()) {
    map_source_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      map_source_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&EidosTransformNode::mapSourceCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Map source on topic '%s'", map_source_topic_.c_str());
  }

  // ---- UTM to map subscriber ----
  if (!utm_to_map_topic_.empty()) {
    utm_to_map_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      utm_to_map_topic_,
      rclcpp::QoS(1).transient_local(),
      std::bind(&EidosTransformNode::utmToMapCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "UTM-to-map on topic '%s'", utm_to_map_topic_.c_str());
  }

  // ---- PredictRelativeTransform service ----
  predict_srv_ = create_service<eidos_msgs::srv::PredictRelativeTransform>(
    predict_service_topic,
    std::bind(
      &EidosTransformNode::predictRelativeTransformCallback, this, std::placeholders::_1, std::placeholders::_2));

  // ---- Tick callback group + timer (created but not started until activate) ----
  tick_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(get_logger(), "Configuration complete.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  odom_pub_->on_activate();
  ekf_model_->activate();

  first_tick_ = true;

  auto period = std::chrono::duration<double>(1.0 / tick_rate_);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EidosTransformNode::tick, this),
    tick_callback_group_);

  RCLCPP_INFO(get_logger(), "Active. Tick rate: %.1f Hz", tick_rate_);
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_deactivate
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  if (tick_timer_) {
    tick_timer_->cancel();
    tick_timer_.reset();
  }

  ekf_model_->deactivate();
  odom_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_cleanup
// ---------------------------------------------------------------------------

EidosTransformNode::CallbackReturn EidosTransformNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");

  tick_timer_.reset();
  ekf_model_.reset();
  sources_.clear();
  odom_pub_.reset();
  map_source_sub_.reset();
  utm_to_map_sub_.reset();
  predict_srv_.reset();
  tf_broadcaster_.reset();
  static_tf_broadcaster_.reset();

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_shutdown
// ---------------------------------------------------------------------------

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

  // Skip if clock hasn't started yet (sim_time = 0)
  if (now.nanoseconds() == 0) {
    return;
  }

  // Compute dt from sim time. Skip if time hasn't advanced (bag clock slower than tick rate).
  double dt = 0.0;
  if (first_tick_) {
    first_tick_ = false;
    last_tick_time_ = now;
    RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First tick, broadcasting identity TF");
    broadcastOdomToBaseTF(now);
    broadcastMapToOdomTF(now);
    return;
  }
  dt = (now - last_tick_time_).seconds();

  if (dt <= 0.0) {
    return;  // Sim time hasn't advanced, nothing to do
  }
  if (dt > 1.0) {
    RCLCPP_WARN(get_logger(), "\033[34m[Transform]\033[0m Large dt=%.4f, clamping to skip", dt);
    last_tick_time_ = now;
    return;
  }
  last_tick_time_ = now;

  // Predict
  ekf_model_->predict(dt);

  // Fuse each measurement source
  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (auto & src : sources_) {
      if (!src.has_new_data || !src.latest_msg) {
        continue;
      }
      src.has_new_data = false;

      gtsam::Pose3 meas_pose = odomMsgToPose3(*src.latest_msg);
      gtsam::Vector6 meas_twist = odomMsgToTwist(*src.latest_msg);

      bool any_pose = false;
      for (int i = 0; i < 6; ++i) {
        if (src.pose_mask[static_cast<size_t>(i)]) {
          any_pose = true;
          break;
        }
      }
      if (any_pose) {
        ekf_model_->updatePose(meas_pose, src.pose_mask, src.pose_noise);
      }

      bool any_twist = false;
      for (int i = 0; i < 6; ++i) {
        if (src.twist_mask[static_cast<size_t>(i)]) {
          any_twist = true;
          break;
        }
      }
      if (any_twist) {
        ekf_model_->updateTwist(meas_twist, src.twist_mask, src.twist_noise);
      }

      auto mt = meas_pose.translation();
      RCLCPP_INFO(
        get_logger(),
        "\033[34m[Transform]\033[0m Source '%s': input pos=(%.2f,%.2f,%.2f)",
        src.name.c_str(), mt.x(), mt.y(), mt.z());

      if (!src.logged_first_msg) {
        RCLCPP_INFO(
          get_logger(),
          "\033[34m[Transform]\033[0m First message from source '%s' on topic '%s'",
          src.name.c_str(), src.odom_topic.c_str());
        src.logged_first_msg = true;
      }
    }
  }

  // Store timestamped EKF pose for precise map->odom lookup
  {
    std::lock_guard lock(map_to_odom_mtx_);
    ekf_pose_history_.push_back({now.seconds(), ekf_model_->pose()});
    while (ekf_pose_history_.size() > kMaxPoseHistory) {
      ekf_pose_history_.pop_front();
    }
  }

  // Broadcast TF and publish odom
  broadcastOdomToBaseTF(now);
  broadcastMapToOdomTF(now);
  publishOdometry(now);

  // Cross-reference log: EKF state, odom->base, map->odom
  gtsam::Pose3 ekf_pose = ekf_model_->pose();
  auto et = ekf_pose.translation();

  if (has_map_to_odom_.load(std::memory_order_acquire)) {
    gtsam::Pose3 m2o;
    {
      std::lock_guard lock(map_to_odom_mtx_);
      m2o = cached_map_to_odom_;
    }
    auto m2ot = m2o.translation();
    auto m2or = m2o.rotation().yaw();
    auto er = ekf_pose.rotation().yaw();
    // map->base = map->odom * odom->base
    gtsam::Pose3 map_to_base = m2o.compose(ekf_pose);
    auto mbt = map_to_base.translation();
    auto mbr = map_to_base.rotation().yaw();
    RCLCPP_INFO(
      get_logger(),
      "\033[34m[Transform]\033[0m dt=%.4f odom->base=(%.2f,%.2f,%.2f yaw=%.2f°) "
      "map->odom=(%.2f,%.2f,%.2f yaw=%.2f°) map->base=(%.2f,%.2f,%.2f yaw=%.2f°)",
      dt, et.x(), et.y(), et.z(), er * 180.0 / M_PI,
      m2ot.x(), m2ot.y(), m2ot.z(), m2or * 180.0 / M_PI,
      mbt.x(), mbt.y(), mbt.z(), mbr * 180.0 / M_PI);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "\033[34m[Transform]\033[0m dt=%.4f odom->base=(%.2f,%.2f,%.2f) map->odom=NOT SET",
      dt, et.x(), et.y(), et.z());
  }
}

// ---------------------------------------------------------------------------
// TF broadcasting
// ---------------------------------------------------------------------------

void EidosTransformNode::broadcastOdomToBaseTF(const rclcpp::Time & stamp)
{
  gtsam::Pose3 p = ekf_model_->pose();
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
  if (!has_map_to_odom_.load(std::memory_order_acquire)) {
    return;
  }

  gtsam::Pose3 map_to_odom;
  {
    std::lock_guard lock(map_to_odom_mtx_);
    map_to_odom = cached_map_to_odom_;
  }
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
// Publish fused odometry
// ---------------------------------------------------------------------------

void EidosTransformNode::publishOdometry(const rclcpp::Time & stamp)
{
  if (!odom_pub_->is_activated()) {
    return;
  }

  gtsam::Pose3 p = ekf_model_->pose();
  gtsam::Vector6 v = ekf_model_->velocity();
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

  // Twist in body frame: [angular, linear] -> ROS convention [linear, angular]
  // velocity_ layout: [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
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

void EidosTransformNode::mapSourceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  gtsam::Pose3 map_to_base = poseStampedToPose3(*msg);

  last_map_to_base_ = map_to_base;

  // Look up the EKF pose at the SLAM timestamp for precise correction
  double slam_time = rclcpp::Time(msg->header.stamp).seconds();
  gtsam::Pose3 odom_to_base;
  double matched_time = 0.0;
  {
    std::lock_guard lock(map_to_odom_mtx_);

    // Find the EKF pose with the largest timestamp <= slam_time
    odom_to_base = ekf_model_->pose();  // fallback to current
    for (auto it = ekf_pose_history_.rbegin(); it != ekf_pose_history_.rend(); ++it) {
      if (it->time <= slam_time) {
        odom_to_base = it->pose;
        matched_time = it->time;
        break;
      }
    }

    cached_map_to_odom_ = map_to_base.compose(odom_to_base.inverse());
  }
  has_map_to_odom_.store(true, std::memory_order_release);

  auto ct = cached_map_to_odom_.translation();
  auto cr = cached_map_to_odom_.rotation().rpy();
  auto mt = map_to_base.translation();
  auto mr = map_to_base.rotation().rpy();
  auto ot = odom_to_base.translation();
  auto or_ = odom_to_base.rotation().rpy();
  RCLCPP_INFO(
    get_logger(),
    "\033[34m[Transform]\033[0m map->odom updated: "
    "slam_pose=(%.2f,%.2f,%.2f rpy=%.2f,%.2f,%.2f) "
    "odom_at_t=(%.2f,%.2f,%.2f rpy=%.2f,%.2f,%.2f) "
    "correction=(%.2f,%.2f,%.2f rpy=%.2f,%.2f,%.2f) slam_t=%.3f matched_t=%.3f dt=%.4f",
    mt.x(), mt.y(), mt.z(), mr.x(), mr.y(), mr.z(),
    ot.x(), ot.y(), ot.z(), or_.x(), or_.y(), or_.z(),
    ct.x(), ct.y(), ct.z(), cr.x(), cr.y(), cr.z(),
    slam_time, matched_time, slam_time - matched_time);
}

void EidosTransformNode::utmToMapCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  cached_utm_to_map_ = *msg;
  cached_utm_to_map_.header.frame_id = utm_frame_;
  cached_utm_to_map_.child_frame_id = map_frame_;
  has_utm_to_map_ = true;

  // Broadcast as static transform.
  static_tf_broadcaster_->sendTransform(cached_utm_to_map_);
  RCLCPP_INFO(get_logger(), "Published static utm->map transform.");
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

  // Integrate the current velocity forward by dt to produce a relative transform.
  gtsam::Vector6 vel;
  {
    std::lock_guard lock(sources_mutex_);
    vel = ekf_model_->velocity();
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
  // Layout: [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
  gtsam::Vector6 twist;
  twist << a.x, a.y, a.z, l.x, l.y, l.z;
  return twist;
}

gtsam::Pose3 EidosTransformNode::poseStampedToPose3(const geometry_msgs::msg::PoseStamped & msg)
{
  const auto & p = msg.pose.position;
  const auto & q = msg.pose.orientation;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
}

}  // namespace eidos_transform
