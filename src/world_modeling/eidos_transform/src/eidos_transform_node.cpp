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
// Helper: parse a list of MeasurementSource entries from parameters
// ---------------------------------------------------------------------------

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

    node->declare_parameter<std::string>(src_name + ".odom_topic", "");
    node->get_parameter(src_name + ".odom_topic", src.odom_topic);
    if (src.odom_topic.empty()) {
      RCLCPP_WARN(node->get_logger(), "%s source '%s' has no topic, skipping.", label.c_str(), src_name.c_str());
      continue;
    }

    // Pose mask
    node->declare_parameter<std::vector<bool>>(src_name + ".pose_mask", {false, false, false, false, false, false});
    std::vector<bool> pm;
    node->get_parameter(src_name + ".pose_mask", pm);
    for (size_t i = 0; i < 6 && i < pm.size(); ++i) src.pose_mask[i] = pm[i];

    // Twist mask
    node->declare_parameter<std::vector<bool>>(src_name + ".twist_mask", {false, false, false, false, false, false});
    std::vector<bool> tm;
    node->get_parameter(src_name + ".twist_mask", tm);
    for (size_t i = 0; i < 6 && i < tm.size(); ++i) src.twist_mask[i] = tm[i];

    // Pose noise
    node->declare_parameter<std::vector<double>>(src_name + ".pose_noise", {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
    std::vector<double> pn;
    node->get_parameter(src_name + ".pose_noise", pn);
    for (size_t i = 0; i < 6 && i < pn.size(); ++i) src.pose_noise(static_cast<int>(i)) = pn[i];

    // Twist noise
    node->declare_parameter<std::vector<double>>(src_name + ".twist_noise", {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
    std::vector<double> tn;
    node->get_parameter(src_name + ".twist_noise", tn);
    for (size_t i = 0; i < 6 && i < tn.size(); ++i) src.twist_noise(static_cast<int>(i)) = tn[i];

    // Subscriber
    size_t idx = out.size();
    src.subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
      src.odom_topic,
      rclcpp::SensorDataQoS(),
      [&out, &mtx, idx](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx);
        if (idx < out.size()) {
          out[idx].latest_msg = msg;
          out[idx].has_new_data = true;
        }
      });

    RCLCPP_INFO(node->get_logger(), "%s source '%s' on topic '%s'", label.c_str(), src_name.c_str(), src.odom_topic.c_str());
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
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
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
  static_tf_broadcaster_.reset();
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

  // Predict both EKFs
  local_ekf_->predict(dt);
  global_ekf_->predict(dt);

  // Fuse odom_sources into local EKF only (odom frame data)
  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (auto & src : odom_sources_) {
      if (!src.has_new_data || !src.latest_msg) continue;
      src.has_new_data = false;

      fuseSource(local_ekf_, src);

      if (!src.logged_first_msg) {
        RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First odom source '%s' on '%s'",
          src.name.c_str(), src.odom_topic.c_str());
        src.logged_first_msg = true;
      }
    }

    // Fuse map_sources into global EKF ONLY
    for (auto & src : map_sources_) {
      if (!src.has_new_data || !src.latest_msg) continue;
      src.has_new_data = false;
      has_map_source_data_ = true;

      fuseSource(global_ekf_, src);

      if (!src.logged_first_msg) {
        RCLCPP_INFO(get_logger(), "\033[34m[Transform]\033[0m First map source '%s' on '%s'",
          src.name.c_str(), src.odom_topic.c_str());
        src.logged_first_msg = true;
      }
    }
  }

  // Broadcast odom→base_link from local EKF
  broadcastOdomToBaseTF(now);

  // Broadcast map→odom from global EKF + TF lookup
  broadcastMapToOdomTF(now);

  // Publish fused odom from local EKF
  publishOdometry(now);

  // Debug log
  auto lp = local_ekf_->pose().translation();
  auto gp = global_ekf_->pose().translation();
  RCLCPP_INFO(get_logger(),
    "\033[34m[Transform]\033[0m dt=%.4f local=(%.2f,%.2f,%.2f) global=(%.2f,%.2f,%.2f) map_src=%s",
    dt, lp.x(), lp.y(), lp.z(), gp.x(), gp.y(), gp.z(),
    has_map_source_data_ ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Fuse a measurement source into an EKF
// ---------------------------------------------------------------------------

void EidosTransformNode::fuseSource(
  std::shared_ptr<EKFModelPlugin> & ekf,
  MeasurementSource & src)
{
  gtsam::Pose3 meas_pose = odomMsgToPose3(*src.latest_msg);
  gtsam::Vector6 meas_twist = odomMsgToTwist(*src.latest_msg);

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

  // Look up odom→base_link from TF (published by local EKF moments ago)
  gtsam::Pose3 odom_to_base;
  try {
    auto tf = tf_buffer_->lookupTransform(odom_frame_, base_link_frame_, stamp, rclcpp::Duration(0, 0));
    const auto & t = tf.transform.translation;
    const auto & r = tf.transform.rotation;
    odom_to_base = gtsam::Pose3(
      gtsam::Rot3::Quaternion(r.w, r.x, r.y, r.z), gtsam::Point3(t.x, t.y, t.z));
  } catch (const tf2::TransformException &) {
    // TF not available yet (first few ticks), use local EKF directly
    odom_to_base = local_ekf_->pose();
  }

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

  static_tf_broadcaster_->sendTransform(cached_utm_to_map_);
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
