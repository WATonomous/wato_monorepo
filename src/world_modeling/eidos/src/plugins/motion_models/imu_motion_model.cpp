#include "eidos/plugins/motion_models/imu_motion_model.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include "eidos/slam_core.hpp"

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

namespace eidos {

// Named constants
constexpr double kTfLookupTimeout = 5.0;               // seconds, timeout for static TF lookup
constexpr double kQuatSquaredNormMin = 0.01;            // minimum q.squaredNorm() to accept orientation
constexpr double kMinIntegrationDt = 1e-6;              // minimum dt before falling back to default_imu_dt

// ---------------------------------------------------------------------------
// Static utilities
// ---------------------------------------------------------------------------
std::pair<gtsam::Vector3, gtsam::Vector3>
ImuMotionModel::extractMeasurement(const sensor_msgs::msg::Imu& msg) {
  return {
    gtsam::Vector3(msg.linear_acceleration.x,
                   msg.linear_acceleration.y,
                   msg.linear_acceleration.z),
    gtsam::Vector3(msg.angular_velocity.x,
                   msg.angular_velocity.y,
                   msg.angular_velocity.z)
  };
}

bool ImuMotionModel::isValidImuMessage(const sensor_msgs::msg::Imu& msg) {
  return !(msg.angular_velocity.x == 0.0 && msg.angular_velocity.y == 0.0 &&
           msg.angular_velocity.z == 0.0 && msg.linear_acceleration.x == 0.0 &&
           msg.linear_acceleration.y == 0.0 && msg.linear_acceleration.z == 0.0);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void ImuMotionModel::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".acc_cov", std::vector<double>{9.0e-6, 9.0e-6, 9.0e-6});
  node_->declare_parameter(prefix + ".gyr_cov", std::vector<double>{1.0e-6, 1.0e-6, 1.0e-6});
  node_->declare_parameter(prefix + ".bias_walk_cov",
      std::vector<double>{1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6});
  node_->declare_parameter(prefix + ".gravity", 9.80511);
  node_->declare_parameter(prefix + ".integration_cov", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-4});
  node_->declare_parameter(prefix + ".initialization.prior_vel_cov",
      std::vector<double>{1e-2, 1e-2, 1e-2});
  node_->declare_parameter(prefix + ".initialization.prior_bias_cov",
      std::vector<double>{1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6});
  node_->declare_parameter(prefix + ".default_imu_dt", 1.0 / 500.0);
  node_->declare_parameter(prefix + ".initialization.quaternion_norm_threshold", 0.1);
  node_->declare_parameter(prefix + ".initialization.stationary_acc_threshold", 0.05);
  node_->declare_parameter(prefix + ".initialization.stationary_gyr_threshold", 0.005);
  node_->declare_parameter(prefix + ".initialization.stationary_samples", 200);
  node_->declare_parameter(prefix + ".odom_topic", name_ + "/odometry/imu_incremental");
  node_->declare_parameter(prefix + ".imu_frame", "imu_link");
  node_->declare_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->declare_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);

  // ---- Read parameters ----
  std::string imu_topic;
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".acc_cov", acc_cov_);
  node_->get_parameter(prefix + ".gyr_cov", gyr_cov_);
  node_->get_parameter(prefix + ".bias_walk_cov", bias_walk_cov_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".integration_cov", integration_cov_);
  node_->get_parameter(prefix + ".initialization.prior_vel_cov", prior_vel_cov_);
  node_->get_parameter(prefix + ".initialization.prior_bias_cov", prior_bias_cov_);
  node_->get_parameter(prefix + ".default_imu_dt", default_imu_dt_);
  node_->get_parameter(prefix + ".initialization.quaternion_norm_threshold", quaternion_norm_threshold_);
  node_->get_parameter(prefix + ".initialization.stationary_acc_threshold", stationary_acc_threshold_);
  node_->get_parameter(prefix + ".initialization.stationary_gyr_threshold", stationary_gyr_threshold_);
  node_->get_parameter(prefix + ".initialization.stationary_samples", stationary_samples_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->get_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);

  // ---- Set up GTSAM preintegration parameters ----
  auto p = gtsam::PreintegrationParams::MakeSharedU(gravity_);
  p->accelerometerCovariance =
      Eigen::Vector3d(acc_cov_[0], acc_cov_[1], acc_cov_[2]).asDiagonal();
  p->gyroscopeCovariance =
      Eigen::Vector3d(gyr_cov_[0], gyr_cov_[1], gyr_cov_[2]).asDiagonal();
  p->integrationCovariance =
      Eigen::Vector3d(integration_cov_[0], integration_cov_[1], integration_cov_[2]).asDiagonal();
  preint_params_ = p;

  gtsam::imuBias::ConstantBias prior_bias;
  imu_integrator_imu_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, prior_bias);

  // ---- Noise models ----
  prior_vel_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(3) << prior_vel_cov_[0], prior_vel_cov_[1], prior_vel_cov_[2]).finished());
  prior_bias_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << prior_bias_cov_[0], prior_bias_cov_[1], prior_bias_cov_[2],
       prior_bias_cov_[3], prior_bias_cov_[4], prior_bias_cov_[5]).finished());
  noise_between_bias_ =
      (gtsam::Vector(6) << std::sqrt(bias_walk_cov_[0]), std::sqrt(bias_walk_cov_[1]),
       std::sqrt(bias_walk_cov_[2]), std::sqrt(bias_walk_cov_[3]),
       std::sqrt(bias_walk_cov_[4]), std::sqrt(bias_walk_cov_[5]))
          .finished();

  // ---- Create subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuMotionModel::imuCallback, this, std::placeholders::_1),
      sub_opts);

  // ---- Create publisher ----
  std::string odom_topic;
  node_->get_parameter(prefix + ".odom_topic", odom_topic);

  imu_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic, 10);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void ImuMotionModel::activate() {
  active_ = true;
  imu_odom_pub_->on_activate();

  // Look up the static transform: imu_frame_ -> base_link_frame_
  try {
    tf_imu_to_base_msg_ = tf_->lookupTransform(
        base_link_frame_, imu_frame_, tf2::TimePointZero, tf2::durationFromSec(kTfLookupTimeout));
    extrinsics_resolved_ = true;

    const auto& q = tf_imu_to_base_msg_.transform.rotation;
    const auto& t = tf_imu_to_base_msg_.transform.translation;
    RCLCPP_INFO(node_->get_logger(),
                "[%s] resolved TF %s -> %s: t=[%.3f, %.3f, %.3f] q=[%.4f, %.4f, %.4f, %.4f]",
                name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str(),
                t.x, t.y, t.z, q.x, q.y, q.z, q.w);
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] failed to look up TF %s -> %s: %s. Using identity transform.",
                 name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str(), e.what());
    tf_imu_to_base_msg_.header.frame_id = base_link_frame_;
    tf_imu_to_base_msg_.child_frame_id = imu_frame_;
    tf_imu_to_base_msg_.transform.rotation.w = 1.0;
    extrinsics_resolved_ = false;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void ImuMotionModel::deactivate() {
  active_ = false;
  imu_odom_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void ImuMotionModel::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.clear();
  }
  stationary_buffer_.clear();
  stationary_acc_rms_.store(0.0, std::memory_order_relaxed);
  stationary_gyr_rms_.store(0.0, std::memory_order_relaxed);
  stationary_count_.store(0, std::memory_order_relaxed);
  gtsam::imuBias::ConstantBias zero_bias;
  imu_integrator_imu_->resetIntegrationAndSetBias(zero_bias);
  odom_reference_state_ = gtsam::NavState();
  odom_reference_bias_ = gtsam::imuBias::ConstantBias();
  prev_state_ = gtsam::NavState();
  prev_vel_ = gtsam::Vector3::Zero();
  prev_bias_ = gtsam::imuBias::ConstantBias();
  last_imu_t_imu_ = -1;
  last_generated_time_ = -1;
  initialized_ = false;
  has_imu_pose_ = false;
}

// ---------------------------------------------------------------------------
// isReady / getReadyStatus
// ---------------------------------------------------------------------------
bool ImuMotionModel::isReady() const {
  int count = stationary_count_.load(std::memory_order_relaxed);
  if (count < stationary_samples_) return false;
  double acc_rms = stationary_acc_rms_.load(std::memory_order_relaxed);
  double gyr_rms = stationary_gyr_rms_.load(std::memory_order_relaxed);
  return acc_rms < stationary_acc_threshold_ && gyr_rms < stationary_gyr_threshold_;
}

std::string ImuMotionModel::getReadyStatus() const {
  int count = stationary_count_.load(std::memory_order_relaxed);
  if (count == 0) return "no IMU data received";
  if (count < stationary_samples_) {
    return "buffering (" + std::to_string(count) + "/"
           + std::to_string(stationary_samples_) + " samples)";
  }

  double acc_rms = stationary_acc_rms_.load(std::memory_order_relaxed);
  double gyr_rms = stationary_gyr_rms_.load(std::memory_order_relaxed);

  char buf[160];
  std::snprintf(buf, sizeof(buf),
      "acc_rms=%.4f (thr %.4f) %s, gyr_rms=%.5f (thr %.5f) %s",
      acc_rms, stationary_acc_threshold_,
      acc_rms < stationary_acc_threshold_ ? "OK" : "HIGH",
      gyr_rms, stationary_gyr_threshold_,
      gyr_rms < stationary_gyr_threshold_ ? "OK" : "HIGH");
  return buf;
}

// ---------------------------------------------------------------------------
// getInitialOrientation — average roll/pitch from stationary buffer
// ---------------------------------------------------------------------------
std::optional<gtsam::Rot3> ImuMotionModel::getInitialOrientation() const {
  if (stationary_buffer_.empty()) return std::nullopt;

  Eigen::Vector3d gravity_sum = Eigen::Vector3d::Zero();
  int count = 0;

  for (const auto& msg : stationary_buffer_) {
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                         msg.orientation.y, msg.orientation.z);
    if (q.squaredNorm() < kQuatSquaredNormMin) continue;
    q.normalize();

    Eigen::Vector3d gravity_body = q.inverse() * Eigen::Vector3d(0, 0, -1);
    gravity_sum += gravity_body;
    count++;
  }

  if (count == 0) return std::nullopt;

  Eigen::Vector3d avg_gravity = (gravity_sum / count).normalized();

  double pitch = std::asin(std::clamp(avg_gravity.x(), -1.0, 1.0));
  double roll = std::atan2(-avg_gravity.y(), -avg_gravity.z());

  RCLCPP_INFO(node_->get_logger(),
      "[%s] gravity alignment from %d samples: roll=%.2f° pitch=%.2f°",
      name_.c_str(), count, roll * 180.0 / M_PI, pitch * 180.0 / M_PI);

  return gtsam::Rot3::RzRyRx(roll, pitch, 0.0);
}

// ---------------------------------------------------------------------------
// getCurrentPose
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> ImuMotionModel::getCurrentPose() const {
  if (!active_) return std::nullopt;
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  if (!has_imu_pose_) return std::nullopt;
  return latest_imu_pose_;
}

// ---------------------------------------------------------------------------
// onStateZero — initialize V/B priors at state 0
// ---------------------------------------------------------------------------
void ImuMotionModel::onStateZero(
    gtsam::Key key, double /*timestamp*/,
    const gtsam::Pose3& initial_pose,
    gtsam::NonlinearFactorGraph& factors,
    gtsam::Values& values) {
  prev_vel_ = gtsam::Vector3::Zero();
  prev_bias_ = gtsam::imuBias::ConstantBias();
  prev_state_ = gtsam::NavState(initial_pose, prev_vel_);

  // Set up odometry reference (fixed for the lifetime of this tracking session)
  odom_reference_state_ = prev_state_;
  odom_reference_bias_ = prev_bias_;
  imu_integrator_imu_->resetIntegrationAndSetBias(odom_reference_bias_);

  // Discard all pre-initialization IMU messages
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.clear();
  }

  gtsam::Symbol sym(key);

  // Add priors for V and B
  factors.push_back(
      gtsam::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          V(sym.index()), prev_vel_, prior_vel_noise_));
  factors.push_back(
      gtsam::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          B(sym.index()), prev_bias_, prior_bias_noise_));

  values.insert(V(sym.index()), prev_vel_);
  values.insert(B(sym.index()), prev_bias_);

  last_generated_key_ = key;
  last_generated_time_ = -1;
  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(),
      "[%s] initialized at state (%c,%lu) [%.3f, %.3f, %.3f]",
      name_.c_str(), sym.chr(), sym.index(),
      initial_pose.translation().x(), initial_pose.translation().y(),
      initial_pose.translation().z());
}

// ---------------------------------------------------------------------------
// generateMotionModel — drain IMU queue from t_begin to t_end, create ImuFactor
// ---------------------------------------------------------------------------
void ImuMotionModel::drainAndIntegrate(
    gtsam::PreintegratedImuMeasurements& integrator,
    double t_begin, double t_end) {
  integrator.resetIntegrationAndSetBias(prev_bias_);

  std::lock_guard<std::mutex> lock(imu_lock_);

  double last_time = -1;
  while (!imu_queue_.empty()) {
    double imu_time = stamp2Sec(imu_queue_.front().header.stamp);
    if (imu_time <= t_begin) {
      imu_queue_.pop_front();
      continue;
    }
    if (imu_time > t_end) break;

    double dt = (last_time < 0) ? default_imu_dt_ : (imu_time - last_time);
    if (dt <= 0) dt = default_imu_dt_;
    auto [acc, gyr] = extractMeasurement(imu_queue_.front());
    integrator.integrateMeasurement(acc, gyr, dt);
    last_time = imu_time;
    imu_queue_.pop_front();
  }
}

void ImuMotionModel::generateMotionModel(
    gtsam::Key key_begin, double t_begin,
    gtsam::Key key_end, double t_end,
    gtsam::NonlinearFactorGraph& factors,
    gtsam::Values& values) {
  if (!initialized_) return;

  gtsam::Symbol sym_begin(key_begin);
  gtsam::Symbol sym_end(key_end);

  gtsam::PreintegratedImuMeasurements integrator(preint_params_, prev_bias_);
  drainAndIntegrate(integrator, t_begin, t_end);

  double dt = integrator.deltaTij();
  if (dt <= 0) {
    dt = std::abs(t_end - t_begin);
    if (dt < kMinIntegrationDt) dt = default_imu_dt_;
    integrator.integrateMeasurement(gtsam::Vector3::Zero(), gtsam::Vector3::Zero(), dt);
    dt = integrator.deltaTij();
  }

  // Add ImuFactor
  factors.push_back(
      gtsam::make_shared<gtsam::ImuFactor>(
          key_begin, V(sym_begin.index()),
          key_end, V(sym_end.index()),
          B(sym_begin.index()), integrator));

  // Add BetweenFactor<Bias> with time-scaled noise
  factors.push_back(
      gtsam::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
          B(sym_begin.index()), B(sym_end.index()),
          gtsam::imuBias::ConstantBias(),
          gtsam::noiseModel::Diagonal::Sigmas(
              std::sqrt(dt) * noise_between_bias_)));

  // Predict initial values for new state
  gtsam::NavState prop_state = integrator.predict(prev_state_, prev_bias_);

  if (!values.exists(V(sym_end.index()))) {
    values.insert(V(sym_end.index()), prop_state.v());
  }
  if (!values.exists(B(sym_end.index()))) {
    values.insert(B(sym_end.index()), prev_bias_);
  }

  // Update prev_state_ so successive calls chain correctly
  prev_state_ = prop_state;
  prev_vel_ = prop_state.v();

  RCLCPP_INFO(node_->get_logger(),
      "\033[35m[%s] ImuFactor (%c,%lu)->(%c,%lu) dt=%.3f |v|=%.3f\033[0m",
      name_.c_str(),
      sym_begin.chr(), sym_begin.index(),
      sym_end.chr(), sym_end.index(),
      dt, prop_state.v().norm());

  last_generated_key_ = key_end;
  last_generated_time_ = t_end;
}

// ---------------------------------------------------------------------------
// Callback -- high-rate IMU processing
// ---------------------------------------------------------------------------
void ImuMotionModel::updateStationaryDetection(
    const sensor_msgs::msg::Imu& imu_converted) {
  if (stationary_buffer_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] first IMU message received",
                name_.c_str());
  }
  stationary_buffer_.push_back(imu_converted);
  while (static_cast<int>(stationary_buffer_.size()) > stationary_samples_) {
    stationary_buffer_.pop_front();
  }

  double acc_sq = 0.0, gyr_sq = 0.0;
  for (const auto& m : stationary_buffer_) {
    double az_compensated = m.linear_acceleration.z - gravity_;
    acc_sq += m.linear_acceleration.x * m.linear_acceleration.x
            + m.linear_acceleration.y * m.linear_acceleration.y
            + az_compensated * az_compensated;
    gyr_sq += m.angular_velocity.x * m.angular_velocity.x
            + m.angular_velocity.y * m.angular_velocity.y
            + m.angular_velocity.z * m.angular_velocity.z;
  }
  int n = static_cast<int>(stationary_buffer_.size());
  stationary_acc_rms_.store(std::sqrt(acc_sq / n), std::memory_order_relaxed);
  stationary_gyr_rms_.store(std::sqrt(gyr_sq / n), std::memory_order_relaxed);
  stationary_count_.store(n, std::memory_order_relaxed);
}

gtsam::NavState ImuMotionModel::integrateSingleAndPredict(
    const sensor_msgs::msg::Imu& imu_converted) {
  double imu_time = stamp2Sec(imu_converted.header.stamp);
  double dt =
      (last_imu_t_imu_ < 0) ? default_imu_dt_ : (imu_time - last_imu_t_imu_);
  last_imu_t_imu_ = imu_time;

  auto [acc, gyr] = extractMeasurement(imu_converted);
  imu_integrator_imu_->integrateMeasurement(acc, gyr, dt);

  return imu_integrator_imu_->predict(odom_reference_state_, odom_reference_bias_);
}

void ImuMotionModel::storeImuPose(const gtsam::Pose3& base_pose) {
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  latest_imu_pose_ = base_pose;
  has_imu_pose_ = true;
}

void ImuMotionModel::publishIncrementalOdom(
    const sensor_msgs::msg::Imu& imu_converted,
    const gtsam::Pose3& base_pose,
    const gtsam::NavState& state) {
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = imu_converted.header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_link_frame_;

  odom.pose.pose.position.x = base_pose.translation().x();
  odom.pose.pose.position.y = base_pose.translation().y();
  odom.pose.pose.position.z = base_pose.translation().z();
  odom.pose.pose.orientation.x = base_pose.rotation().toQuaternion().x();
  odom.pose.pose.orientation.y = base_pose.rotation().toQuaternion().y();
  odom.pose.pose.orientation.z = base_pose.rotation().toQuaternion().z();
  odom.pose.pose.orientation.w = base_pose.rotation().toQuaternion().w();

  odom.twist.twist.linear.x = state.velocity().x();
  odom.twist.twist.linear.y = state.velocity().y();
  odom.twist.twist.linear.z = state.velocity().z();
  odom.twist.twist.angular.x = imu_converted.angular_velocity.x;
  odom.twist.twist.angular.y = imu_converted.angular_velocity.y;
  odom.twist.twist.angular.z = imu_converted.angular_velocity.z;

  // Pose covariance diagonal (6x6 row-major: x, y, z, roll, pitch, yaw)
  odom.pose.covariance[0]  = odom_pose_cov_[0];
  odom.pose.covariance[7]  = odom_pose_cov_[1];
  odom.pose.covariance[14] = odom_pose_cov_[2];
  odom.pose.covariance[21] = odom_pose_cov_[3];
  odom.pose.covariance[28] = odom_pose_cov_[4];
  odom.pose.covariance[35] = odom_pose_cov_[5];

  // Twist covariance diagonal
  odom.twist.covariance[0]  = odom_twist_cov_[0];
  odom.twist.covariance[7]  = odom_twist_cov_[1];
  odom.twist.covariance[14] = odom_twist_cov_[2];
  odom.twist.covariance[21] = odom_twist_cov_[3];
  odom.twist.covariance[28] = odom_twist_cov_[4];
  odom.twist.covariance[35] = odom_twist_cov_[5];

  imu_odom_pub_->publish(odom);
}

void ImuMotionModel::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!isValidImuMessage(*msg)) return;

  sensor_msgs::msg::Imu imu_converted = imuConverter(*msg);
  updateStationaryDetection(imu_converted);

  if (core_->getState() != SlamState::TRACKING) return;

  // Buffer for generateMotionModel() stitching
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.push_back(imu_converted);
  }

  // Real-time odometry prediction
  auto current_state = integrateSingleAndPredict(imu_converted);
  gtsam::Pose3 base_pose(current_state.quaternion(), current_state.position());
  storeImuPose(base_pose);
  publishIncrementalOdom(imu_converted, base_pose, current_state);
}

// ---------------------------------------------------------------------------
// IMU extrinsic conversion
// ---------------------------------------------------------------------------
sensor_msgs::msg::Imu ImuMotionModel::imuConverter(
    const sensor_msgs::msg::Imu& imu_in) {
  sensor_msgs::msg::Imu imu_out = imu_in;

  geometry_msgs::msg::Vector3Stamped acc_in, acc_out;
  acc_in.header.frame_id = imu_frame_;
  acc_in.vector = imu_in.linear_acceleration;
  tf2::doTransform(acc_in, acc_out, tf_imu_to_base_msg_);
  imu_out.linear_acceleration = acc_out.vector;

  geometry_msgs::msg::Vector3Stamped gyr_in, gyr_out;
  gyr_in.header.frame_id = imu_frame_;
  gyr_in.vector = imu_in.angular_velocity;
  tf2::doTransform(gyr_in, gyr_out, tf_imu_to_base_msg_);
  imu_out.angular_velocity = gyr_out.vector;

  tf2::Quaternion q_imu;
  tf2::fromMsg(imu_in.orientation, q_imu);
  tf2::Quaternion q_base_imu;
  tf2::fromMsg(tf_imu_to_base_msg_.transform.rotation, q_base_imu);
  tf2::Quaternion q_base = q_imu * q_base_imu.inverse();
  if (q_base.length() < quaternion_norm_threshold_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid quaternion, please use a 9-axis IMU!");
  }
  q_base.normalize();
  imu_out.orientation = tf2::toMsg(q_base);

  return imu_out;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::ImuMotionModel, eidos::MotionModelPlugin)
