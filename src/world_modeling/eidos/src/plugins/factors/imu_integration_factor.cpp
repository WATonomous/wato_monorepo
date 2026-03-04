#include "eidos/plugins/factors/imu_integration_factor.hpp"

#include <cmath>
#include <fstream>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "eidos/slam_core.hpp"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

namespace eidos {

// ---------------------------------------------------------------------------
// Static utilities
// ---------------------------------------------------------------------------
std::pair<gtsam::Vector3, gtsam::Vector3>
ImuIntegrationFactor::extractMeasurement(const sensor_msgs::msg::Imu& msg) {
  return {
    gtsam::Vector3(msg.linear_acceleration.x,
                   msg.linear_acceleration.y,
                   msg.linear_acceleration.z),
    gtsam::Vector3(msg.angular_velocity.x,
                   msg.angular_velocity.y,
                   msg.angular_velocity.z)
  };
}

bool ImuIntegrationFactor::isValidImuMessage(const sensor_msgs::msg::Imu& msg) {
  return !(msg.angular_velocity.x == 0.0 && msg.angular_velocity.y == 0.0 &&
           msg.angular_velocity.z == 0.0 && msg.linear_acceleration.x == 0.0 &&
           msg.linear_acceleration.y == 0.0 && msg.linear_acceleration.z == 0.0);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void ImuIntegrationFactor::onInitialize() {
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
  node_->declare_parameter(prefix + ".prior_vel_cov",
      std::vector<double>{1e-2, 1e-2, 1e-2});
  node_->declare_parameter(prefix + ".prior_bias_cov",
      std::vector<double>{1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6});
  node_->declare_parameter(prefix + ".attitude_prior_cov",
      std::vector<double>{1e-2, 1e-2});
  node_->declare_parameter(prefix + ".use_attitude_prior", true);
  node_->declare_parameter(prefix + ".default_imu_dt", 1.0 / 500.0);
  node_->declare_parameter(prefix + ".quaternion_norm_threshold", 0.1);
  node_->declare_parameter(prefix + ".stationary_acc_threshold", 0.05);
  node_->declare_parameter(prefix + ".stationary_gyr_threshold", 0.005);
  node_->declare_parameter(prefix + ".stationary_samples", 200);
  node_->declare_parameter(prefix + ".odom_incremental_topic", name_ + "/odometry/imu_incremental");
  node_->declare_parameter(prefix + ".odom_fused_topic", name_ + "/odometry/imu");
  node_->declare_parameter(prefix + ".imu_frame", "imu_link");

  // ---- Read parameters ----
  std::string imu_topic;
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".acc_cov", acc_cov_);
  node_->get_parameter(prefix + ".gyr_cov", gyr_cov_);
  node_->get_parameter(prefix + ".bias_walk_cov", bias_walk_cov_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".integration_cov", integration_cov_);
  node_->get_parameter(prefix + ".prior_vel_cov", prior_vel_cov_);
  node_->get_parameter(prefix + ".prior_bias_cov", prior_bias_cov_);
  node_->get_parameter(prefix + ".attitude_prior_cov", attitude_prior_cov_);
  node_->get_parameter(prefix + ".use_attitude_prior", use_attitude_prior_);
  node_->get_parameter(prefix + ".default_imu_dt", default_imu_dt_);
  node_->get_parameter(prefix + ".quaternion_norm_threshold", quaternion_norm_threshold_);
  node_->get_parameter(prefix + ".stationary_acc_threshold", stationary_acc_threshold_);
  node_->get_parameter(prefix + ".stationary_gyr_threshold", stationary_gyr_threshold_);
  node_->get_parameter(prefix + ".stationary_samples", stationary_samples_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);

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
  imu_integrator_opt_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, prior_bias);
  imu_integrator_imu_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, prior_bias);

  // ---- Noise models ----
  prior_vel_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(3) << prior_vel_cov_[0], prior_vel_cov_[1], prior_vel_cov_[2]).finished());
  prior_bias_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << prior_bias_cov_[0], prior_bias_cov_[1], prior_bias_cov_[2],
       prior_bias_cov_[3], prior_bias_cov_[4], prior_bias_cov_[5]).finished());
  attitude_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(2) << attitude_prior_cov_[0], attitude_prior_cov_[1]).finished());
  // BetweenFactor<Bias> uses sigmas (sqrt of covariance)
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
      std::bind(&ImuIntegrationFactor::imuCallback, this,
                std::placeholders::_1),
      sub_opts);

  // ---- Create publishers ----
  std::string odom_incremental_topic, odom_fused_topic;
  node_->get_parameter(prefix + ".odom_incremental_topic", odom_incremental_topic);
  node_->get_parameter(prefix + ".odom_fused_topic", odom_fused_topic);

  imu_odom_incremental_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_incremental_topic, 10);
  imu_odom_fused_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_fused_topic, 10);
  // ---- Register keyframe data types with MapManager ----
  auto& map_manager = core_->getMapManager();
  map_manager.registerType("imu_integration_factor/bias", {
    [](const std::any& data, const std::string& path) {
      auto bias = std::any_cast<gtsam::imuBias::ConstantBias>(data);
      std::ofstream ofs(path, std::ios::binary);
      auto acc = bias.accelerometer();
      auto gyr = bias.gyroscope();
      ofs.write(reinterpret_cast<const char*>(acc.data()), 3 * sizeof(double));
      ofs.write(reinterpret_cast<const char*>(gyr.data()), 3 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      double acc[3], gyr[3];
      ifs.read(reinterpret_cast<char*>(acc), 3 * sizeof(double));
      ifs.read(reinterpret_cast<char*>(gyr), 3 * sizeof(double));
      return gtsam::imuBias::ConstantBias(
          gtsam::Vector3(acc[0], acc[1], acc[2]),
          gtsam::Vector3(gyr[0], gyr[1], gyr[2]));
    }
  });
  map_manager.registerType("imu_integration_factor/velocity", {
    [](const std::any& data, const std::string& path) {
      auto vel = std::any_cast<gtsam::Vector3>(data);
      std::ofstream ofs(path, std::ios::binary);
      ofs.write(reinterpret_cast<const char*>(vel.data()), 3 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      double v[3];
      ifs.read(reinterpret_cast<char*>(v), 3 * sizeof(double));
      return gtsam::Vector3(v[0], v[1], v[2]);
    }
  });

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void ImuIntegrationFactor::activate() {
  active_ = true;
  imu_odom_incremental_pub_->on_activate();
  imu_odom_fused_pub_->on_activate();

  // Look up the static transform: imu_frame_ -> base_link_frame_
  try {
    tf_imu_to_base_msg_ = tf_->lookupTransform(
        base_link_frame_, imu_frame_, tf2::TimePointZero, tf2::durationFromSec(5.0));
    extrinsics_resolved_ = true;

    const auto& q = tf_imu_to_base_msg_.transform.rotation;
    const auto& t = tf_imu_to_base_msg_.transform.translation;
    RCLCPP_INFO(node_->get_logger(),
                "[%s] resolved TF %s -> %s: t=[%.3f, %.3f, %.3f] q=[%.4f, %.4f, %.4f, %.4f]",
                name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str(),
                t.x, t.y, t.z, q.x, q.y, q.z, q.w);
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] failed to look up TF %s -> %s: %s. "
                 "Using identity transform.",
                 name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str(), e.what());
    tf_imu_to_base_msg_.header.frame_id = base_link_frame_;
    tf_imu_to_base_msg_.child_frame_id = imu_frame_;
    tf_imu_to_base_msg_.transform.rotation.w = 1.0;
    extrinsics_resolved_ = false;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void ImuIntegrationFactor::deactivate() {
  active_ = false;
  imu_odom_incremental_pub_->on_deactivate();
  imu_odom_fused_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void ImuIntegrationFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_opt_.clear();
    imu_queue_imu_.clear();
    imu_odom_queue_.clear();
  }
  // Stationary buffer is callback-only; safe to clear while deactivated
  stationary_buffer_.clear();
  stationary_acc_rms_.store(0.0, std::memory_order_relaxed);
  stationary_gyr_rms_.store(0.0, std::memory_order_relaxed);
  stationary_count_.store(0, std::memory_order_relaxed);
  graph_odom_affine_ = Eigen::Isometry3d::Identity();
  graph_odom_time_ = -1;
  gtsam::imuBias::ConstantBias zero_bias;
  imu_integrator_opt_->resetIntegrationAndSetBias(zero_bias);
  imu_integrator_imu_->resetIntegrationAndSetBias(zero_bias);
  prev_state_ = gtsam::NavState();
  prev_vel_ = gtsam::Vector3::Zero();
  prev_bias_ = gtsam::imuBias::ConstantBias();
  prev_state_odom_ = gtsam::NavState();
  prev_bias_odom_ = gtsam::imuBias::ConstantBias();
  last_imu_t_imu_ = -1;
  last_imu_t_opt_ = -1;
  initialized_ = false;
  has_imu_pose_ = false;
}

// ---------------------------------------------------------------------------
// isReady / getReadyStatus -- stationary detection (zero-velocity gate)
//
// A gravity-compensated IMU at constant velocity reads near-zero, just like
// at rest.  Thresholds must therefore be tight enough that only genuine
// standstill passes.  Tune via config if needed.
// ---------------------------------------------------------------------------
bool ImuIntegrationFactor::isReady() const {
  int count = stationary_count_.load(std::memory_order_relaxed);
  if (count < stationary_samples_) return false;
  double acc_rms = stationary_acc_rms_.load(std::memory_order_relaxed);
  double gyr_rms = stationary_gyr_rms_.load(std::memory_order_relaxed);
  return acc_rms < stationary_acc_threshold_ && gyr_rms < stationary_gyr_threshold_;
}

std::string ImuIntegrationFactor::getReadyStatus() const {
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
// processFrame - return latest IMU-predicted pose
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> ImuIntegrationFactor::processFrame(
    double /*timestamp*/) {
  if (!active_) return std::nullopt;
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  if (!has_imu_pose_) return std::nullopt;
  return latest_imu_pose_;
}

// ---------------------------------------------------------------------------
// getFactors - ImuFactor for unified pose graph
// ---------------------------------------------------------------------------
FactorResult
ImuIntegrationFactor::getFactors(int state_index,
                                 const gtsam::Pose3& state_pose,
                                 double /*timestamp*/) {
  FactorResult result;

  if (state_index == 0) {
    // Initialize velocity and bias at state 0
    prev_vel_ = gtsam::Vector3::Zero();
    prev_bias_ = gtsam::imuBias::ConstantBias();
    prev_state_ = gtsam::NavState(state_pose, prev_vel_);
    prev_state_odom_ = prev_state_;
    prev_bias_odom_ = prev_bias_;

    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
    imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_);

    // Discard all pre-initialization IMU messages so state 1's
    // drainAndIntegrate only sees data from after state 0.
    {
      std::lock_guard<std::mutex> lock(imu_lock_);
      imu_queue_opt_.clear();
    }

    // Add priors for V(0) and B(0) to the main graph
    result.factors.push_back(
        gtsam::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
            V(0), prev_vel_, prior_vel_noise_));
    result.factors.push_back(
        gtsam::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
            B(0), prev_bias_, prior_bias_noise_));

    result.values.insert(V(0), prev_vel_);
    result.values.insert(B(0), prev_bias_);

    initialized_ = true;

    RCLCPP_INFO(node_->get_logger(),
        "[%s] initialized at state 0 [%.3f, %.3f, %.3f]",
        name_.c_str(),
        state_pose.translation().x(), state_pose.translation().y(),
        state_pose.translation().z());

  } else if (initialized_) {
    std::lock_guard<std::mutex> lock(imu_lock_);

    // Drain all available IMU messages into the optimization integrator
    drainAndIntegrate(imu_queue_opt_, *imu_integrator_opt_,
                      std::numeric_limits<double>::max(), last_imu_t_opt_);

    if (imu_integrator_opt_->deltaTij() > 0) {
      // Add ImuFactor connecting consecutive states
      result.factors.push_back(
          gtsam::make_shared<gtsam::ImuFactor>(
              state_index - 1, V(state_index - 1),
              state_index, V(state_index),
              B(state_index - 1), *imu_integrator_opt_));

      // Add bias between factor (noise scales with sqrt of integration time)
      result.factors.push_back(
          gtsam::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
              B(state_index - 1), B(state_index),
              gtsam::imuBias::ConstantBias(),
              gtsam::noiseModel::Diagonal::Sigmas(
                  std::sqrt(imu_integrator_opt_->deltaTij()) *
                  noise_between_bias_)));

      // Predict values for new state
      gtsam::NavState prop_state =
          imu_integrator_opt_->predict(prev_state_, prev_bias_);
      result.values.insert(V(state_index), prop_state.v());
      result.values.insert(B(state_index), prev_bias_);

      RCLCPP_INFO(node_->get_logger(),
          "\033[35m[%s] added ImuFactor %d->%d (dt=%.3f s, |v|=%.3f)\033[0m",
          name_.c_str(), state_index - 1, state_index,
          imu_integrator_opt_->deltaTij(), prop_state.v().norm());

      // Gravity alignment: constrain body Z to stay near world Z
      // Prevents optimizer from tilting pitch/roll to resolve GPS-IMU disagreements
      if (use_attitude_prior_) {
        result.factors.push_back(
            gtsam::make_shared<gtsam::Pose3AttitudeFactor>(
                state_index, gtsam::Unit3(0, 0, 1), attitude_noise_));
      }

      // Reset integrator with current bias
      imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
    }
  }

  // Store bias and velocity estimates in MapManager
  if (initialized_) {
    auto& map_manager = core_->getMapManager();
    map_manager.addKeyframeData(state_index, "imu_integration_factor/bias",
                                prev_bias_);
    map_manager.addKeyframeData(state_index, "imu_integration_factor/velocity",
                                prev_vel_);
  }

  return result;
}

// ---------------------------------------------------------------------------
// Integration helpers
// ---------------------------------------------------------------------------
void ImuIntegrationFactor::drainAndIntegrate(
    std::deque<sensor_msgs::msg::Imu>& queue,
    gtsam::PreintegratedImuMeasurements& integrator,
    double cutoff_time, double& last_time) {
  while (!queue.empty()) {
    double imu_time = stamp2Sec(queue.front().header.stamp);
    if (imu_time >= cutoff_time) break;
    double dt = (last_time < 0) ? default_imu_dt_ : (imu_time - last_time);
    auto [acc, gyr] = extractMeasurement(queue.front());
    integrator.integrateMeasurement(acc, gyr, dt);
    last_time = imu_time;
    queue.pop_front();
  }
}

void ImuIntegrationFactor::reintegrateQueue(
    const std::deque<sensor_msgs::msg::Imu>& queue,
    gtsam::PreintegratedImuMeasurements& integrator,
    double last_time) {
  for (const auto& msg : queue) {
    double imu_time = stamp2Sec(msg.header.stamp);
    double dt = (last_time < 0) ? default_imu_dt_ : (imu_time - last_time);
    auto [acc, gyr] = extractMeasurement(msg);
    integrator.integrateMeasurement(acc, gyr, dt);
    last_time = imu_time;
  }
}

// ---------------------------------------------------------------------------
// onOptimizationComplete -- extract V/B from optimized values
//
// Called after the main SLAM graph optimizes. Extracts the jointly-optimized
// velocity and bias, updates internal state, and repropagates the real-time
// IMU integrator with the new bias.
// ---------------------------------------------------------------------------
double ImuIntegrationFactor::getCorrectionTime() const {
  double correction_time = node_->now().seconds();
  auto poses_6d = core_->getMapManager().getKeyPoses6D();
  if (!poses_6d->empty()) {
    correction_time = static_cast<double>(poses_6d->points.back().time);
  }
  return correction_time;
}

void ImuIntegrationFactor::storeGraphCorrection(
    const gtsam::Pose3& graph_pose, double correction_time) {
  auto q = graph_pose.rotation().toQuaternion();
  auto t_vec = graph_pose.translation();
  graph_odom_affine_ = Eigen::Isometry3d::Identity();
  graph_odom_affine_.linear() =
      Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
  graph_odom_affine_.translation() =
      Eigen::Vector3d(t_vec.x(), t_vec.y(), t_vec.z());
  // Use sensor time (last_imu_t_opt_) for transform fusion pruning
  // so bag playback without sim_time works correctly.
  graph_odom_time_ = (last_imu_t_opt_ > 0) ? last_imu_t_opt_ : correction_time;
}

void ImuIntegrationFactor::repropagateBias() {
  prev_state_odom_ = prev_state_;
  prev_bias_odom_ = prev_bias_;

  // Pop old messages from the real-time queue
  // Use last_imu_t_opt_ (sensor time) instead of correction_time (wall clock)
  // so that bag playback without sim_time works correctly.
  double last_imu_qt = -1;
  while (!imu_queue_imu_.empty() &&
         stamp2Sec(imu_queue_imu_.front().header.stamp) < last_imu_t_opt_) {
    last_imu_qt = stamp2Sec(imu_queue_imu_.front().header.stamp);
    imu_queue_imu_.pop_front();
  }

  // Re-propagate remaining messages with new bias
  if (!imu_queue_imu_.empty()) {
    imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_odom_);
    reintegrateQueue(imu_queue_imu_, *imu_integrator_imu_, last_imu_qt);
  }
}

void ImuIntegrationFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!active_ || !initialized_) return;

  std::lock_guard<std::mutex> lock(imu_lock_);

  int state_index = core_->getCurrentStateIndex();
  if (state_index < 0) return;
  if (!optimized_values.exists(state_index)) return;

  auto graph_pose = optimized_values.at<gtsam::Pose3>(state_index);

  // Extract velocity and bias from the jointly-optimized values
  gtsam::Vector3 graph_vel = prev_vel_;
  if (optimized_values.exists(V(state_index))) {
    graph_vel = optimized_values.at<gtsam::Vector3>(V(state_index));
  }
  gtsam::imuBias::ConstantBias graph_bias = prev_bias_;
  if (optimized_values.exists(B(state_index))) {
    graph_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index));
  }

  // Update state from optimized values
  prev_state_ = gtsam::NavState(graph_pose, graph_vel);
  prev_vel_ = graph_vel;
  prev_bias_ = graph_bias;


  // Reset optimization integrator with updated bias
  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

  // Store graph correction for fused odometry
  double correction_time = getCorrectionTime();
  storeGraphCorrection(graph_pose, correction_time);

  // Update real-time prediction state and repropagate bias
  repropagateBias();
}

// ---------------------------------------------------------------------------
// Callback -- high-rate IMU processing
// ---------------------------------------------------------------------------
void ImuIntegrationFactor::updateStationaryDetection(
    const sensor_msgs::msg::Imu& imu_converted) {
  if (stationary_buffer_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] first IMU message received",
                name_.c_str());
  }
  stationary_buffer_.push_back(imu_converted);
  while (static_cast<int>(stationary_buffer_.size()) > stationary_samples_) {
    stationary_buffer_.pop_front();
  }
  // Compute RMS of gravity-compensated acceleration and angular velocity.
  // imu_converted is in base_link frame (MakeSharedU -> gravity along +Z),
  // so subtract gravity_ from the z-axis to handle raw IMUs.
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

gtsam::NavState ImuIntegrationFactor::integrateSingleAndPredict(
    const sensor_msgs::msg::Imu& imu_converted) {
  double imu_time = stamp2Sec(imu_converted.header.stamp);
  double dt =
      (last_imu_t_imu_ < 0) ? default_imu_dt_ : (imu_time - last_imu_t_imu_);
  last_imu_t_imu_ = imu_time;

  auto [acc, gyr] = extractMeasurement(imu_converted);
  imu_integrator_imu_->integrateMeasurement(acc, gyr, dt);

  return imu_integrator_imu_->predict(prev_state_odom_, prev_bias_odom_);
}

void ImuIntegrationFactor::storeImuPose(const gtsam::Pose3& base_pose) {
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  latest_imu_pose_ = base_pose;
  has_imu_pose_ = true;
}

void ImuIntegrationFactor::publishIncrementalOdom(
    const sensor_msgs::msg::Imu& imu_converted,
    const gtsam::Pose3& base_pose,
    const gtsam::NavState& state) {
  auto odom_incremental = nav_msgs::msg::Odometry();
  odom_incremental.header.stamp = imu_converted.header.stamp;
  odom_incremental.header.frame_id = odom_frame_;
  odom_incremental.child_frame_id = base_link_frame_;

  odom_incremental.pose.pose.position.x = base_pose.translation().x();
  odom_incremental.pose.pose.position.y = base_pose.translation().y();
  odom_incremental.pose.pose.position.z = base_pose.translation().z();
  odom_incremental.pose.pose.orientation.x = base_pose.rotation().toQuaternion().x();
  odom_incremental.pose.pose.orientation.y = base_pose.rotation().toQuaternion().y();
  odom_incremental.pose.pose.orientation.z = base_pose.rotation().toQuaternion().z();
  odom_incremental.pose.pose.orientation.w = base_pose.rotation().toQuaternion().w();

  odom_incremental.twist.twist.linear.x = state.velocity().x();
  odom_incremental.twist.twist.linear.y = state.velocity().y();
  odom_incremental.twist.twist.linear.z = state.velocity().z();
  odom_incremental.twist.twist.angular.x =
      imu_converted.angular_velocity.x + prev_bias_odom_.gyroscope().x();
  odom_incremental.twist.twist.angular.y =
      imu_converted.angular_velocity.y + prev_bias_odom_.gyroscope().y();
  odom_incremental.twist.twist.angular.z =
      imu_converted.angular_velocity.z + prev_bias_odom_.gyroscope().z();

  imu_odom_incremental_pub_->publish(odom_incremental);

  // Queue for fusion
  imu_odom_queue_.push_back(odom_incremental);
}

void ImuIntegrationFactor::publishFusedOdom(
    const sensor_msgs::msg::Imu& imu_converted) {
  // Can only fuse once we have a graph correction
  if (graph_odom_time_ < 0) return;

  // Prune queue: remove entries older than latest graph correction
  while (!imu_odom_queue_.empty()) {
    if (stamp2Sec(imu_odom_queue_.front().header.stamp) <= graph_odom_time_)
      imu_odom_queue_.pop_front();
    else
      break;
  }

  if (imu_odom_queue_.empty()) return;

  // Compute IMU increment since graph correction
  auto odom_to_affine = [](const nav_msgs::msg::Odometry& o) {
    Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
    a.linear() = Eigen::Quaterniond(
        o.pose.pose.orientation.w, o.pose.pose.orientation.x,
        o.pose.pose.orientation.y, o.pose.pose.orientation.z)
                     .toRotationMatrix();
    a.translation() = Eigen::Vector3d(
        o.pose.pose.position.x, o.pose.pose.position.y,
        o.pose.pose.position.z);
    return a;
  };

  Eigen::Isometry3d imu_odom_front = odom_to_affine(imu_odom_queue_.front());
  Eigen::Isometry3d imu_odom_back = odom_to_affine(imu_odom_queue_.back());
  Eigen::Isometry3d imu_increment = imu_odom_front.inverse() * imu_odom_back;

  // Fused = graph correction * IMU increment
  Eigen::Isometry3d fused = graph_odom_affine_ * imu_increment;
  Eigen::Quaterniond fused_q(fused.linear());
  Eigen::Vector3d fused_t = fused.translation();

  // Publish fused odometry
  auto odom_fused = imu_odom_queue_.back();  // copy twist/covariance from latest
  odom_fused.header.frame_id = odom_frame_;
  odom_fused.child_frame_id = base_link_frame_;
  odom_fused.pose.pose.position.x = fused_t.x();
  odom_fused.pose.pose.position.y = fused_t.y();
  odom_fused.pose.pose.position.z = fused_t.z();
  odom_fused.pose.pose.orientation.x = fused_q.x();
  odom_fused.pose.pose.orientation.y = fused_q.y();
  odom_fused.pose.pose.orientation.z = fused_q.z();
  odom_fused.pose.pose.orientation.w = fused_q.w();
  imu_odom_fused_pub_->publish(odom_fused);

}

void ImuIntegrationFactor::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!isValidImuMessage(*msg)) return;

  sensor_msgs::msg::Imu imu_converted = imuConverter(*msg);
  updateStationaryDetection(imu_converted);

  if (core_->getState() != SlamState::TRACKING) return;

  std::lock_guard<std::mutex> lock(imu_lock_);

  imu_queue_opt_.push_back(imu_converted);
  imu_queue_imu_.push_back(imu_converted);

  auto current_state = integrateSingleAndPredict(imu_converted);
  gtsam::Pose3 base_pose(current_state.quaternion(), current_state.position());
  storeImuPose(base_pose);
  publishIncrementalOdom(imu_converted, base_pose, current_state);
  publishFusedOdom(imu_converted);
}

// ---------------------------------------------------------------------------
// IMU extrinsic conversion -- transforms measurements from imu_frame_ to base_link_frame_
// ---------------------------------------------------------------------------
sensor_msgs::msg::Imu ImuIntegrationFactor::imuConverter(
    const sensor_msgs::msg::Imu& imu_in) {
  sensor_msgs::msg::Imu imu_out = imu_in;

  // Transform acceleration (free vector: only rotation, no translation)
  geometry_msgs::msg::Vector3Stamped acc_in, acc_out;
  acc_in.header.frame_id = imu_frame_;
  acc_in.vector = imu_in.linear_acceleration;
  tf2::doTransform(acc_in, acc_out, tf_imu_to_base_msg_);
  imu_out.linear_acceleration = acc_out.vector;

  // Transform angular velocity
  geometry_msgs::msg::Vector3Stamped gyr_in, gyr_out;
  gyr_in.header.frame_id = imu_frame_;
  gyr_in.vector = imu_in.angular_velocity;
  tf2::doTransform(gyr_in, gyr_out, tf_imu_to_base_msg_);
  imu_out.angular_velocity = gyr_out.vector;

  // Transform orientation: q_world_base = q_world_imu * inverse(q_base_imu)
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

PLUGINLIB_EXPORT_CLASS(eidos::ImuIntegrationFactor, eidos::FactorPlugin)
