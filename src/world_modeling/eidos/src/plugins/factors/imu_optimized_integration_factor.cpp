#include "eidos/plugins/factors/imu_optimized_integration_factor.hpp"

#include <fstream>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include "eidos/slam_core.hpp"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

namespace eidos {

// ---------------------------------------------------------------------------
// Static utilities
// ---------------------------------------------------------------------------
std::pair<gtsam::Vector3, gtsam::Vector3>
ImuOptimizedIntegrationFactor::extractMeasurement(const sensor_msgs::msg::Imu& msg) {
  return {
    gtsam::Vector3(msg.linear_acceleration.x,
                   msg.linear_acceleration.y,
                   msg.linear_acceleration.z),
    gtsam::Vector3(msg.angular_velocity.x,
                   msg.angular_velocity.y,
                   msg.angular_velocity.z)
  };
}

bool ImuOptimizedIntegrationFactor::isValidImuMessage(const sensor_msgs::msg::Imu& msg) {
  return !(msg.angular_velocity.x == 0.0 && msg.angular_velocity.y == 0.0 &&
           msg.angular_velocity.z == 0.0 && msg.linear_acceleration.x == 0.0 &&
           msg.linear_acceleration.y == 0.0 && msg.linear_acceleration.z == 0.0);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void ImuOptimizedIntegrationFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".acc_noise", 3.9939570888238808e-03);
  node_->declare_parameter(prefix + ".gyr_noise", 1.5636343949698187e-03);
  node_->declare_parameter(prefix + ".acc_bias_noise", 6.4356659353532566e-05);
  node_->declare_parameter(prefix + ".gyr_bias_noise", 3.5640318696367613e-05);
  node_->declare_parameter(prefix + ".gravity", 9.80511);
  node_->declare_parameter(prefix + ".integration_noise", 1e-4);
  node_->declare_parameter(prefix + ".prior_pose_sigma", 1e-2);
  node_->declare_parameter(prefix + ".prior_vel_sigma", 1e4);
  node_->declare_parameter(prefix + ".prior_bias_sigma", 1e-3);
  node_->declare_parameter(prefix + ".correction_rot_sigma", 0.05);
  node_->declare_parameter(prefix + ".correction_trans_sigma", 0.1);
  node_->declare_parameter(prefix + ".correction_degrade_sigma", 1.0);
  node_->declare_parameter(prefix + ".isam2_relinearize_threshold", 0.1);
  node_->declare_parameter(prefix + ".max_velocity", 30.0);
  node_->declare_parameter(prefix + ".max_bias_norm", 1.0);
  node_->declare_parameter(prefix + ".default_imu_dt", 1.0 / 500.0);
  node_->declare_parameter(prefix + ".quaternion_norm_threshold", 0.1);
  node_->declare_parameter(prefix + ".graph_reset_interval", 100);
  node_->declare_parameter(prefix + ".stationary_acc_threshold", 0.05);
  node_->declare_parameter(prefix + ".stationary_gyr_threshold", 0.005);
  node_->declare_parameter(prefix + ".stationary_samples", 200);
  node_->declare_parameter(prefix + ".odom_rot_noise", 1e-2);
  node_->declare_parameter(prefix + ".odom_trans_noise", 1e-1);
  node_->declare_parameter(prefix + ".odom_incremental_topic", name_ + "/odometry/imu_incremental");
  node_->declare_parameter(prefix + ".odom_fused_topic", name_ + "/odometry/imu");
  node_->declare_parameter(prefix + ".imu_frame", "imu_link");
  node_->declare_parameter(prefix + ".imu_odom_child_frame", "odom_imu");

  // ---- Read parameters ----
  std::string imu_topic;
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".acc_noise", acc_noise_);
  node_->get_parameter(prefix + ".gyr_noise", gyr_noise_);
  node_->get_parameter(prefix + ".acc_bias_noise", acc_bias_noise_);
  node_->get_parameter(prefix + ".gyr_bias_noise", gyr_bias_noise_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".integration_noise", integration_noise_);
  node_->get_parameter(prefix + ".prior_pose_sigma", prior_pose_sigma_);
  node_->get_parameter(prefix + ".prior_vel_sigma", prior_vel_sigma_);
  node_->get_parameter(prefix + ".prior_bias_sigma", prior_bias_sigma_);
  node_->get_parameter(prefix + ".correction_rot_sigma", correction_rot_sigma_);
  node_->get_parameter(prefix + ".correction_trans_sigma", correction_trans_sigma_);
  node_->get_parameter(prefix + ".correction_degrade_sigma", correction_degrade_sigma_);
  node_->get_parameter(prefix + ".isam2_relinearize_threshold", isam2_relinearize_threshold_);
  node_->get_parameter(prefix + ".max_velocity", max_velocity_);
  node_->get_parameter(prefix + ".max_bias_norm", max_bias_norm_);
  node_->get_parameter(prefix + ".default_imu_dt", default_imu_dt_);
  node_->get_parameter(prefix + ".quaternion_norm_threshold", quaternion_norm_threshold_);
  node_->get_parameter(prefix + ".graph_reset_interval", graph_reset_interval_);
  node_->get_parameter(prefix + ".stationary_acc_threshold", stationary_acc_threshold_);
  node_->get_parameter(prefix + ".stationary_gyr_threshold", stationary_gyr_threshold_);
  node_->get_parameter(prefix + ".stationary_samples", stationary_samples_);
  node_->get_parameter(prefix + ".odom_rot_noise", odom_rot_noise_);
  node_->get_parameter(prefix + ".odom_trans_noise", odom_trans_noise_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".imu_odom_child_frame", imu_odom_child_frame_);

  // ---- Set up GTSAM preintegration parameters ----
  auto p = gtsam::PreintegrationParams::MakeSharedU(gravity_);
  p->accelerometerCovariance =
      gtsam::Matrix33::Identity() * std::pow(acc_noise_, 2);
  p->gyroscopeCovariance =
      gtsam::Matrix33::Identity() * std::pow(gyr_noise_, 2);
  p->integrationCovariance =
      gtsam::Matrix33::Identity() * std::pow(integration_noise_, 2);
  preint_params_ = p;

  gtsam::imuBias::ConstantBias prior_bias;
  imu_integrator_opt_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, prior_bias);
  imu_integrator_imu_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, prior_bias);

  // ---- Noise models ----
  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_pose_sigma_, prior_pose_sigma_, prior_pose_sigma_,
       prior_pose_sigma_, prior_pose_sigma_, prior_pose_sigma_).finished());
  prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, prior_vel_sigma_);
  prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, prior_bias_sigma_);
  correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << correction_rot_sigma_, correction_rot_sigma_, correction_rot_sigma_,
       correction_trans_sigma_, correction_trans_sigma_, correction_trans_sigma_).finished());
  correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << correction_degrade_sigma_, correction_degrade_sigma_, correction_degrade_sigma_,
       correction_degrade_sigma_, correction_degrade_sigma_, correction_degrade_sigma_).finished());
  noise_between_bias_ =
      (gtsam::Vector(6) << acc_bias_noise_, acc_bias_noise_, acc_bias_noise_,
       gyr_bias_noise_, gyr_bias_noise_, gyr_bias_noise_)
          .finished();

  // ---- Create subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuOptimizedIntegrationFactor::imuCallback, this,
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
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

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

void ImuOptimizedIntegrationFactor::activate() {
  active_ = true;
  imu_odom_incremental_pub_->on_activate();
  imu_odom_fused_pub_->on_activate();

  // Look up the static transform: imu_frame_ → base_link_frame_
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

void ImuOptimizedIntegrationFactor::deactivate() {
  active_ = false;
  imu_odom_incremental_pub_->on_deactivate();
  imu_odom_fused_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void ImuOptimizedIntegrationFactor::reset() {
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
  prev_bias_ = gtsam::imuBias::ConstantBias();
  prev_state_odom_ = gtsam::NavState();
  prev_bias_odom_ = gtsam::imuBias::ConstantBias();
  last_imu_t_imu_ = -1;
  last_imu_t_opt_ = -1;
  key_ = 1;
  done_first_opt_ = false;
  initialized_ = false;
  has_imu_pose_ = false;
  has_last_graph_pose_ = false;
}

void ImuOptimizedIntegrationFactor::resetOptimization() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = isam2_relinearize_threshold_;
  params.relinearizeSkip = 1;
  optimizer_ = gtsam::ISAM2(params);
  graph_factors_.resize(0);
  graph_values_.clear();
}

bool ImuOptimizedIntegrationFactor::failureDetection(
    const gtsam::Vector3& vel,
    const gtsam::imuBias::ConstantBias& bias) {
  Eigen::Vector3f v(vel.x(), vel.y(), vel.z());
  if (v.norm() > max_velocity_) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Large velocity (%.1f m/s), reset IMU-preintegration!",
                name_.c_str(), static_cast<double>(v.norm()));
    return true;
  }

  Eigen::Vector3f ba(bias.accelerometer().x(), bias.accelerometer().y(),
                     bias.accelerometer().z());
  Eigen::Vector3f bg(bias.gyroscope().x(), bias.gyroscope().y(),
                     bias.gyroscope().z());
  if (ba.norm() > max_bias_norm_ || bg.norm() > max_bias_norm_) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Large bias, reset IMU-preintegration!", name_.c_str());
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// isReady / getReadyStatus — stationary detection (zero-velocity gate)
//
// A gravity-compensated IMU at constant velocity reads near-zero, just like
// at rest.  Thresholds must therefore be tight enough that only genuine
// standstill passes.  Tune via config if needed.
// ---------------------------------------------------------------------------
bool ImuOptimizedIntegrationFactor::isReady() const {
  int count = stationary_count_.load(std::memory_order_relaxed);
  if (count < stationary_samples_) return false;
  double acc_rms = stationary_acc_rms_.load(std::memory_order_relaxed);
  double gyr_rms = stationary_gyr_rms_.load(std::memory_order_relaxed);
  return acc_rms < stationary_acc_threshold_ && gyr_rms < stationary_gyr_threshold_;
}

std::string ImuOptimizedIntegrationFactor::getReadyStatus() const {
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
std::optional<gtsam::Pose3> ImuOptimizedIntegrationFactor::processFrame(
    double /*timestamp*/) {
  if (!active_) return std::nullopt;
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  if (!has_imu_pose_) return std::nullopt;
  return latest_imu_pose_;
}

// ---------------------------------------------------------------------------
// getFactors - BetweenFactor odometry for main pose graph + internal bookkeeping
// ---------------------------------------------------------------------------
std::vector<gtsam::NonlinearFactor::shared_ptr>
ImuOptimizedIntegrationFactor::getFactors(int state_index,
                                 const gtsam::Pose3& state_pose,
                                 double /*timestamp*/) {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  // Store bias and velocity estimates in MapManager
  if (initialized_) {
    auto& map_manager = core_->getMapManager();
    map_manager.addKeyframeData(state_index, "imu_integration_factor/bias",
                                prev_bias_);
    map_manager.addKeyframeData(state_index, "imu_integration_factor/velocity",
                                prev_vel_);
  }

  // Add BetweenFactor connecting consecutive states in main graph
  if (state_index == 0) {
    last_graph_pose_ = state_pose;
    has_last_graph_pose_ = true;
  } else if (has_last_graph_pose_) {
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << odom_rot_noise_, odom_rot_noise_, odom_rot_noise_,
         odom_trans_noise_, odom_trans_noise_, odom_trans_noise_).finished());
    gtsam::Pose3 relative = last_graph_pose_.between(state_pose);
    auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        state_index - 1, state_index, relative, noise);
    factors.push_back(factor);
    last_graph_pose_ = state_pose;
  }

  return factors;
}

// ---------------------------------------------------------------------------
// Integration helpers
// ---------------------------------------------------------------------------
void ImuOptimizedIntegrationFactor::drainAndIntegrate(
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

void ImuOptimizedIntegrationFactor::reintegrateQueue(
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
// onOptimizationComplete — correction handler
//
// Called after the main SLAM graph optimizes. Uses the graph-optimized pose
// as a correction for the internal IMU ISAM2 to estimate biases.
// ---------------------------------------------------------------------------
double ImuOptimizedIntegrationFactor::getCorrectionTime() const {
  double correction_time = node_->now().seconds();
  auto poses_6d = core_->getMapManager().getKeyPoses6D();
  if (!poses_6d->empty()) {
    correction_time = static_cast<double>(poses_6d->points.back().time);
  }
  return correction_time;
}

void ImuOptimizedIntegrationFactor::storeGraphCorrection(
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

void ImuOptimizedIntegrationFactor::initializeIsam2(
    const gtsam::Pose3& graph_pose, double correction_time) {
  resetOptimization();

  // Pop old IMU messages
  while (!imu_queue_opt_.empty()) {
    if (stamp2Sec(imu_queue_opt_.front().header.stamp) < correction_time) {
      last_imu_t_opt_ = stamp2Sec(imu_queue_opt_.front().header.stamp);
      imu_queue_opt_.pop_front();
    } else {
      break;
    }
  }

  // Initial pose (in base_link frame — matches the converted IMU measurements)
  prev_pose_ = graph_pose;
  graph_factors_.add(
      gtsam::PriorFactor<gtsam::Pose3>(X(0), prev_pose_, prior_pose_noise_));

  // Initial velocity (zero)
  prev_vel_ = gtsam::Vector3(0, 0, 0);
  graph_factors_.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(0), prev_vel_, prior_vel_noise_));

  // Initial bias (zero)
  prev_bias_ = gtsam::imuBias::ConstantBias();
  graph_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          B(0), prev_bias_, prior_bias_noise_));

  // Add initial values
  graph_values_.insert(X(0), prev_pose_);
  graph_values_.insert(V(0), prev_vel_);
  graph_values_.insert(B(0), prev_bias_);

  // Optimize once
  optimizer_.update(graph_factors_, graph_values_);
  graph_factors_.resize(0);
  graph_values_.clear();

  imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_);
  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

  prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
  prev_state_odom_ = prev_state_;
  prev_bias_odom_ = prev_bias_;

  key_ = 1;
  initialized_ = true;
  done_first_opt_ = true;

  RCLCPP_INFO(node_->get_logger(),
      "[%s] internal ISAM2 initialized at [%.3f, %.3f, %.3f]",
      name_.c_str(),
      prev_pose_.translation().x(), prev_pose_.translation().y(),
      prev_pose_.translation().z());
}

void ImuOptimizedIntegrationFactor::resetGraphIfNeeded() {
  if (key_ != graph_reset_interval_) return;

  auto updated_pose_noise = gtsam::noiseModel::Gaussian::Covariance(
      optimizer_.marginalCovariance(X(key_ - 1)));
  auto updated_vel_noise = gtsam::noiseModel::Gaussian::Covariance(
      optimizer_.marginalCovariance(V(key_ - 1)));
  auto updated_bias_noise = gtsam::noiseModel::Gaussian::Covariance(
      optimizer_.marginalCovariance(B(key_ - 1)));

  resetOptimization();

  graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), prev_pose_, updated_pose_noise));
  graph_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(0), prev_vel_, updated_vel_noise));
  graph_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(0), prev_bias_, updated_bias_noise));

  graph_values_.insert(X(0), prev_pose_);
  graph_values_.insert(V(0), prev_vel_);
  graph_values_.insert(B(0), prev_bias_);

  optimizer_.update(graph_factors_, graph_values_);
  graph_factors_.resize(0);
  graph_values_.clear();

  key_ = 1;
}

bool ImuOptimizedIntegrationFactor::integrateAndOptimize(
    const gtsam::Pose3& graph_pose, double correction_time) {
  // Integrate IMU data between corrections
  drainAndIntegrate(imu_queue_opt_, *imu_integrator_opt_,
                    correction_time, last_imu_t_opt_);

  // Add IMU factor to internal graph
  const auto& preint_imu = dynamic_cast<
      const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_opt_);
  graph_factors_.add(gtsam::ImuFactor(
      X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu));

  // Add bias between factor
  graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(
          std::sqrt(imu_integrator_opt_->deltaTij()) *
          noise_between_bias_)));

  // Add pose correction factor (base_link frame — matches converted measurements)
  graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(key_), graph_pose, correction_noise_));

  // Insert predicted values
  gtsam::NavState prop_state =
      imu_integrator_opt_->predict(prev_state_, prev_bias_);
  graph_values_.insert(X(key_), prop_state.pose());
  graph_values_.insert(V(key_), prop_state.v());
  graph_values_.insert(B(key_), prev_bias_);

  // Optimize
  optimizer_.update(graph_factors_, graph_values_);
  optimizer_.update();
  graph_factors_.resize(0);
  graph_values_.clear();

  // Get results
  gtsam::Values result = optimizer_.calculateEstimate();
  prev_pose_ = result.at<gtsam::Pose3>(X(key_));
  prev_vel_ = result.at<gtsam::Vector3>(V(key_));
  prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
  prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));

  // Reset optimization integrator with updated bias
  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

  // Failure detection
  if (failureDetection(prev_vel_, prev_bias_)) {
    resetOptimization();
    imu_integrator_opt_->resetIntegrationAndSetBias(
        gtsam::imuBias::ConstantBias());
    imu_integrator_imu_->resetIntegrationAndSetBias(
        gtsam::imuBias::ConstantBias());
    prev_state_ = gtsam::NavState();
    prev_bias_ = gtsam::imuBias::ConstantBias();
    prev_state_odom_ = gtsam::NavState();
    prev_bias_odom_ = gtsam::imuBias::ConstantBias();
    key_ = 1;
    initialized_ = false;
    done_first_opt_ = false;
    has_imu_pose_ = false;
    last_imu_t_imu_ = -1;
    last_imu_t_opt_ = -1;
    return false;
  }

  return true;
}

void ImuOptimizedIntegrationFactor::repropagateBias() {
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

void ImuOptimizedIntegrationFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!active_) return;

  std::lock_guard<std::mutex> lock(imu_lock_);

  int state_index = core_->getCurrentStateIndex();
  if (state_index < 0) return;
  if (!optimized_values.exists(state_index)) return;

  auto graph_pose = optimized_values.at<gtsam::Pose3>(state_index);
  double correction_time = getCorrectionTime();
  storeGraphCorrection(graph_pose, correction_time);

  if (imu_queue_opt_.empty()) return;

  if (!initialized_) {
    initializeIsam2(graph_pose, correction_time);
    return;
  }

  resetGraphIfNeeded();
  if (!integrateAndOptimize(graph_pose, correction_time)) return;
  repropagateBias();
  ++key_;
}

// ---------------------------------------------------------------------------
// Callback — high-rate IMU processing
// ---------------------------------------------------------------------------
void ImuOptimizedIntegrationFactor::updateStationaryDetection(
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
    acc_sq += m.linear_acceleration.x * m.linear_acceleration.x
            + m.linear_acceleration.y * m.linear_acceleration.y
            + m.linear_acceleration.z * m.linear_acceleration.z;
    gyr_sq += m.angular_velocity.x * m.angular_velocity.x
            + m.angular_velocity.y * m.angular_velocity.y
            + m.angular_velocity.z * m.angular_velocity.z;
  }
  int n = static_cast<int>(stationary_buffer_.size());
  stationary_acc_rms_.store(std::sqrt(acc_sq / n), std::memory_order_relaxed);
  stationary_gyr_rms_.store(std::sqrt(gyr_sq / n), std::memory_order_relaxed);
  stationary_count_.store(n, std::memory_order_relaxed);
}

gtsam::NavState ImuOptimizedIntegrationFactor::integrateSingleAndPredict(
    const sensor_msgs::msg::Imu& imu_converted) {
  double imu_time = stamp2Sec(imu_converted.header.stamp);
  double dt =
      (last_imu_t_imu_ < 0) ? default_imu_dt_ : (imu_time - last_imu_t_imu_);
  last_imu_t_imu_ = imu_time;

  auto [acc, gyr] = extractMeasurement(imu_converted);
  imu_integrator_imu_->integrateMeasurement(acc, gyr, dt);

  return imu_integrator_imu_->predict(prev_state_odom_, prev_bias_odom_);
}

void ImuOptimizedIntegrationFactor::storeImuPose(const gtsam::Pose3& base_pose) {
  std::lock_guard<std::mutex> lock(imu_pose_lock_);
  latest_imu_pose_ = base_pose;
  has_imu_pose_ = true;
}

void ImuOptimizedIntegrationFactor::publishIncrementalOdom(
    const sensor_msgs::msg::Imu& imu_converted,
    const gtsam::Pose3& base_pose,
    const gtsam::NavState& state) {
  auto odom_incremental = nav_msgs::msg::Odometry();
  odom_incremental.header.stamp = imu_converted.header.stamp;
  odom_incremental.header.frame_id = odom_frame_;
  odom_incremental.child_frame_id = imu_odom_child_frame_;

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

void ImuOptimizedIntegrationFactor::publishFusedOdom(
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

  // Broadcast TF: odom -> base_link (using fused pose)
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = imu_converted.header.stamp;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_link_frame_;
  tf_msg.transform.translation.x = fused_t.x();
  tf_msg.transform.translation.y = fused_t.y();
  tf_msg.transform.translation.z = fused_t.z();
  tf_msg.transform.rotation.x = fused_q.x();
  tf_msg.transform.rotation.y = fused_q.y();
  tf_msg.transform.rotation.z = fused_q.z();
  tf_msg.transform.rotation.w = fused_q.w();
  tf_broadcaster_->sendTransform(tf_msg);
}

void ImuOptimizedIntegrationFactor::imuCallback(
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
// IMU extrinsic conversion — transforms measurements from imu_frame_ to base_link_frame_
// ---------------------------------------------------------------------------
sensor_msgs::msg::Imu ImuOptimizedIntegrationFactor::imuConverter(
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

PLUGINLIB_EXPORT_CLASS(eidos::ImuOptimizedIntegrationFactor, eidos::FactorPlugin)
