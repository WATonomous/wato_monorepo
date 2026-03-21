#include "eidos/plugins/motion_models/imu_motion_model.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace eidos {

constexpr double kTfLookupTimeout = 5.0;

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
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".acc_cov", std::vector<double>{9.0e-6, 9.0e-6, 9.0e-6});
  node_->declare_parameter(prefix + ".gyr_cov", std::vector<double>{1.0e-6, 1.0e-6, 1.0e-6});
  node_->declare_parameter(prefix + ".gravity", 9.80511);
  node_->declare_parameter(prefix + ".integration_cov", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-4});
  node_->declare_parameter(prefix + ".default_imu_dt", 1.0 / 500.0);
  node_->declare_parameter(prefix + ".initialization.quaternion_norm_threshold", 0.1);
  node_->declare_parameter(prefix + ".initialization.stationary_acc_threshold", 0.05);
  node_->declare_parameter(prefix + ".initialization.stationary_gyr_threshold", 0.005);
  node_->declare_parameter(prefix + ".initialization.stationary_samples", 200);
  node_->declare_parameter(prefix + ".odom_topic", name_ + "/odometry/imu_incremental");
  node_->declare_parameter(prefix + ".imu_frame", "imu_link");
  node_->declare_parameter(prefix + ".odom_pose_cov",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
  node_->declare_parameter(prefix + ".odom_twist_cov",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});

  std::string imu_topic;
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".acc_cov", acc_cov_);
  node_->get_parameter(prefix + ".gyr_cov", gyr_cov_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".integration_cov", integration_cov_);
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

  // GTSAM preintegration (for real-time odom prediction only)
  auto p = gtsam::PreintegrationParams::MakeSharedU(gravity_);
  p->accelerometerCovariance =
      Eigen::Vector3d(acc_cov_[0], acc_cov_[1], acc_cov_[2]).asDiagonal();
  p->gyroscopeCovariance =
      Eigen::Vector3d(gyr_cov_[0], gyr_cov_[1], gyr_cov_[2]).asDiagonal();
  p->integrationCovariance =
      Eigen::Vector3d(integration_cov_[0], integration_cov_[1], integration_cov_[2]).asDiagonal();
  preint_params_ = p;

  imu_integrator_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      preint_params_, gtsam::imuBias::ConstantBias());

  // Subscription
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuMotionModel::imuCallback, this, std::placeholders::_1),
      sub_opts);

  // Publisher
  std::string odom_topic;
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  imu_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (pure odom filler)", name_.c_str());
}

void ImuMotionModel::activate() {
  active_ = true;
  imu_odom_pub_->on_activate();

  // Resolve IMU → base_link static TF
  try {
    tf_imu_to_base_msg_ = tf_->lookupTransform(
        base_link_frame_, imu_frame_, tf2::TimePointZero,
        tf2::durationFromSec(kTfLookupTimeout));
    extrinsics_resolved_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] resolved TF %s -> %s",
                name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str());
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] failed TF %s -> %s: %s. Using identity.",
                 name_.c_str(), imu_frame_.c_str(), base_link_frame_.c_str(), e.what());
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

// ---------------------------------------------------------------------------
// Readiness
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
// IMU callback — high-rate processing
// ---------------------------------------------------------------------------
void ImuMotionModel::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!isValidImuMessage(*msg)) return;

  sensor_msgs::msg::Imu imu_converted = imuConverter(*msg);
  updateStationaryDetection(imu_converted);

  // Only produce odom while TRACKING
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return;

  auto current_state = integrateSingleAndPredict(imu_converted);
  gtsam::Pose3 base_pose(current_state.quaternion(), current_state.position());

  // Lock-free write — TransformManager reads this
  setOdomPose(base_pose);

  publishIncrementalOdom(imu_converted, base_pose, current_state);
}

void ImuMotionModel::updateStationaryDetection(
    const sensor_msgs::msg::Imu& imu_converted) {
  if (stationary_buffer_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] first IMU message received", name_.c_str());
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
  double imu_time = rclcpp::Time(imu_converted.header.stamp).seconds();
  double dt = (last_imu_t_ < 0) ? default_imu_dt_ : (imu_time - last_imu_t_);
  last_imu_t_ = imu_time;

  auto [acc, gyr] = extractMeasurement(imu_converted);
  imu_integrator_->integrateMeasurement(acc, gyr, dt);

  return imu_integrator_->predict(odom_reference_state_, odom_reference_bias_);
}

void ImuMotionModel::publishIncrementalOdom(
    const sensor_msgs::msg::Imu& imu_converted,
    const gtsam::Pose3& base_pose,
    const gtsam::NavState& state) {
  if (!imu_odom_pub_->is_activated()) return;

  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = imu_converted.header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_link_frame_;

  auto q = base_pose.rotation().toQuaternion();
  odom.pose.pose.position.x = base_pose.translation().x();
  odom.pose.pose.position.y = base_pose.translation().y();
  odom.pose.pose.position.z = base_pose.translation().z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.velocity().x();
  odom.twist.twist.linear.y = state.velocity().y();
  odom.twist.twist.linear.z = state.velocity().z();
  odom.twist.twist.angular.x = imu_converted.angular_velocity.x;
  odom.twist.twist.angular.y = imu_converted.angular_velocity.y;
  odom.twist.twist.angular.z = imu_converted.angular_velocity.z;

  for (size_t i = 0; i < odom_pose_cov_.size() && i < 6; i++)
    odom.pose.covariance[i * 7] = odom_pose_cov_[i];
  for (size_t i = 0; i < odom_twist_cov_.size() && i < 6; i++)
    odom.twist.covariance[i * 7] = odom_twist_cov_[i];

  imu_odom_pub_->publish(odom);
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
    RCLCPP_ERROR(node_->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
  }
  q_base.normalize();
  imu_out.orientation = tf2::toMsg(q_base);

  return imu_out;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::ImuMotionModel, eidos::MotionModelPlugin)
