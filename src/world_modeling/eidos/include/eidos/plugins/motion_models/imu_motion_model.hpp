#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

#include "eidos/plugins/base_motion_model_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief IMU-based motion model plugin.
 *
 * Replaces the former ImuIntegrationFactor. Subscribes to IMU data,
 * buffers measurements, and generates ImuFactor constraints between
 * consecutive states via generateMotionModel().
 *
 * Also provides high-rate IMU odometry and the forward-propagated pose
 * for displacement checks via getCurrentPose().
 */
class ImuMotionModel : public MotionModelPlugin {
public:
  ImuMotionModel() = default;
  ~ImuMotionModel() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  void generateMotionModel(
      gtsam::Key key_begin, double t_begin,
      gtsam::Key key_end, double t_end,
      gtsam::NonlinearFactorGraph& factors,
      gtsam::Values& values) override;

  void onStateZero(
      gtsam::Key key, double timestamp,
      const gtsam::Pose3& initial_pose,
      gtsam::NonlinearFactorGraph& factors,
      gtsam::Values& values) override;

  void onOptimizationComplete(
      const gtsam::Values& optimized_values,
      bool loop_closure_detected) override;

  bool isReady() const override;
  std::string getReadyStatus() const override;

  std::optional<gtsam::Pose3> getCurrentPose() const override;
  std::optional<gtsam::Rot3> getInitialOrientation() const override;

private:
  // ---- Callbacks ----
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ---- Static utilities ----
  static std::pair<gtsam::Vector3, gtsam::Vector3>
  extractMeasurement(const sensor_msgs::msg::Imu& msg);
  static bool isValidImuMessage(const sensor_msgs::msg::Imu& msg);

  // ---- Integration helpers ----
  void drainAndIntegrate(
      gtsam::PreintegratedImuMeasurements& integrator,
      double t_begin, double t_end);

  // ---- imuCallback sub-methods ----
  void updateStationaryDetection(const sensor_msgs::msg::Imu& imu_converted);
  gtsam::NavState integrateSingleAndPredict(const sensor_msgs::msg::Imu& imu_converted);
  void storeImuPose(const gtsam::Pose3& base_pose);
  void publishIncrementalOdom(const sensor_msgs::msg::Imu& imu_converted,
                              const gtsam::Pose3& base_pose,
                              const gtsam::NavState& state);
  void publishFusedOdom(const sensor_msgs::msg::Imu& imu_converted);

  // ---- IMU extrinsics ----
  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in);

  // ---- Subscription ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_fused_pub_;

  // ---- Transform fusion state ----
  Eigen::Isometry3d graph_odom_affine_ = Eigen::Isometry3d::Identity();
  double graph_odom_time_ = -1;
  std::deque<nav_msgs::msg::Odometry> imu_odom_queue_;

  // ---- IMU buffer (single queue for optimization) ----
  std::deque<sensor_msgs::msg::Imu> imu_queue_;
  // Separate queue for real-time prediction
  std::deque<sensor_msgs::msg::Imu> imu_queue_imu_;
  mutable std::mutex imu_lock_;

  // ---- Stationary detection ----
  std::deque<sensor_msgs::msg::Imu> stationary_buffer_;
  std::atomic<double> stationary_acc_rms_{0.0};
  std::atomic<double> stationary_gyr_rms_{0.0};
  std::atomic<int> stationary_count_{0};

  // ---- GTSAM preintegration ----
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_imu_;

  // ---- Noise models ----
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::Vector noise_between_bias_;

  // ---- State ----
  gtsam::Vector3 prev_vel_ = gtsam::Vector3::Zero();
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  // State for real-time prediction
  gtsam::NavState prev_state_odom_;
  gtsam::imuBias::ConstantBias prev_bias_odom_;

  // Track the last key/timestamp we generated a motion model to
  gtsam::Key last_generated_key_ = 0;
  double last_generated_time_ = -1;

  double last_imu_t_imu_ = -1;
  bool initialized_ = false;
  bool active_ = false;

  // Latest IMU-predicted pose (for getCurrentPose)
  gtsam::Pose3 latest_imu_pose_;
  bool has_imu_pose_ = false;
  mutable std::mutex imu_pose_lock_;

  // ---- Parameters ----
  std::vector<double> acc_cov_;
  std::vector<double> gyr_cov_;
  std::vector<double> bias_walk_cov_;
  std::vector<double> integration_cov_;
  double gravity_;
  std::vector<double> prior_vel_cov_;
  std::vector<double> prior_bias_cov_;
  double default_imu_dt_;
  double quaternion_norm_threshold_;
  double stationary_acc_threshold_;
  double stationary_gyr_threshold_;
  int stationary_samples_;

  // Published odometry covariance diagonals [x, y, z, roll, pitch, yaw]
  std::vector<double> odom_pose_cov_ = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  std::vector<double> odom_twist_cov_ = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

  // Cached static TF
  geometry_msgs::msg::TransformStamped tf_imu_to_base_msg_;
  bool extrinsics_resolved_ = false;

  // Frame names
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
  std::string imu_frame_ = "imu_link";
};

}  // namespace eidos
