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

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief IMU preintegration factor plugin.
 *
 * Provides gtsam::ImuFactor directly to the main SLAM graph, which jointly
 * optimizes poses (X), velocities (V), and biases (B). High-rate IMU odometry
 * (forward propagation) is kept for inter-keyframe pose prediction and TF
 * publishing.
 */
class ImuIntegrationFactor : public FactorPlugin {
public:
  ImuIntegrationFactor() = default;
  ~ImuIntegrationFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  FactorResult getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;
  bool isReady() const override;
  std::string getReadyStatus() const override;

private:
  // ---- Callbacks ----
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ---- Static utilities ----
  static std::pair<gtsam::Vector3, gtsam::Vector3>
  extractMeasurement(const sensor_msgs::msg::Imu& msg);
  static bool isValidImuMessage(const sensor_msgs::msg::Imu& msg);

  // ---- Integration helpers (caller must hold imu_lock_) ----
  void drainAndIntegrate(
      std::deque<sensor_msgs::msg::Imu>& queue,
      gtsam::PreintegratedImuMeasurements& integrator,
      double cutoff_time, double& last_time);
  void reintegrateQueue(
      const std::deque<sensor_msgs::msg::Imu>& queue,
      gtsam::PreintegratedImuMeasurements& integrator,
      double last_time);

  // ---- onOptimizationComplete sub-methods (caller must hold imu_lock_) ----
  double getCorrectionTime() const;
  void storeGraphCorrection(const gtsam::Pose3& graph_pose, double correction_time);
  void repropagateBias();

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

  // ---- Failure detection ----
  bool failureDetection(const gtsam::Vector3& vel,
                        const gtsam::imuBias::ConstantBias& bias);

  // ---- Subscription ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_fused_pub_;
  // ---- Transform fusion state ----
  Eigen::Isometry3d graph_odom_affine_ = Eigen::Isometry3d::Identity();
  double graph_odom_time_ = -1;
  std::deque<nav_msgs::msg::Odometry> imu_odom_queue_;

  // ---- IMU buffers (two queues like LIO-SAM) ----
  std::deque<sensor_msgs::msg::Imu> imu_queue_opt_;
  std::deque<sensor_msgs::msg::Imu> imu_queue_imu_;
  mutable std::mutex imu_lock_;

  // ---- Stationary detection ----
  //  The callback maintains a sliding window and publishes computed RMS values.
  //  isReady() / getReadyStatus() only read atomics — no lock needed.
  std::deque<sensor_msgs::msg::Imu> stationary_buffer_;  // callback-only, no lock needed
  std::atomic<double> stationary_acc_rms_{0.0};
  std::atomic<double> stationary_gyr_rms_{0.0};
  std::atomic<int> stationary_count_{0};

  // ---- GTSAM preintegration (two integrators like LIO-SAM) ----
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_opt_;
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

  double last_imu_t_imu_ = -1;
  double last_imu_t_opt_ = -1;
  bool initialized_ = false;
  bool active_ = false;

  // Latest IMU-predicted pose (for processFrame)
  gtsam::Pose3 latest_imu_pose_;
  bool has_imu_pose_ = false;
  std::mutex imu_pose_lock_;

  // ---- Parameters (populated from ROS params in onInitialize) ----
  double acc_noise_;
  double gyr_noise_;
  double acc_bias_noise_;
  double gyr_bias_noise_;
  double gravity_;
  double integration_noise_;
  double prior_vel_sigma_;
  double prior_bias_sigma_;
  double max_velocity_;
  double max_bias_norm_;
  double default_imu_dt_;
  double quaternion_norm_threshold_;
  double stationary_acc_threshold_;
  double stationary_gyr_threshold_;
  int stationary_samples_;

  // Cached static TF: imu_frame_ -> base_link_frame_ (looked up once in activate())
  geometry_msgs::msg::TransformStamped tf_imu_to_base_msg_;
  bool extrinsics_resolved_ = false;

  // Frame names
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
  std::string imu_frame_ = "imu_link";
};

}  // namespace eidos
