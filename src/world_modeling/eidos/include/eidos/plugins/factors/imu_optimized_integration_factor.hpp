#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief IMU preintegration factor plugin.
 *
 * Runs its own internal ISAM2 optimizer to estimate IMU biases using
 * graph-optimized pose corrections. Publishes high-rate IMU odometry
 * and TF from the IMU callback.
 */
class ImuOptimizedIntegrationFactor : public FactorPlugin {
public:
  ImuOptimizedIntegrationFactor() = default;
  ~ImuOptimizedIntegrationFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
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
  void initializeIsam2(const gtsam::Pose3& graph_pose, double correction_time);
  void resetGraphIfNeeded();
  bool integrateAndOptimize(const gtsam::Pose3& graph_pose, double correction_time);
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

  // ---- Internal ISAM2 helpers ----
  void resetOptimization();
  bool failureDetection(const gtsam::Vector3& vel,
                        const gtsam::imuBias::ConstantBias& bias);

  // ---- Subscription ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_fused_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

  // ---- Internal ISAM2 optimizer (for bias estimation) ----
  gtsam::ISAM2 optimizer_;
  gtsam::NonlinearFactorGraph graph_factors_;
  gtsam::Values graph_values_;

  // ---- Noise models ----
  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise2_;
  gtsam::Vector noise_between_bias_;

  // ---- State for internal optimizer ----
  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_ = gtsam::Vector3::Zero();
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  // State for real-time prediction
  gtsam::NavState prev_state_odom_;
  gtsam::imuBias::ConstantBias prev_bias_odom_;

  // Internal optimizer key counter
  int key_ = 1;
  bool done_first_opt_ = false;
  double last_imu_t_imu_ = -1;
  double last_imu_t_opt_ = -1;
  bool initialized_ = false;
  bool active_ = false;

  // Latest IMU-predicted pose (for processFrame)
  gtsam::Pose3 latest_imu_pose_;
  bool has_imu_pose_ = false;
  std::mutex imu_pose_lock_;

  // ---- Main graph odometry (BetweenFactor) ----
  gtsam::Pose3 last_graph_pose_;
  bool has_last_graph_pose_ = false;
  double odom_rot_noise_;
  double odom_trans_noise_;

  // ---- Parameters (populated from ROS params in onInitialize) ----
  double acc_noise_;
  double gyr_noise_;
  double acc_bias_noise_;
  double gyr_bias_noise_;
  double gravity_;
  double integration_noise_;
  double prior_pose_sigma_;
  double prior_vel_sigma_;
  double prior_bias_sigma_;
  double correction_rot_sigma_;
  double correction_trans_sigma_;
  double correction_degrade_sigma_;
  double isam2_relinearize_threshold_;
  double max_velocity_;
  double max_bias_norm_;
  double default_imu_dt_;
  double quaternion_norm_threshold_;
  int graph_reset_interval_;
  double stationary_acc_threshold_;
  double stationary_gyr_threshold_;
  int stationary_samples_;

  // Cached static TF: imu_frame_ → base_link_frame_ (looked up once in activate())
  geometry_msgs::msg::TransformStamped tf_imu_to_base_msg_;
  bool extrinsics_resolved_ = false;

  // Frame names
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
  std::string imu_frame_ = "imu_link";
  std::string imu_odom_child_frame_ = "odom_imu";
};

}  // namespace eidos
