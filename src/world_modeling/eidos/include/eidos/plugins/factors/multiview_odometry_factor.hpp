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

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <eidos_msgs/msg/loop_closure_measurement.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/plugins/visual/lidar_depth.hpp"
#include "eidos/plugins/visual/visual_keyframes.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Keyframes and odometry from camera images alone, through a multi-view model service.
 *
 * A state-creating plugin that needs no lidar. A worker thread picks keyframes at camera frame
 * times: the first once tracking begins, then whenever the GPS shows keyframe_distance of travel
 * since the previous one, or keyframe_max_interval has passed. The pose service (perception's
 * map_anything server) measures the rig's motion from the previous keyframe's images to the new
 * one's, and returns the new keyframe's dense coloured points. produceFactor() then creates the
 * state, at the images' time, with a BetweenFactor<Pose3> from the previous one and the measured
 * motion as its initial value. The points are stored under `<name>/cloud` for
 * KeyframeMapVisualization.
 *
 * A measurement that fails or is rejected (too few cameras, cameras disagreeing) is not used: with
 * create_without_measurement the state is still created and other plugins (e.g. ImuFactor) must
 * constrain it; otherwise the keyframe is dropped and the next one is measured from the previous.
 * With add_factors false (shadow mode) every keyframe becomes a state and no visual factor is
 * added, so other plugins can be compared on the same keyframes.
 *
 * Every measurement is published on `<name>/measurement` (candidate_source "consecutive"), with the
 * graph's relative pose of the same two states once optimized.
 *
 * With bias.enabled, every state also gets a bias vector C(i) = gtsam::Symbol('c', i): translation
 * scale, yaw per metre and rotation scale of the measurements (BiasedRelativePoseFactor). The first
 * has a zero-mean prior, each later one a random walk from the previous. Other plugins (IMU, GPS)
 * make it observable; its estimate is published on `<name>/bias`.
 */
class MultiViewOdometryFactor : public FactorPlugin
{
public:
  MultiViewOdometryFactor() = default;
  ~MultiViewOdometryFactor() override;

  /// @brief Declare parameters, subscribe to the cameras and the GPS, create the publisher.
  void onInitialize() override;

  /// @brief Start the keyframe worker.
  void activate() override;

  /// @brief Stop the worker and drop pending keyframes.
  void deactivate() override;

  /// @brief Create the next measured keyframe as a state, if one is ready.
  StampedFactorResult produceFactor(gtsam::Key key, double timestamp) override;

  /// @brief Start choosing keyframes.
  void onTrackingBegin(const gtsam::Pose3 & pose) override;

  /// @brief Remember optimized poses and publish the measurements of optimized states.
  void onOptimizationComplete(const gtsam::Values & values, bool graph_corrected) override;

  /// @brief Bias key of a state key (same index, character 'c').
  static gtsam::Key biasKey(gtsam::Key state);

private:
  /// A keyframe that has been measured and waits to become a state.
  struct Keyframe
  {
    VisualFrameSet frames;
    std::optional<VisualPoseEstimate> estimate;  ///< motion from the previous keyframe
    std::string reject;  ///< why estimate is not used (empty if it is)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  };

  /// A measurement waiting for both of its states to be optimized, to be published.
  struct Pending
  {
    gtsam::Key from;
    gtsam::Key to;
    double from_stamp;
    double to_stamp;
    Keyframe keyframe;
    bool added;
  };

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /// @brief Choose, measure and queue keyframes until stopped.
  void workLoop();

  /**
   * @brief Time of the next keyframe after the previous one at `after`.
   *
   * The first GPS fix at least keyframe_distance from where the vehicle was at `after`, or
   * keyframe_max_interval after it, whichever comes first, and not before `not_before`.
   * nullopt while neither is known yet.
   */
  std::optional<double> nextKeyframeTime(double after, double not_before) const;

  /// @brief Whether the estimate passes the quality checks; sets reason otherwise.
  bool check(const VisualPoseEstimate & estimate, std::string & reason) const;

  gtsam::SharedNoiseModel noiseFor(double distance) const;

  void publish(const Pending & pending, const gtsam::Values & values);

  void logCounters();

  // ---- Inputs ----
  std::unique_ptr<VisualKeyframes> keyframes_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::LoopClosureMeasurement>::SharedPtr measurement_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_pub_;

  // ---- GPS travel (guarded by gps_mtx_) ----
  mutable std::mutex gps_mtx_;
  std::deque<std::pair<double, Eigen::Vector2d>> gps_;  ///< (stamp, UTM easting/northing)

  // ---- Worker ----
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> tracking_{false};
  std::atomic<bool> restart_{false};  ///< tracking began again: the worker forgets its anchor
  std::optional<VisualFrameSet> anchor_;  ///< worker thread only: the previous keyframe's images
  std::vector<sensor_msgs::msg::PointCloud2> anchor_depth_;  ///< worker thread only: and their lidar depth
  std::unique_ptr<LidarDepth> depth_;  ///< null unless depth.enabled

  // ---- Measured keyframes waiting for produceFactor (guarded by ready_mtx_) ----
  std::mutex ready_mtx_;
  std::deque<Keyframe> ready_;

  // ---- SLAM thread ----
  std::optional<gtsam::Key> prev_key_;
  double prev_stamp_ = 0.0;
  std::optional<gtsam::Pose3> prev_pose_;  ///< optimized pose of prev_key_, once known
  gtsam::Vector3 bias_ = gtsam::Vector3::Zero();  ///< newest bias estimate (bias.enabled)
  std::deque<Pending> pending_;

  // ---- Counters ----
  struct Counters
  {
    std::atomic<std::size_t> keyframes{0}, measured{0}, failed{0}, rejected{0}, dropped{0}, no_images{0};
    std::atomic<std::size_t> depth_points{0}, depth_sets{0}, not_deskewed{0};
  } counters_;
  std::chrono::steady_clock::time_point last_log_{};

  // ---- Parameters ----
  std::string cloud_key_;
  std::string base_link_frame_;
  double time_offset_ = 0.0;  ///< s: an image was taken at its stamp + time_offset
  double keyframe_distance_ = 5.0;
  double keyframe_max_interval_ = 1.0;
  bool add_factors_ = true;
  bool create_without_measurement_ = false;
  VisualCloudRequest cloud_request_;
  int cloud_every_n_ = 1;
  int min_cameras_ = 2;
  double max_trans_spread_ = 1.0;
  double max_rot_spread_ = 0.05;
  double rot_sigma_ = 0.008;
  double trans_sigma_ = 0.05;
  double trans_sigma_per_m_ = 0.07;
  bool bias_enabled_ = false;
  std::vector<double> bias_prior_sigma_{0.1, 0.002, 0.1};
  std::vector<double> bias_random_walk_{0.002, 2e-5, 0.002};
  std::string robust_kernel_ = "huber";
  double robust_k_ = 1.345;
  std::size_t max_ready_ = 30;
};

}  // namespace eidos
