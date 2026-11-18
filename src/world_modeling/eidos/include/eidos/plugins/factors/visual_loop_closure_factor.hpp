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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <eidos_msgs/msg/loop_closure_measurement.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/plugins/visual/visual_keyframes.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Loop closure from camera images: a relative pose between two keyframes measured by a
 *        multi-view model (perception's map_anything server), added as a BetweenFactor<Pose3>.
 *
 * For every keyframe the plugin keeps the camera images nearest to it (stored in the map, so a
 * saved map can later be relocalized against by VisualRelocalization) and, if place recognition is
 * on, their place descriptors. Candidate pairs come from the keyframe poses (like
 * EuclideanDistanceLoopClosureFactor), from place recognition (which also finds loops the drifted
 * poses no longer put close together), or both. The pose service is slow, so measuring runs on a
 * thread of the plugin and a finished constraint is delivered on the next latchFactor() call.
 *
 * Every measurement is published on <name>/measurement, whether or not it becomes a factor
 * (add_factors: false = shadow mode). With shadow_odometry the plugin also measures every
 * consecutive keyframe pair, only to publish it: a baseline to score against the graph's odometry.
 */
class VisualLoopClosureFactor : public FactorPlugin
{
public:
  VisualLoopClosureFactor() = default;

  /// @brief Stop the worker threads.
  ~VisualLoopClosureFactor() override;

  /// @brief Declare parameters, create the camera subscriptions, service clients and publisher.
  void onInitialize() override;

  /// @brief Start keeping images and measuring.
  void activate() override;

  /// @brief Stop the worker threads and drop pending work.
  void deactivate() override;

  /**
   * @brief Deliver finished loop closures and queue the new keyframe for recording.
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state (seconds).
   * @return BetweenFactor<Pose3> for every constraint finished since the last call (possibly none).
   */
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  struct Pending
  {
    gtsam::Key key;
    double stamp;
    std::chrono::steady_clock::time_point queued;
  };

  struct Constraint
  {
    gtsam::Key from;
    gtsam::Key to;
    gtsam::Pose3 measured;  ///< T_from^-1 * T_to
    gtsam::SharedNoiseModel noise;
  };

  enum class RecordStatus
  {
    kWait,  ///< Images or the keyframe's pose have not arrived yet
    kRecorded,
    kSkipped  ///< No images near the keyframe
  };

  /// @brief Thread: store each new keyframe's images as soon as they are known.
  void recordLoop();

  /// @brief Thread: describe keyframes, find candidates and measure them.
  void measureLoop();

  RecordStatus record(const Pending & pending);

  /// @brief Describe the keyframe, then measure its consecutive pair and loop candidates.
  void process(gtsam::Key key);

  /// @brief Nearest earlier keyframe by position, as EuclideanDistanceLoopClosureFactor picks it.
  std::optional<gtsam::Key> distanceCandidate(gtsam::Key key) const;

  /// @brief Earlier keyframe that looks most alike, if alike enough.
  std::optional<gtsam::Key> placeCandidate(gtsam::Key key, double & similarity) const;

  /// @brief Measure one pair, publish it, and queue a constraint if it passes the checks.
  void measure(gtsam::Key from, gtsam::Key to, const std::string & candidate_source, double similarity);

  gtsam::SharedNoiseModel noiseFor(double distance) const;

  void logCounters();

  // ---- ROS ----
  std::unique_ptr<VisualKeyframes> keyframes_;
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::LoopClosureMeasurement>::SharedPtr measurement_pub_;
  std::string base_link_frame_;

  // ---- Threads ----
  std::thread record_thread_;
  std::thread measure_thread_;
  std::atomic<bool> running_{false};
  std::mutex queue_mtx_;  ///< Guards pending_, recorded_ and results_
  std::condition_variable queue_cv_;
  std::deque<Pending> pending_;  ///< Keyframes whose images are not stored yet
  std::deque<gtsam::Key> recorded_;  ///< Keyframes waiting to be described and measured
  std::vector<Constraint> results_;  ///< Finished constraints, delivered by latchFactor()

  // ---- Measure-thread state ----
  std::map<gtsam::Key, std::shared_ptr<const std::vector<float>>> descriptors_;
  std::size_t descriptor_dim_ = 0;
  std::optional<gtsam::Key> last_processed_;

  // ---- Parameters ----
  bool add_factors_ = true;
  bool shadow_odometry_ = false;
  std::string candidate_mode_ = "distance";
  double search_radius_ = 20.0;
  double search_time_diff_ = 40.0;
  double max_heading_diff_ = 1.05;
  double min_similarity_ = 0.6;
  double max_frame_wait_ = 0.5;
  int max_queue_ = 30;
  int min_cameras_ = 1;
  double max_trans_spread_ = 1.0;
  double max_rot_spread_ = 0.05;
  double gate_trans_ = 5.0;
  double gate_rot_ = 0.3;
  bool gate_place_recognition_ = false;
  double rot_sigma_ = 0.02;
  double trans_sigma_ = 0.3;
  double trans_sigma_per_m_ = 0.08;
  std::string robust_kernel_ = "huber";
  double robust_k_ = 1.345;
  std::string frames_key_;
  std::string descriptors_key_;
  bool active_ = false;

  // ---- Counters ----
  struct Counters
  {
    std::atomic<std::size_t> recorded{0};
    std::atomic<std::size_t> no_images{0};
    std::atomic<std::size_t> dropped{0};
    std::atomic<std::size_t> measured{0};
    std::atomic<std::size_t> failed{0};
    std::atomic<std::size_t> rejected{0};
    std::atomic<std::size_t> accepted{0};
  } counters_;

  std::chrono::steady_clock::time_point last_log_{};
};

}  // namespace eidos
