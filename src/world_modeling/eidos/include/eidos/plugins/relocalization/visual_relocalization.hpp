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

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/plugins/visual/visual_keyframes.hpp"

namespace eidos
{

/**
 * @brief Relocalization from camera images against a prior map that VisualLoopClosureFactor recorded.
 *
 * The prior map's keyframes carry their camera images and place descriptors. An attempt:
 *  1. describes the current camera images and picks the prior keyframes that look most alike;
 *  2. measures the rig's pose relative to each (perception's map_anything server) and needs two
 *     of them to agree on where in the map the vehicle is;
 *  3. measures a second, newer set of images against the best keyframe, which gives the vehicle's
 *     velocity, so the pose handed to eidos can be carried forward to the time it is used.
 * Attempts run on a thread of the plugin; tryRelocalize() returns nullopt until one has succeeded.
 * Every accepted estimate is also published on <name>/estimate at its images' stamp.
 */
class VisualRelocalization : public RelocalizationPlugin
{
public:
  VisualRelocalization() = default;

  /// @brief Join the attempt thread.
  ~VisualRelocalization() override;

  /// @brief Declare parameters, create the camera subscriptions, service clients and publisher.
  void onInitialize() override;

  /// @brief Start keeping images.
  void activate() override;

  /// @brief Stop keeping images and join the attempt thread.
  void deactivate() override;

  /**
   * @brief Start an attempt if none is running, and return a finished one carried forward to timestamp.
   * @param timestamp Current SLAM cycle timestamp (seconds).
   * @return The vehicle's pose in the map at timestamp, or nullopt while no attempt has succeeded.
   */
  std::optional<RelocalizationResult> tryRelocalize(double timestamp) override;

private:
  struct Candidate
  {
    gtsam::Key key;
    int index;  ///< Keyframe index in MapManager
    double similarity;
  };

  struct Estimate
  {
    gtsam::Pose3 pose;  ///< Rig pose in the map at stamp
    double stamp = 0.0;
    gtsam::Vector6 twist = gtsam::Vector6::Zero();  ///< Body twist [rot, trans] per second
    double spread = 0.0;
    int keyframe_index = -1;
  };

  /// @brief One relocalization attempt (attempt thread).
  void attempt();

  /// @brief Prior-map keyframes that look most like the given descriptors, best first.
  std::vector<Candidate> rank(const std::vector<float> & descriptors, std::size_t dim) const;

  /// @brief Rig pose in the map at the frame set's stamp, measured against one prior keyframe.
  std::optional<gtsam::Pose3> locate(const Candidate & candidate, const VisualFrameSet & live, double & spread);

  std::unique_ptr<VisualKeyframes> keyframes_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pub_;
  std::string map_frame_;

  std::thread thread_;
  std::atomic<bool> attempt_running_{false};
  std::mutex result_mtx_;
  std::optional<Estimate> result_;
  bool active_ = false;

  // ---- Parameters ----
  std::string keyframes_from_ = "visual_loop_closure_factor";
  int top_k_ = 3;
  double min_similarity_ = 0.5;
  double agree_trans_ = 1.5;
  double agree_rot_ = 0.1;
  double max_trans_spread_ = 1.0;
  double max_rot_spread_ = 0.05;
  double min_interval_ = 0.2;
  double max_speed_ = 40.0;
  double max_age_ = 3.0;
};

}  // namespace eidos
