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

#ifndef PREDICTION_ML__SCENE_BUILDER_HPP_
#define PREDICTION_ML__SCENE_BUILDER_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "prediction_ml/mtr_types.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

namespace prediction_ml
{

// Maintains per-object history and packs MTR input tensors. (Person A)
class SceneBuilder
{
public:
  explicit SceneBuilder(MtrConfig config);

  // Append the latest tracked detections into per-object history.
  void addFrame(const vision_msgs::msg::Detection3DArray & detections);

  // Pack the current scene into MTR input tensors + sidecar.
  MtrInputTensors build(const MtrFrameContext & frame);

private:
  enum class MtrObjectClass
  {
    Vehicle,
    Pedestrian,
    Cyclist
  };

  struct HistorySample
  {
    std::string detection_id;
    double timestamp{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double dx{0.0};
    double dy{0.0};
    double dz{0.0};
    double heading{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    MtrObjectClass object_class{MtrObjectClass::Vehicle};
    std::string frame_id;
  };

  // A context track ranked by ego-frame distance for the 128-row obj_trajs dimension.
  struct ContextTrack
  {
    std::string detection_id;
    double distance_sq{0.0};
  };

  struct ResampledTrack
  {
    std::string detection_id;
    std::vector<std::optional<HistorySample>> samples;
  };

  struct TargetCandidate
  {
    HistorySample sample;
    double distance_sq{0.0};
    bool in_forward_region{false};
  };

  static std::optional<double> timestampFromHeader(const vision_msgs::msg::Detection3DArray & detections);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static std::optional<MtrObjectClass> parseObjectClass(const vision_msgs::msg::Detection3D & detection);
  static std::optional<HistorySample> sampleFromDetection(
    const vision_msgs::msg::Detection3D & detection, double timestamp, const std::string & frame_id);
  static double clampForwardHalfAngleDeg(double half_angle_deg);

  void addFrameAtTimestamp(const vision_msgs::msg::Detection3DArray & detections, double timestamp);
  void addSample(const HistorySample & sample);
  // Append the ego pose as an SDC track sample; velocity is finite-differenced from the
  // previous ego sample (eventually move to odom).
  void addEgoSample(const geometry_msgs::msg::PoseStamped & ego_pose, double timestamp);
  void pruneHistory(double current_time);
  std::vector<double> desiredSampleTimes(double current_time) const;
  std::optional<HistorySample> nearestSampleForSlot(
    const std::vector<HistorySample> & samples, double desired_time) const;
  std::vector<ResampledTrack> resampleTracks(
    const std::vector<std::string> & track_ids, const std::vector<double> & desired_times) const;
  std::vector<TargetCandidate> selectTargets(
    const vision_msgs::msg::Detection3DArray & detections, double current_time, const geometry_msgs::msg::PoseStamped & ego_pose)
    const;
  // Rank every retained track (incl. ego) by ego-frame 2D distance_sq and return the closest
  // kContextCapacity ids, tie-broken by detection_id ascending.
  std::vector<std::string> contextTrackIds(const geometry_msgs::msg::PoseStamped & ego_pose) const;
  static int findTrackIndex(const std::vector<std::string> & track_ids, const std::string & detection_id);
  // Pack obj_trajs / masks / last_pos / center_* to the fixed [8,128,T,F] contract. Returns false
  // (hard frame failure) if any selected target is absent from the context list.
  bool packObjectTensors(
    const std::vector<TargetCandidate> & targets, const std::vector<std::string> & context_track_ids,
    const std::vector<ResampledTrack> & resampled_tracks, const std::vector<double> & desired_times,
    double current_time, MtrInputTensors & tensors) const;

  MtrConfig config_;
  bool config_valid_{false};
  std::unordered_map<std::string, std::vector<HistorySample>> history_;
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__SCENE_BUILDER_HPP_
