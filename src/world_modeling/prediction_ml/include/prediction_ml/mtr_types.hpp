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

#ifndef PREDICTION_ML__MTR_TYPES_HPP_
#define PREDICTION_ML__MTR_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/prediction.hpp"

namespace prediction_ml
{

// MTR execution mode. Disabled and Null both yield fallback-only output.
enum class MtrMode
{
  Disabled,
  Null,
  TensorRt
};

struct MtrConfig
{
  MtrMode mode{MtrMode::Disabled};
  std::string engine_path;
  std::string metadata_path;
  double cache_ttl_s{0.5};
  int selected_target_agent_limit{8};
  int history_steps{11};
  double history_rate_hz{10.0};
};

// One engine binding (name + dtype + shape) used for contract validation.
struct MtrTensorSpec
{
  std::string name;
  std::string dtype;
  std::vector<int64_t> shape;
};

struct MtrModelContract
{
  std::vector<MtrTensorSpec> inputs;
  std::vector<MtrTensorSpec> outputs;
};

// Per-tick bundle handed to the scene builder.
struct MtrFrameContext
{
  vision_msgs::msg::Detection3DArray detections;
  geometry_msgs::msg::PoseStamped ego_pose;
  lanelet_msgs::msg::LaneletAhead lanelet_ahead;
  double timestamp{0.0};
  bool has_ego{false};
  bool has_map{false};
};

// Maps a target slot back to its source detection and map-frame pose.
struct MtrTargetSidecar
{
  std::string detection_id;
  int track_index{0};
  double center_x{0.0};
  double center_y{0.0};
  double center_heading{0.0};
};

struct MtrBatchSidecar
{
  std::vector<MtrTargetSidecar> targets;
  std::string frame_id;
};

// Flat MTR input buffers (row-major) plus shapes and the sidecar.
struct MtrInputTensors
{
  std::vector<float> obj_trajs;
  std::vector<uint8_t> obj_trajs_mask;
  std::vector<float> obj_trajs_last_pos;
  std::vector<int32_t> track_index_to_predict;
  std::vector<std::string> center_objects_type;
  std::vector<float> map_polylines;
  std::vector<uint8_t> map_polylines_mask;
  std::vector<float> map_polylines_center;
  std::vector<int64_t> obj_trajs_shape;
  std::vector<int64_t> map_polylines_shape;
  MtrBatchSidecar sidecar;
  bool valid{false};
};

// Raw MTR outputs: pred_scores [num_target, K], pred_trajs [num_target, K, T, C].
struct MtrOutputTensors
{
  std::vector<float> pred_scores;
  std::vector<float> pred_trajs;
  std::vector<int64_t> scores_shape;
  std::vector<int64_t> trajs_shape;
  bool valid{false};
};

// Converted per-object result (map frame, world_model_msgs/Prediction).
struct MtrObjectPrediction
{
  std::string detection_id;
  std::vector<world_model_msgs::msg::Prediction> predictions;
};

struct MtrInferenceResult
{
  std::vector<MtrObjectPrediction> objects;
  bool ok{false};
  std::string error;
};

// Parse "disabled" | "null" | "tensorrt" (case-insensitive) into MtrMode.
MtrMode parseMtrMode(const std::string & mode_str);

// Load expected/actual engine contract from a JSON/YAML metadata file.
MtrModelContract loadMtrModelContract(const std::string & metadata_path);

// Returns true if actual matches expected; fills error otherwise.
bool validateMtrModelContract(
  const MtrModelContract & expected, const MtrModelContract & actual, std::string & error);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_TYPES_HPP_
