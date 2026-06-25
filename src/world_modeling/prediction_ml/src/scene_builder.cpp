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

#include "prediction_ml/scene_builder.hpp"

#include <utility>

namespace prediction_ml
{

SceneBuilder::SceneBuilder(MtrConfig config) : config_(std::move(config)) {}

void SceneBuilder::addFrame(const vision_msgs::msg::Detection3DArray & /*detections*/)
{
  // TODO(Person A): update per-object history keyed by detection id
  // (pose, dims, heading, velocity, type, timestamp, validity).
}

MtrInputTensors SceneBuilder::build(const MtrFrameContext & /*frame*/)
{
  // TODO(Person A): resample history, select target agents up to
  // config_.selected_target_agent_limit, pack obj_trajs* / track_index_to_predict /
  // center_objects_type, convert lanelet context to map_polylines*, fill sidecar.
  MtrInputTensors tensors;
  tensors.valid = false;
  return tensors;
}

}  // namespace prediction_ml
