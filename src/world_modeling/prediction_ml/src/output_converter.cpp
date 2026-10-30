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

#include "prediction_ml/output_converter.hpp"

#include <string>

namespace prediction_ml
{

MtrInferenceResult convertMtrOutput(
  const MtrOutputTensors & out,
  const MtrBatchSidecar & /*sidecar*/,
  const std::string & /*frame_id*/,
  double /*horizon_s*/,
  double /*time_step_s*/)
{
  MtrInferenceResult result;
  if (!out.valid) {
    result.ok = false;
    result.error = "mtr output invalid";
    return result;
  }
  // TODO(Person C): validate pred_scores/pred_trajs, rotate/translate each mode from
  // target frame to map frame using sidecar, infer yaw from consecutive points, and
  // emit world_model_msgs/Prediction with conf = pred_score.
  result.ok = true;
  return result;
}

}  // namespace prediction_ml
