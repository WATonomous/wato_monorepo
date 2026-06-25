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

#ifndef PREDICTION_ML__OUTPUT_CONVERTER_HPP_
#define PREDICTION_ML__OUTPUT_CONVERTER_HPP_

#include <string>

#include "prediction_ml/mtr_types.hpp"

namespace prediction_ml
{

// Convert raw MTR outputs (target frame) into map-frame world_model predictions.
MtrInferenceResult convertMtrOutput(
  const MtrOutputTensors & out,
  const MtrBatchSidecar & sidecar,
  const std::string & frame_id,
  double horizon_s,
  double time_step_s);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__OUTPUT_CONVERTER_HPP_
