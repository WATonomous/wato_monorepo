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

#include "prediction_ml/mtr_runtime.hpp"

#include <string>
#include <utility>
#include <vector>

namespace prediction_ml
{

MtrRuntime::MtrRuntime(MtrConfig config)
: config_(std::move(config))
{
  if (config_.mode == MtrMode::TensorRt) {
    engine_ = createTensorRtMtrInferenceEngine(config_);
  } else {
    engine_ = createNullMtrInferenceEngine(config_);
  }
}

MtrRuntime::~MtrRuntime() = default;

void MtrRuntime::submitFrame(
  const MtrInputTensors & /*input*/, const std::string & /*frame_id*/, double /*horizon_s*/, double /*time_step_s*/)
{
  // TODO(Person C): hand latest-only frame to a background worker; on completion,
  // run convertMtrOutput and store results in a per-object TTL cache.
}

std::vector<world_model_msgs::msg::WorldObject> MtrRuntime::selectOutput(
  const std::vector<world_model_msgs::msg::WorldObject> & fallback, double /*now_s*/)
{
  // TODO(Person C): replace fallback entries whose detection id has a fresh
  // (age <= config_.cache_ttl_s) valid cached MTR prediction.
  return fallback;
}

bool MtrRuntime::ready() const
{
  return engine_ && engine_->ready();
}

std::string MtrRuntime::lastError() const
{
  return engine_ ? engine_->lastError() : "no engine";
}

}  // namespace prediction_ml
