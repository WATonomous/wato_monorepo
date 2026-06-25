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

#ifndef PREDICTION_ML__MTR_RUNTIME_HPP_
#define PREDICTION_ML__MTR_RUNTIME_HPP_

#include <memory>
#include <string>
#include <vector>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_types.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace prediction_ml
{

// Owns the inference engine, runs it async (latest-only), caches per-object
// predictions with a TTL, and merges them onto the fallback. (Person C)
class MtrRuntime
{
public:
  explicit MtrRuntime(MtrConfig config);
  ~MtrRuntime();

  // Hand the latest packed tensors to the (future) async worker.
  void submitFrame(const MtrInputTensors & input, const std::string & frame_id, double horizon_s, double time_step_s);

  // Start from fallback; replace only objects with fresh valid cached MTR predictions.
  std::vector<world_model_msgs::msg::WorldObject> selectOutput(
    const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s);

  bool ready() const;
  std::string lastError() const;

private:
  MtrConfig config_;
  std::unique_ptr<IMtrInferenceEngine> engine_;
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_RUNTIME_HPP_
