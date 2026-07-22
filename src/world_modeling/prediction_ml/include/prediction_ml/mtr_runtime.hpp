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

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
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
  MtrRuntime(MtrConfig config, std::unique_ptr<IMtrInferenceEngine> engine);
  ~MtrRuntime();

  MtrRuntime(const MtrRuntime &) = delete;
  MtrRuntime & operator=(const MtrRuntime &) = delete;

  // Hand packed tensors to the async worker. A queued frame is replaced by the
  // newest submission, and an in-flight result is discarded if a newer frame arrives.
  void submitFrame(
    MtrInputTensors input, const std::string & frame_id, double source_time_s, double horizon_s, double time_step_s);

  // Start from fallback; replace only objects with fresh valid cached MTR predictions.
  std::vector<world_model_msgs::msg::WorldObject> selectOutput(
    const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s);

  bool ready() const;
  std::string lastError() const;

private:
  struct PendingFrame
  {
    MtrInputTensors input;
    std::string frame_id;
    double source_time_s{0.0};
    double horizon_s{0.0};
    double time_step_s{0.0};
    std::uint64_t sequence{0};
  };

  struct CachedPrediction
  {
    std::vector<world_model_msgs::msg::Prediction> predictions;
    double source_time_s{0.0};
  };

  void workerLoop();

  MtrConfig config_;
  std::unique_ptr<IMtrInferenceEngine> engine_;
  bool engine_ready_{false};
  std::string engine_error_;

  mutable std::mutex mutex_;
  std::condition_variable worker_cv_;
  std::optional<PendingFrame> pending_frame_;
  std::unordered_map<std::string, CachedPrediction> cache_;
  std::string worker_error_;
  std::thread worker_;
  std::uint64_t latest_sequence_{0};
  bool stop_requested_{false};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_RUNTIME_HPP_
