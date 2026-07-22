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

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "prediction_ml/output_converter.hpp"

namespace prediction_ml
{

MtrRuntime::MtrRuntime(MtrConfig config)
: MtrRuntime(config, createMtrInferenceEngine(config))
{}

MtrRuntime::MtrRuntime(MtrConfig config, std::unique_ptr<IMtrInferenceEngine> engine)
: config_(std::move(config))
, engine_(std::move(engine))
{
  if (engine_) {
    engine_ready_ = engine_->ready();
    engine_error_ = engine_->lastError();
  } else {
    engine_error_ = "no engine";
  }

  if (engine_ready_) {
    worker_ = std::thread(&MtrRuntime::workerLoop, this);
  }
}

MtrRuntime::~MtrRuntime()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    pending_frame_.reset();
  }
  worker_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MtrRuntime::submitFrame(
  MtrInputTensors input,
  const std::string & frame_id,
  const double source_time_s,
  const double horizon_s,
  const double time_step_s)
{
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t sequence = ++latest_sequence_;

    // Advancing the sequence even for an unusable frame prevents an older
    // in-flight result from becoming current after scene construction failed.
    if (
      engine_ready_ && input.valid && !frame_id.empty() && std::isfinite(source_time_s) && std::isfinite(horizon_s) &&
      horizon_s > 0.0 && std::isfinite(time_step_s) && time_step_s > 0.0)
    {
      pending_frame_ = PendingFrame{std::move(input), frame_id, source_time_s, horizon_s, time_step_s, sequence};
      queued = true;
    } else {
      pending_frame_.reset();
    }
  }

  if (queued) {
    worker_cv_.notify_one();
  }
}

std::vector<world_model_msgs::msg::WorldObject> MtrRuntime::selectOutput(
  const std::vector<world_model_msgs::msg::WorldObject> & fallback, const double now_s)
{
  auto output = fallback;
  if (!std::isfinite(now_s) || !std::isfinite(config_.cache_ttl_s) || config_.cache_ttl_s < 0.0) {
    return output;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (auto it = cache_.begin(); it != cache_.end();) {
    const double age_s = now_s - it->second.source_time_s;
    if (!std::isfinite(age_s) || age_s < 0.0 || age_s > config_.cache_ttl_s) {
      it = cache_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto & object : output) {
    if (object.detection.id.empty()) {
      continue;
    }
    const auto cached = cache_.find(object.detection.id);
    if (cached != cache_.end() && !cached->second.predictions.empty()) {
      object.predictions = cached->second.predictions;
    }
  }
  return output;
}

bool MtrRuntime::ready() const
{
  return engine_ready_;
}

std::string MtrRuntime::lastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return worker_error_.empty() ? engine_error_ : worker_error_;
}

void MtrRuntime::workerLoop()
{
  while (true) {
    PendingFrame frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      worker_cv_.wait(lock, [this]() { return stop_requested_ || pending_frame_.has_value(); });
      if (stop_requested_) {
        return;
      }
      frame = std::move(*pending_frame_);
      pending_frame_.reset();
    }

    MtrInferenceResult converted;
    try {
      const MtrOutputTensors output = engine_->infer(frame.input);
      converted = convertMtrOutput(
        output, frame.input.sidecar, frame.frame_id, frame.source_time_s, frame.horizon_s, frame.time_step_s);
    } catch (const std::exception & error) {
      converted.ok = false;
      converted.error = std::string("mtr inference threw: ") + error.what();
    } catch (...) {
      converted.ok = false;
      converted.error = "mtr inference threw an unknown exception";
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_) {
      return;
    }
    if (frame.sequence != latest_sequence_) {
      continue;
    }
    if (!converted.ok) {
      worker_error_ = converted.error.empty() ? "mtr output conversion failed" : converted.error;
      continue;
    }

    for (auto & object : converted.objects) {
      if (object.detection_id.empty() || object.predictions.empty()) {
        continue;
      }
      cache_[object.detection_id] = CachedPrediction{std::move(object.predictions), frame.source_time_s};
    }
    worker_error_.clear();
  }
}

}  // namespace prediction_ml
