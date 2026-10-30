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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_runtime.hpp"
#include "rcutils/logging.h"

namespace
{

class ImmediateEngine : public prediction_ml::IMtrInferenceEngine
{
public:
  explicit ImmediateEngine(prediction_ml::MtrOutputTensors output)
  : output_(std::move(output))
  {}

  bool ready() const override
  {
    return true;
  }

  std::string lastError() const override
  {
    return {};
  }

  prediction_ml::MtrOutputTensors infer(const prediction_ml::MtrInputTensors &) override
  {
    return output_;
  }

private:
  prediction_ml::MtrOutputTensors output_;
};

prediction_ml::MtrInputTensors makeInput(const std::string & detection_id)
{
  prediction_ml::MtrInputTensors input;
  input.valid = true;
  input.sidecar.frame_id = "map";
  prediction_ml::MtrTargetSidecar target;
  target.detection_id = detection_id;
  input.sidecar.targets.push_back(target);
  return input;
}

prediction_ml::MtrOutputTensors makeOutput()
{
  prediction_ml::MtrOutputTensors output;
  output.valid = true;
  output.scores_shape = {1, 1};
  output.trajs_shape = {1, 1, 2, 2};
  output.pred_scores = {0.8F};
  output.pred_trajs = {1.0F, 0.0F, 2.0F, 0.0F};
  return output;
}

world_model_msgs::msg::WorldObject makeFallback(const std::string & detection_id)
{
  world_model_msgs::msg::WorldObject object;
  object.detection.id = detection_id;
  world_model_msgs::msg::Prediction prediction;
  prediction.conf = 1.0;
  object.predictions.push_back(prediction);
  return object;
}

}  // namespace

TEST(MtrRuntime, DisabledReturnsFallbackUnchanged)
{
  prediction_ml::MtrConfig cfg;  // Disabled
  prediction_ml::MtrRuntime runtime(cfg);

  std::vector<world_model_msgs::msg::WorldObject> fallback(3);
  fallback[0].detection.id = "a";
  fallback[1].detection.id = "b";
  fallback[2].detection.id = "c";

  auto out = runtime.selectOutput(fallback, 0.0);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].detection.id, "a");
  EXPECT_FALSE(runtime.ready());
}

TEST(MtrRuntime, FreshResultReplacesMatchingFallbackAndExpiresBySourceTime)
{
  prediction_ml::MtrConfig cfg;
  cfg.cache_ttl_s = 0.5;
  cfg.model_time_step_s = 0.5;
  auto engine = std::make_unique<ImmediateEngine>(makeOutput());
  prediction_ml::MtrRuntime runtime(cfg, std::move(engine));

  std::vector<world_model_msgs::msg::WorldObject> fallback;
  fallback.push_back(makeFallback("target"));
  fallback.push_back(makeFallback("other"));
  runtime.submitFrame(makeInput("target"), "map", 100.0, 1.0, 0.5);

  bool replaced = false;
  std::vector<world_model_msgs::msg::WorldObject> fresh;
  for (int attempt = 0; attempt < 100; ++attempt) {
    fresh = runtime.selectOutput(fallback, 100.1);
    if (std::abs(fresh[0].predictions[0].conf - 0.8) < 1.0e-6) {
      replaced = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  ASSERT_TRUE(replaced);
  ASSERT_EQ(fresh.size(), 2U);
  EXPECT_NEAR(fresh[0].predictions[0].conf, 0.8, 1.0e-6);
  EXPECT_DOUBLE_EQ(fresh[1].predictions[0].conf, 1.0);
  EXPECT_FALSE(runtime.takeDiagnosticOutput()) << "production target selection must not emit evaluation diagnostics";

  const auto stale = runtime.selectOutput(fallback, 100.6);
  ASSERT_EQ(stale.size(), 2U);
  EXPECT_DOUBLE_EQ(stale[0].predictions[0].conf, 1.0);
}

TEST(MtrRuntime, InvalidSubmissionNeverStartsInference)
{
  prediction_ml::MtrConfig cfg;
  cfg.model_time_step_s = 0.5;
  auto engine = std::make_unique<ImmediateEngine>(makeOutput());
  prediction_ml::MtrRuntime runtime(cfg, std::move(engine));
  auto input = makeInput("target");
  input.valid = false;
  runtime.submitFrame(std::move(input), "map", 100.0, 1.0, 0.5);

  const std::vector<world_model_msgs::msg::WorldObject> fallback{makeFallback("target")};
  const auto output = runtime.selectOutput(fallback, 100.1);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_DOUBLE_EQ(output[0].predictions[0].conf, 1.0);
}

TEST(MtrRuntime, PinnedEgoDiagnosticIsOneShotAndIndependentOfFallback)
{
  prediction_ml::MtrConfig cfg;
  cfg.model_time_step_s = 0.5;
  cfg.pinned_target_ids = {"__ego__"};
  auto engine = std::make_unique<ImmediateEngine>(makeOutput());
  prediction_ml::MtrRuntime runtime(cfg, std::move(engine));

  runtime.submitFrame(makeInput("__ego__"), "map", 100.0, 1.0, 0.5);

  std::optional<world_model_msgs::msg::WorldObjectArray> diagnostic;
  for (int attempt = 0; attempt < 100 && !diagnostic; ++attempt) {
    diagnostic = runtime.takeDiagnosticOutput();
    if (!diagnostic) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  ASSERT_TRUE(diagnostic);
  EXPECT_EQ(diagnostic->header.frame_id, "map");
  EXPECT_EQ(diagnostic->header.stamp.sec, 100);
  ASSERT_EQ(diagnostic->objects.size(), 1U);
  EXPECT_EQ(diagnostic->objects[0].detection.id, "__ego__");
  ASSERT_EQ(diagnostic->objects[0].predictions.size(), 1U);
  EXPECT_NEAR(diagnostic->objects[0].predictions[0].conf, 0.8, 1.0e-6);
  EXPECT_FALSE(runtime.takeDiagnosticOutput()) << "each diagnostic batch must be returned exactly once";

  const auto production = runtime.selectOutput({}, 100.1);
  EXPECT_TRUE(production.empty()) << "ego diagnostics must not enter production world_object_seeds";
}

namespace
{

// Blocks the first infer() until released, so the test controls which frames are in flight.
class GatedEngine : public prediction_ml::IMtrInferenceEngine
{
public:
  GatedEngine(std::shared_future<void> first_release, std::shared_future<void> later_release)
  : first_release_(std::move(first_release))
  , later_release_(std::move(later_release))
  {}

  bool ready() const override
  {
    return true;
  }

  std::string lastError() const override
  {
    return {};
  }

  prediction_ml::MtrOutputTensors infer(const prediction_ml::MtrInputTensors &) override
  {
    (calls_++ == 0 ? first_release_ : later_release_).wait();
    return makeOutput();
  }

  std::atomic<int> calls_{0};

private:
  std::shared_future<void> first_release_;
  std::shared_future<void> later_release_;
};

bool waitFor(const std::function<bool()> & done)
{
  for (int attempt = 0; attempt < 200; ++attempt) {
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

}  // namespace

TEST(MtrRuntime, ResultSupersededByValidFrameIsStillCached)
{
  // Inference slower than the track period: frame "a" is still running when frame "b" arrives.
  std::promise<void> first;
  std::promise<void> later;
  prediction_ml::MtrConfig cfg;
  cfg.model_time_step_s = 0.5;
  auto engine = std::make_unique<GatedEngine>(first.get_future().share(), later.get_future().share());
  auto * gated = engine.get();
  prediction_ml::MtrRuntime runtime(cfg, std::move(engine));

  runtime.submitFrame(makeInput("a"), "map", 100.0, 1.0, 0.5);
  ASSERT_TRUE(waitFor([&] { return gated->calls_ == 1; }));
  runtime.submitFrame(makeInput("b"), "map", 100.1, 1.0, 0.5);
  first.set_value();

  const std::vector<world_model_msgs::msg::WorldObject> fallback{makeFallback("a"), makeFallback("b")};
  std::vector<world_model_msgs::msg::WorldObject> output;
  EXPECT_TRUE(waitFor([&] {
    output = runtime.selectOutput(fallback, 100.2);
    return std::abs(output[0].predictions[0].conf - 0.8) < 1.0e-6;
  }))
    << "a result superseded by a valid newer frame must not be dropped";
  EXPECT_DOUBLE_EQ(output[1].predictions[0].conf, 1.0) << "frame b is still in flight";
  later.set_value();
}

TEST(MtrRuntime, ResultSupersededByUnusableFrameIsDropped)
{
  std::promise<void> first;
  std::promise<void> later;
  prediction_ml::MtrConfig cfg;
  cfg.model_time_step_s = 0.5;
  auto engine = std::make_unique<GatedEngine>(first.get_future().share(), later.get_future().share());
  auto * gated = engine.get();
  prediction_ml::MtrRuntime runtime(cfg, std::move(engine));

  runtime.submitFrame(makeInput("a"), "map", 100.0, 1.0, 0.5);
  ASSERT_TRUE(waitFor([&] { return gated->calls_ == 1; }));
  auto unusable = makeInput("a");
  unusable.valid = false;  // e.g. scene construction failed for the newer frame
  runtime.submitFrame(std::move(unusable), "map", 100.1, 1.0, 0.5);
  first.set_value();

  const std::vector<world_model_msgs::msg::WorldObject> fallback{makeFallback("a")};
  EXPECT_FALSE(
    waitFor([&] { return std::abs(runtime.selectOutput(fallback, 100.2)[0].predictions[0].conf - 0.8) < 1.0e-6; }));
  later.set_value();
}

namespace
{

class FailingEngine : public prediction_ml::IMtrInferenceEngine
{
public:
  std::atomic<int> calls_{0};

  bool ready() const override
  {
    return true;
  }

  std::string lastError() const override
  {
    return "cudaMalloc(obj_trajs) failed: out of memory";
  }

  prediction_ml::MtrOutputTensors infer(const prediction_ml::MtrInputTensors &) override
  {
    ++calls_;
    return {};  // invalid output, as the TensorRT backend returns on failure
  }
};

std::mutex g_warnings_mutex;
std::vector<std::string> g_warnings;

std::size_t warningCount()
{
  std::lock_guard<std::mutex> lock(g_warnings_mutex);
  return g_warnings.size();
}

void captureWarnings(
  const rcutils_log_location_t *,
  int severity,
  const char *,
  rcutils_time_point_value_t,
  const char * format,
  va_list * args)
{
  if (severity == RCUTILS_LOG_SEVERITY_WARN) {
    char buffer[512];
    std::vsnprintf(buffer, sizeof(buffer), format, *args);
    std::lock_guard<std::mutex> lock(g_warnings_mutex);
    g_warnings.emplace_back(buffer);
  }
}

}  // namespace

TEST(MtrRuntime, InferenceFailureIsLoggedOnceWithTheBackendReason)
{
  ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
  const auto previous_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(captureWarnings);
  {
    prediction_ml::MtrConfig cfg;
    cfg.model_time_step_s = 0.5;
    auto engine = std::make_unique<FailingEngine>();
    auto * failing = engine.get();
    prediction_ml::MtrRuntime runtime(cfg, std::move(engine));
    runtime.submitFrame(makeInput("a"), "map", 100.0, 1.0, 0.5);
    EXPECT_TRUE(waitFor([] { return warningCount() == 1U; }));
    runtime.submitFrame(makeInput("a"), "map", 100.1, 1.0, 0.5);
    EXPECT_TRUE(waitFor([&] { return failing->calls_ == 2; }));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_NE(runtime.lastError().find("out of memory"), std::string::npos) << runtime.lastError();
  }
  rcutils_logging_set_output_handler(previous_handler);

  ASSERT_EQ(g_warnings.size(), 1U) << "a repeated identical failure must not spam the log";
  EXPECT_NE(g_warnings[0].find("out of memory"), std::string::npos) << g_warnings[0];
}
