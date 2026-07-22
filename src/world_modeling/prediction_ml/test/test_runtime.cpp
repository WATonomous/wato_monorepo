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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_runtime.hpp"

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

  const auto stale = runtime.selectOutput(fallback, 100.6);
  ASSERT_EQ(stale.size(), 2U);
  EXPECT_DOUBLE_EQ(stale[0].predictions[0].conf, 1.0);
}

TEST(MtrRuntime, InvalidSubmissionNeverStartsInference)
{
  prediction_ml::MtrConfig cfg;
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
