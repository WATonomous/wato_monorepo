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

#include "prediction_ml/mtr_inference_engine.hpp"

TEST(NullBackend, NotReadyAndInvalidOutput)
{
  prediction_ml::MtrConfig cfg;
  auto engine = prediction_ml::createNullMtrInferenceEngine(cfg);
  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  prediction_ml::MtrInputTensors in;
  auto out = engine->infer(in);
  EXPECT_FALSE(out.valid);
}

TEST(NullBackend, TensorRtFactoryFallsBackWhenDisabled)
{
  prediction_ml::MtrConfig cfg;
  auto engine = prediction_ml::createTensorRtMtrInferenceEngine(cfg);
  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
}
