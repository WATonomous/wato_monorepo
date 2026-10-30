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
#include "prediction_ml/mtr_types.hpp"

TEST(MtrContract, ParseMode)
{
  EXPECT_EQ(prediction_ml::parseMtrMode("disabled"), prediction_ml::MtrMode::Disabled);
  EXPECT_EQ(prediction_ml::parseMtrMode("null"), prediction_ml::MtrMode::Null);
  EXPECT_EQ(prediction_ml::parseMtrMode("tensorrt"), prediction_ml::MtrMode::TensorRt);
}

TEST(MtrContract, DefaultsAreFallbackSafe)
{
  prediction_ml::MtrConfig cfg;
  EXPECT_EQ(cfg.mode, prediction_ml::MtrMode::Disabled);
  prediction_ml::MtrInputTensors in;
  EXPECT_FALSE(in.valid);
  prediction_ml::MtrOutputTensors out;
  EXPECT_FALSE(out.valid);
}
