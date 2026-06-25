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

#include <vector>

#include "prediction_ml/mtr_runtime.hpp"

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
