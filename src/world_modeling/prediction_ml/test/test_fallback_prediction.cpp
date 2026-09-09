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

// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "prediction_ml/fallback_prediction.hpp"

namespace prediction_ml
{
namespace
{

constexpr double kVehicleSizeThresholdM = 3.5;
constexpr double kVehicleSpeedMps = 5.0;
constexpr double kVruSpeedMps = 1.4;

TEST(SelectFallbackSpeed, UsesVehicleSpeedAboveThreshold)
{
  EXPECT_DOUBLE_EQ(selectFallbackSpeed(3.6, kVehicleSizeThresholdM, kVehicleSpeedMps, kVruSpeedMps), kVehicleSpeedMps);
}

TEST(SelectFallbackSpeed, UsesVruSpeedAtThreshold)
{
  EXPECT_DOUBLE_EQ(
    selectFallbackSpeed(kVehicleSizeThresholdM, kVehicleSizeThresholdM, kVehicleSpeedMps, kVruSpeedMps), kVruSpeedMps);
}

TEST(SelectFallbackSpeed, UsesVruSpeedBelowThreshold)
{
  EXPECT_DOUBLE_EQ(selectFallbackSpeed(3.4, kVehicleSizeThresholdM, kVehicleSpeedMps, kVruSpeedMps), kVruSpeedMps);
}

}  // namespace
}  // namespace prediction_ml
