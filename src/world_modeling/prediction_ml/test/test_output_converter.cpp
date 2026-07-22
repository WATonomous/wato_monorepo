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

#include <cmath>
#include <limits>

#include "prediction_ml/output_converter.hpp"

namespace
{

prediction_ml::MtrBatchSidecar makeSidecar()
{
  prediction_ml::MtrBatchSidecar sidecar;
  sidecar.frame_id = "map";
  prediction_ml::MtrTargetSidecar target;
  target.detection_id = "agent-7";
  target.center_x = 10.0;
  target.center_y = 20.0;
  target.center_heading = 3.14159265358979323846 / 2.0;
  sidecar.targets.push_back(target);
  return sidecar;
}

prediction_ml::MtrOutputTensors makeTwoModeOutput()
{
  prediction_ml::MtrOutputTensors output;
  output.valid = true;
  output.scores_shape = {1, 2};
  output.trajs_shape = {1, 2, 3, 2};
  output.pred_scores = {0.25F, 0.75F};
  output.pred_trajs = {1.0F, 0.0F, 2.0F, 0.0F, 3.0F, 0.0F, 0.0F, 1.0F, 0.0F, 2.0F, 0.0F, 3.0F};
  return output;
}

}  // namespace

TEST(OutputConverter, RotatesTranslatesSortsAndTimestampsModes)
{
  const auto result = prediction_ml::convertMtrOutput(makeTwoModeOutput(), makeSidecar(), "map", 12.0, 1.5, 0.5);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.objects.size(), 1U);
  EXPECT_EQ(result.objects[0].detection_id, "agent-7");
  ASSERT_EQ(result.objects[0].predictions.size(), 2U);
  const auto & prediction = result.objects[0].predictions[0];
  EXPECT_DOUBLE_EQ(prediction.conf, 0.75);
  ASSERT_EQ(prediction.poses.size(), 3U);
  EXPECT_NEAR(prediction.poses[0].pose.position.x, 9.0, 1.0e-6);
  EXPECT_NEAR(prediction.poses[0].pose.position.y, 20.0, 1.0e-6);
  EXPECT_NEAR(prediction.poses[0].pose.orientation.z, 1.0, 1.0e-6);
  EXPECT_NEAR(prediction.poses[0].pose.orientation.w, 0.0, 1.0e-6);
  EXPECT_EQ(prediction.poses[0].header.stamp.sec, 12);
  EXPECT_EQ(prediction.poses[0].header.stamp.nanosec, 500000000U);
}

TEST(OutputConverter, RejectsShapeAndBufferMismatch)
{
  auto output = makeTwoModeOutput();
  output.pred_trajs.pop_back();
  const auto result = prediction_ml::convertMtrOutput(output, makeSidecar(), "map", 12.0, 1.5, 0.5);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, "mtr output buffer size mismatch");
}

TEST(OutputConverter, DropsInvalidModeButKeepsValidObject)
{
  auto output = makeTwoModeOutput();
  output.pred_trajs[0] = std::numeric_limits<float>::quiet_NaN();
  const auto result = prediction_ml::convertMtrOutput(output, makeSidecar(), "map", 12.0, 1.5, 0.5);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.objects.size(), 1U);
  ASSERT_EQ(result.objects[0].predictions.size(), 1U);
  EXPECT_DOUBLE_EQ(result.objects[0].predictions[0].conf, 0.75);
}
