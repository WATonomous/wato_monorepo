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

#include <limits>

#include "prediction_ml/fallback_prediction.hpp"

namespace prediction_ml
{
namespace
{

vision_msgs::msg::Detection3D withVelocity(double vx, double vy, double score)
{
  vision_msgs::msg::Detection3D detection;
  vision_msgs::msg::ObjectHypothesisWithPose label;
  label.hypothesis.class_id = "car";
  label.hypothesis.score = 1.0;
  detection.results.push_back(label);
  vision_msgs::msg::ObjectHypothesisWithPose velocity;
  velocity.hypothesis.class_id = "linear_velocity";
  velocity.hypothesis.score = score;
  velocity.pose.pose.position.x = vx;
  velocity.pose.pose.position.y = vy;
  detection.results.push_back(velocity);
  return detection;
}

TEST(TrackerVelocity, UsesScoredLinearVelocity)
{
  const auto v = trackerVelocity(withVelocity(3.0, -1.5, 1.0));
  EXPECT_DOUBLE_EQ(v[0], 3.0);
  EXPECT_DOUBLE_EQ(v[1], -1.5);
}

TEST(TrackerVelocity, ZeroWithoutAUsableVelocity)
{
  for (const auto & detection :
       {vision_msgs::msg::Detection3D{},
        withVelocity(3.0, 0.0, 0.0),
        withVelocity(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)})
  {
    const auto v = trackerVelocity(detection);
    EXPECT_DOUBLE_EQ(v[0], 0.0);
    EXPECT_DOUBLE_EQ(v[1], 0.0);
  }
}

}  // namespace
}  // namespace prediction_ml
