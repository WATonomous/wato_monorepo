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

#include "eidos/utils/lidar_deskew.hpp"

namespace eidos
{

TEST(LidarDeskew, FiringTimesSpanTheRevolutionEndingAtTheStamp)
{
  EXPECT_NEAR(firingTime(10.0, 0.05, 0, 1000), 9.950025, 1e-9);
  EXPECT_NEAR(firingTime(10.0, 0.05, 999, 1000), 9.999975, 1e-9);
  EXPECT_DOUBLE_EQ(firingTime(10.0, 0.0, 500, 1000), 10.0);
}

TEST(LidarDeskew, DrivingForwardBringsEarlierPointsCloser)
{
  // 10 m/s forward; a point 10 m ahead seen 20 ms before tau is 9.8 m ahead at tau
  const Eigen::Vector3d p = moveToTime({10, 0, 0}, -0.02, {10, 0, 0}, Eigen::Vector3d::Zero());
  EXPECT_NEAR(p.x(), 9.8, 1e-12);
  EXPECT_NEAR(p.y(), 0.0, 1e-12);
}

TEST(LidarDeskew, TurningLeftPutsLaterPointsFurtherLeft)
{
  // Yawing left at 0.5 rad/s, the vehicle at tau + 40 ms has turned 0.02 rad left of its heading at
  // tau. A point straight ahead of it then is 0.02 rad left of straight ahead at tau.
  const Eigen::Vector3d p = moveToTime({20, 0, 0}, 0.04, Eigen::Vector3d::Zero(), {0, 0, 0.5});
  EXPECT_NEAR(std::atan2(p.y(), p.x()), 0.02, 1e-12);
  EXPECT_NEAR(p.norm(), 20.0, 1e-12);
}

TEST(LidarDeskew, NoMotionNoChange)
{
  const Eigen::Vector3d p = moveToTime({1, 2, 3}, 0.03, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_TRUE(p.isApprox(Eigen::Vector3d(1, 2, 3)));
}

}  // namespace eidos
