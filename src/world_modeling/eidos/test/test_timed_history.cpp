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

#include "eidos/utils/timed_history.hpp"

namespace
{

using eidos::interpolateAngle;
using eidos::interpolatePoint;
using eidos::TimedHistory;

TEST(TimedHistory, InterpolatesPoints)
{
  TimedHistory<Eigen::Vector3d> history(10.0);
  history.push(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  history.push(2.0, Eigen::Vector3d(10.0, -2.0, 1.0));
  const auto p = interpolatePoint(history, 1.25);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->x(), 2.5, 1e-12);
  EXPECT_NEAR(p->y(), -0.5, 1e-12);
  EXPECT_FALSE(interpolatePoint(history, 0.5).has_value());  // before the first sample
  EXPECT_FALSE(interpolatePoint(history, 2.5).has_value());  // after the newest: not known yet
}

TEST(TimedHistory, InterpolatesAnglesAcrossTheWrap)
{
  TimedHistory<double> history(10.0);
  history.push(0.0, M_PI - 0.1);
  history.push(1.0, -M_PI + 0.1);  // turned 0.2 rad through +-pi, not 2 pi - 0.2 the other way
  const auto a = interpolateAngle(history, 0.5);
  ASSERT_TRUE(a.has_value());
  EXPECT_NEAR(std::abs(*a), M_PI, 1e-9);
  EXPECT_NEAR(*interpolateAngle(history, 0.25), M_PI - 0.05, 1e-9);
}

TEST(TimedHistory, KeepsOnlyRecentInOrderSamples)
{
  TimedHistory<double> history(1.0);
  for (int i = 0; i <= 30; ++i) history.push(0.1 * i, 0.0);
  history.push(2.0, 5.0);  // older than the newest: ignored
  EXPECT_FALSE(interpolateAngle(history, 1.0).has_value());
  EXPECT_NEAR(*interpolateAngle(history, 2.95), 0.0, 1e-12);
}

}  // namespace
