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

#include <array>

#include "eidos_transform/msg_covariance.hpp"

using eidos_transform::noiseFromMsgCovariance;
using Vec6 = Eigen::Matrix<double, 6, 1>;

static std::array<double, 36> diagonal(const std::array<double, 6> & d)
{
  std::array<double, 36> cov{};
  for (size_t i = 0; i < 6; ++i) cov[i * 6 + i] = d[i];
  return cov;
}

TEST(MsgCovariance, ReordersLinearAngularToEkfOrder)
{
  // msg order: [x, y, z, rot_x, rot_y, rot_z] variances
  auto cov = diagonal({1.0, 4.0, 9.0, 16.0, 25.0, 36.0});
  Vec6 out = noiseFromMsgCovariance(cov, Vec6::Constant(99.0));
  // EKF order: [rot_x, rot_y, rot_z, x, y, z] std-devs
  EXPECT_DOUBLE_EQ(out(0), 4.0);
  EXPECT_DOUBLE_EQ(out(1), 5.0);
  EXPECT_DOUBLE_EQ(out(2), 6.0);
  EXPECT_DOUBLE_EQ(out(3), 1.0);
  EXPECT_DOUBLE_EQ(out(4), 2.0);
  EXPECT_DOUBLE_EQ(out(5), 3.0);
}

TEST(MsgCovariance, FallsBackOnUnsetEntries)
{
  auto cov = diagonal({0.25, 0.0, -1.0, 0.0, 0.0, 0.01});
  Vec6 fallback;
  fallback << 7.0, 8.0, 9.0, 10.0, 11.0, 12.0;
  Vec6 out = noiseFromMsgCovariance(cov, fallback);
  EXPECT_DOUBLE_EQ(out(0), 7.0);  // rot_x unset
  EXPECT_DOUBLE_EQ(out(1), 8.0);  // rot_y unset
  EXPECT_DOUBLE_EQ(out(2), 0.1);  // rot_z
  EXPECT_DOUBLE_EQ(out(3), 0.5);  // x
  EXPECT_DOUBLE_EQ(out(4), 11.0);  // y zero
  EXPECT_DOUBLE_EQ(out(5), 12.0);  // z negative
}

TEST(MsgCovariance, IgnoresOffDiagonal)
{
  auto cov = diagonal({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  cov[1] = 100.0;
  cov[6] = 100.0;
  Vec6 out = noiseFromMsgCovariance(cov, Vec6::Zero());
  for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(out(i), 1.0);
}
