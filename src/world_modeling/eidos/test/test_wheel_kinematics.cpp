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
#include <deque>

#include "eidos/utils/wheel_kinematics.hpp"

namespace wk = eidos::wheel_kinematics;

static constexpr double kRadius = 0.31235;
static constexpr double kRearAxleOffset = -0.8629;  // rear axle behind base_footprint

TEST(WheelKinematics, AxleSpeedAveragesWheels)
{
  // 10 m/s and 12 m/s at the tire -> 11 m/s at the axle centre
  EXPECT_NEAR(wk::axleSpeed(10.0 / kRadius, 12.0 / kRadius, kRadius), 11.0, 1e-9);
}

TEST(WheelKinematics, AxleSpeedIsForwardOnly)
{
  EXPECT_DOUBLE_EQ(wk::axleSpeed(-1.0, -1.0, kRadius), 0.0);
  EXPECT_DOUBLE_EQ(wk::axleSpeed(0.0, 0.0, kRadius), 0.0);
}

TEST(WheelKinematics, LeverArmStraightHasNoLateral)
{
  Eigen::Vector3d v = wk::axleToBaseVelocity(
    Eigen::Vector3d(5.0, 0.0, 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d(kRearAxleOffset, 0.0, 0.3));
  EXPECT_NEAR(v.x(), 5.0, 1e-12);
  EXPECT_NEAR(v.y(), 0.0, 1e-12);
  EXPECT_NEAR(v.z(), 0.0, 1e-12);
}

TEST(WheelKinematics, LeverArmYawGivesLateralAtBase)
{
  // Left turn: a point ahead of the rear axle moves left at omega * distance
  const double wz = 0.2;
  Eigen::Vector3d v = wk::axleToBaseVelocity(
    Eigen::Vector3d(5.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, wz), Eigen::Vector3d(kRearAxleOffset, 0.0, 0.0));
  EXPECT_NEAR(v.x(), 5.0, 1e-12);
  EXPECT_NEAR(v.y(), wz * -kRearAxleOffset, 1e-12);
}

TEST(WheelKinematics, CovarianceWidensAtZeroSpeed)
{
  wk::SpeedNoise n;
  auto moving = wk::twistCovariance(false, n);
  auto stopped = wk::twistCovariance(true, n);
  EXPECT_DOUBLE_EQ(moving[0], n.speed_std_moving * n.speed_std_moving);
  EXPECT_DOUBLE_EQ(stopped[0], n.zero_speed_std * n.zero_speed_std);
  EXPECT_DOUBLE_EQ(moving[7], n.lateral_std * n.lateral_std);  // lin_y
  EXPECT_DOUBLE_EQ(moving[14], n.vertical_std * n.vertical_std);  // lin_z
  EXPECT_DOUBLE_EQ(moving[35], n.yaw_rate_std * n.yaw_rate_std);  // ang_z
  EXPECT_DOUBLE_EQ(moving[1], 0.0);  // off-diagonal
}

TEST(WheelKinematics, ConstantTwistMatchesCircularArc)
{
  // v along a circle of radius R = v / w, quarter turn
  const double v = 4.0;
  const double w = 0.5;
  const double T = (M_PI / 2.0) / w;
  wk::Pose2 d = wk::integrateConstantTwist(v, 0.0, w, T);
  const double R = v / w;
  EXPECT_NEAR(d.x, R, 1e-9);
  EXPECT_NEAR(d.y, R, 1e-9);
  EXPECT_NEAR(d.theta, M_PI / 2.0, 1e-12);
}

TEST(WheelKinematics, SmallStepsAccumulateToArc)
{
  const double v = 4.0;
  const double w = 0.5;
  const double T = (M_PI / 2.0) / w;
  const int n = 1000;
  wk::Pose2 p;
  for (int i = 0; i < n; ++i) {
    p = wk::compose(p, wk::integrateConstantTwist(v, 0.0, w, T / n));
  }
  EXPECT_NEAR(p.x, v / w, 1e-6);
  EXPECT_NEAR(p.y, v / w, 1e-6);
  EXPECT_NEAR(p.theta, M_PI / 2.0, 1e-9);
}

TEST(WheelKinematics, SegmentIntegratesWindowOnly)
{
  std::deque<wk::Sample> samples;
  for (int i = 1; i <= 20; ++i) {
    wk::Sample s;
    s.t = i * 0.1;
    s.vx = 2.0;
    samples.push_back(s);
  }
  // Window (0.5, 1.5]: 1.0 s at 2 m/s
  wk::Segment seg = wk::integrateSegment(samples, 0.5, 1.5, 0.5);
  EXPECT_NEAR(seg.delta.x, 2.0, 1e-9);
  EXPECT_NEAR(seg.distance, 2.0, 1e-9);
  EXPECT_NEAR(seg.delta.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(seg.zero_speed_time, 0.0);
}

TEST(WheelKinematics, SegmentHoldsLastSampleToWindowEnd)
{
  std::deque<wk::Sample> samples;
  wk::Sample s;
  s.t = 0.1;
  s.vx = 1.0;
  samples.push_back(s);
  wk::Segment seg = wk::integrateSegment(samples, 0.0, 0.3, 0.5);
  EXPECT_NEAR(seg.distance, 0.3, 1e-9);
}

TEST(WheelKinematics, SegmentSkipsStaleGaps)
{
  std::deque<wk::Sample> samples;
  wk::Sample a;
  a.t = 0.1;
  a.vx = 1.0;
  wk::Sample b;
  b.t = 2.0;  // 1.9 s gap > max_dt
  b.vx = 1.0;
  samples.push_back(a);
  samples.push_back(b);
  wk::Segment seg = wk::integrateSegment(samples, 0.0, 2.0, 0.5);
  EXPECT_NEAR(seg.distance, 0.1, 1e-9);
}

TEST(WheelKinematics, SegmentTracksZeroSpeedTime)
{
  std::deque<wk::Sample> samples;
  for (int i = 1; i <= 10; ++i) {
    wk::Sample s;
    s.t = i * 0.1;
    s.zero_speed = true;
    samples.push_back(s);
  }
  wk::Segment seg = wk::integrateSegment(samples, 0.0, 1.0, 0.5);
  EXPECT_NEAR(seg.zero_speed_time, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(seg.distance, 0.0);
}

TEST(WheelKinematics, BetweenNoiseScalesWithDistanceAndCreep)
{
  wk::FactorNoise n;
  wk::Segment short_seg;
  short_seg.distance = 1.0;
  wk::Segment long_seg;
  long_seg.distance = 50.0;
  wk::Segment creep_seg;
  creep_seg.zero_speed_time = 2.0;

  auto s_short = wk::betweenStd(short_seg, n);
  auto s_long = wk::betweenStd(long_seg, n);
  auto s_creep = wk::betweenStd(creep_seg, n);

  EXPECT_GT(s_long[3], s_short[3]);
  EXPECT_NEAR(s_long[3], n.trans_std_per_meter * 50.0, 1e-12);
  EXPECT_NEAR(s_creep[3], n.zero_speed_std * 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(s_short[2], n.min_std);  // no yaw change -> floor
  EXPECT_DOUBLE_EQ(s_short[0], n.loose_std);  // roll unconstrained
  EXPECT_DOUBLE_EQ(s_short[5], n.loose_std);  // z unconstrained
}
