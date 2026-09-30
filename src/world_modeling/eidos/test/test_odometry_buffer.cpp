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
#include <gtsam/inference/Symbol.h>

#include <cmath>

#include "eidos/utils/odometry_buffer.hpp"

namespace
{

using eidos::KeyframePairQueue;
using eidos::OdometryBuffer;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

/// Straight line at 10 m/s along x while yawing at 0.2 rad/s, sampled at 20 Hz from t0.
OdometryBuffer drive(double t0, double t1, double max_gap = 0.075)
{
  OdometryBuffer buf(max_gap, 30.0);
  for (double t = t0; t <= t1 + 1e-9; t += 0.05) {
    buf.push(t, Pose3(Rot3::Yaw(0.2 * t), Point3(10.0 * t, 0.0, 0.0)));
  }
  return buf;
}

TEST(OdometryBuffer, InterpolatesBetweenSamples)
{
  auto buf = drive(0.0, 2.0);
  Pose3 rel;
  ASSERT_EQ(buf.relative(0.525, 1.2625, rel), OdometryBuffer::Status::kOk);
  const Pose3 expected =
    Pose3(Rot3::Yaw(0.2 * 0.525), Point3(5.25, 0, 0)).between(Pose3(Rot3::Yaw(0.2 * 1.2625), Point3(12.625, 0, 0)));
  EXPECT_TRUE(rel.equals(expected, 1e-9));
}

TEST(OdometryBuffer, ExactSampleTimesAndZeroInterval)
{
  auto buf = drive(0.0, 1.0);
  Pose3 rel;
  ASSERT_EQ(buf.relative(0.0, 1.0, rel), OdometryBuffer::Status::kOk);
  EXPECT_NEAR(rel.translation().norm(), 10.0, 1e-9);
  ASSERT_EQ(buf.relative(0.5, 0.5, rel), OdometryBuffer::Status::kOk);
  EXPECT_TRUE(rel.equals(Pose3(), 1e-12));
}

TEST(OdometryBuffer, ReportsCoverageAndRange)
{
  auto buf = drive(1.0, 2.0);
  Pose3 rel;
  EXPECT_EQ(buf.relative(1.5, 2.2, rel), OdometryBuffer::Status::kNotYetCovered);
  EXPECT_EQ(buf.relative(0.9, 1.5, rel), OdometryBuffer::Status::kOutOfRange);
  EXPECT_EQ(OdometryBuffer(0.1, 30.0).relative(0.0, 0.1, rel), OdometryBuffer::Status::kOutOfRange);
}

TEST(OdometryBuffer, NeverBridgesAGap)
{
  OdometryBuffer buf(0.075, 30.0);
  for (double t : {0.0, 0.05, 0.10, 0.25, 0.30}) {  // one 150 ms gap: a VO reset
    buf.push(t, Pose3(Rot3(), Point3(t, 0, 0)));
  }
  Pose3 rel;
  EXPECT_EQ(buf.relative(0.02, 0.28, rel), OdometryBuffer::Status::kGap);
  EXPECT_EQ(buf.relative(0.12, 0.2, rel), OdometryBuffer::Status::kGap);  // interval inside the gap
  EXPECT_EQ(buf.relative(0.0, 0.1, rel), OdometryBuffer::Status::kOk);
  EXPECT_EQ(buf.relative(0.25, 0.3, rel), OdometryBuffer::Status::kOk);
}

TEST(OdometryBuffer, IgnoresNonIncreasingAndTrimsHistory)
{
  OdometryBuffer buf(0.1, 1.0);
  EXPECT_TRUE(buf.push(0.0, Pose3()));
  EXPECT_FALSE(buf.push(0.0, Pose3()));
  EXPECT_FALSE(buf.push(-1.0, Pose3()));
  for (double t = 0.05; t <= 3.0; t += 0.05) buf.push(t, Pose3());
  EXPECT_LE(buf.newest() - buf.oldest(), 1.0 + 1e-9);
}

TEST(ConjugateMotion, CameraRigMotionToBaseFootprint)
{
  // base_link is 1.76 m above base_footprint; the rig (base_link) yaws 90 deg in place.
  const Pose3 T_fp_bl(Rot3(), Point3(0, 0, 1.76));
  const Pose3 rel_bl(Rot3::Yaw(M_PI / 2), Point3(0, 0, 0));
  const Pose3 rel_fp = eidos::conjugateMotion(T_fp_bl, rel_bl);
  EXPECT_TRUE(rel_fp.equals(Pose3(Rot3::Yaw(M_PI / 2), Point3(0, 0, 0)), 1e-12));  // vertical offset: pure yaw

  // A rig 1 m ahead of the body turning in place means the body swings around it.
  const Pose3 T_b_c(Rot3(), Point3(1.0, 0, 0));
  const Pose3 rel_b = eidos::conjugateMotion(T_b_c, rel_bl);
  EXPECT_TRUE(rel_b.rotation().equals(Rot3::Yaw(M_PI / 2), 1e-12));
  EXPECT_TRUE(gtsam::assert_equal(Point3(1.0, -1.0, 0.0), rel_b.translation(), 1e-12));
  // And consistency: body motion re-expressed for the rig gives the rig motion back.
  EXPECT_TRUE(eidos::conjugateMotion(T_b_c.inverse(), rel_b).equals(rel_bl, 1e-12));
}

TEST(KeyframePairQueue, PairsConsecutiveStatesInOrder)
{
  using gtsam::Symbol;
  KeyframePairQueue q;
  q.addState(Symbol('x', 0), 1.0);
  EXPECT_TRUE(q.empty());  // first state has no partner
  q.addState(Symbol('x', 1), 1.5);
  q.addState(Symbol('x', 2), 2.0);
  ASSERT_EQ(q.size(), 2u);
  EXPECT_EQ(q.front().from.key, Symbol('x', 0));
  EXPECT_EQ(q.front().to.key, Symbol('x', 1));
  EXPECT_DOUBLE_EQ(q.front().to.t, 1.5);
  q.pop();
  EXPECT_EQ(q.front().from.key, Symbol('x', 1));
  EXPECT_EQ(q.front().to.key, Symbol('x', 2));
}

TEST(KeyframePairQueue, IgnoresOutOfOrderStatesAndClears)
{
  using gtsam::Symbol;
  KeyframePairQueue q;
  q.addState(Symbol('x', 0), 1.0);
  q.addState(Symbol('x', 1), 2.0);
  q.addState(Symbol('x', 2), 1.5);  // older than the last state: ignored
  q.addState(Symbol('x', 3), 2.0);  // same time as the last state: ignored
  ASSERT_EQ(q.size(), 1u);
  q.addState(Symbol('x', 4), 3.0);
  EXPECT_EQ(q.pending().back().from.key, Symbol('x', 1));
  q.clear();
  q.addState(Symbol('x', 5), 7.0);
  EXPECT_TRUE(q.empty());  // clear() forgets the previous state too
}

}  // namespace
