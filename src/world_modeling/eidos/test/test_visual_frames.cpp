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

#include <any>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "eidos/map/raw_float_vector.hpp"
#include "eidos/map/visual_frame_set.hpp"
#include "eidos/utils/visual_frames.hpp"

namespace
{

using eidos::VisualFrame;
using eidos::VisualFrameBuffer;
using eidos::VisualFrameSet;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

std::shared_ptr<const VisualFrame> frame(const std::string & camera, double stamp, uint8_t fill = 0)
{
  auto f = std::make_shared<VisualFrame>();
  f->frame_id = camera;
  f->stamp = stamp;
  f->format = "jpeg";
  f->data = {fill, 1, 2, 3};
  return f;
}

/// Two cameras at 20 Hz, the second 20 ms behind the first.
VisualFrameBuffer twoCameras(double t0, double t1)
{
  VisualFrameBuffer buf(2, 10.0);
  for (double t = t0; t <= t1 + 1e-9; t += 0.05) {
    buf.push(0, frame("a", t));
    buf.push(1, frame("b", t + 0.02));
  }
  return buf;
}

TEST(VisualFrameBuffer, PicksEveryCamerasNearestFrame)
{
  auto buf = twoCameras(0.0, 1.0);
  const auto set = buf.nearest(0.51, 0.06);
  ASSERT_TRUE(set.has_value());
  ASSERT_EQ(set->frames.size(), 2u);
  EXPECT_NEAR(set->frames[0]->stamp, 0.50, 1e-9);
  EXPECT_NEAR(set->frames[1]->stamp, 0.52, 1e-9);
  EXPECT_NEAR(set->stamp, 0.50, 1e-9);  // the first camera's stamp
}

TEST(VisualFrameBuffer, NoSetWhenACameraIsTooFarInTime)
{
  VisualFrameBuffer buf(2, 10.0);
  buf.push(0, frame("a", 1.0));
  buf.push(1, frame("b", 1.5));
  EXPECT_FALSE(buf.nearest(1.0, 0.06).has_value());
  EXPECT_TRUE(buf.nearest(1.0, 0.6).has_value());
}

TEST(VisualFrameBuffer, CoversOnceEveryCameraHasALaterFrame)
{
  VisualFrameBuffer buf(2, 10.0);
  buf.push(0, frame("a", 1.0));
  EXPECT_FALSE(buf.covers(0.9));  // camera b has delivered nothing
  buf.push(1, frame("b", 0.95));
  EXPECT_TRUE(buf.covers(0.9));
  EXPECT_FALSE(buf.covers(0.97));
  EXPECT_NEAR(*buf.newest(), 1.0, 1e-9);
  EXPECT_NEAR(*buf.newestCommon(), 0.95, 1e-9);
}

TEST(VisualFrameBuffer, DropsOldFramesAndIgnoresOutOfOrderOnes)
{
  VisualFrameBuffer buf(1, 1.0);
  for (double t = 0.0; t <= 3.0 + 1e-9; t += 0.1) buf.push(0, frame("a", t));
  buf.push(0, frame("a", 2.5));  // older than the newest: ignored
  EXPECT_FALSE(buf.nearest(0.5, 0.06).has_value());
  EXPECT_NEAR(buf.nearest(2.52, 0.06)->frames[0]->stamp, 2.5, 1e-9);
  EXPECT_NEAR(*buf.newest(), 3.0, 1e-9);
}

TEST(ConstantVelocityMotion, InterpolatesAlongAnArc)
{
  // 10 m/s forward while yawing at 0.2 rad/s
  const gtsam::Vector6 twist = (gtsam::Vector6() << 0.0, 0.0, 0.2, 10.0, 0.0, 0.0).finished();
  const Pose3 a(Rot3::Yaw(0.3), Point3(5.0, -2.0, 0.0));
  const Pose3 b = a * Pose3::Expmap(twist * 0.5);
  const Pose3 motion = eidos::constantVelocityMotion(a, 10.0, b, 10.5, 0.04);
  EXPECT_TRUE(motion.equals(Pose3::Expmap(twist * 0.04), 1e-9));
  EXPECT_NEAR(motion.x(), 0.4, 1e-3);
  // backwards in time
  EXPECT_TRUE(eidos::constantVelocityMotion(a, 10.0, b, 10.5, -0.04).equals(Pose3::Expmap(-twist * 0.04), 1e-9));
  // the same instant twice says nothing about velocity
  EXPECT_TRUE(eidos::constantVelocityMotion(a, 10.0, b, 10.0, 0.04).equals(Pose3(), 1e-12));
}

TEST(PlaceSimilarity, MatchesCamerasInAnyOrder)
{
  // unit descriptors of length 2 for two cameras
  const std::vector<float> a = {1.0f, 0.0f, 0.0f, 1.0f};
  const std::vector<float> swapped = {0.0f, 1.0f, 1.0f, 0.0f};
  const std::vector<float> other = {-1.0f, 0.0f, 0.0f, -1.0f};
  EXPECT_NEAR(eidos::placeSimilarity(a, a, 2), 1.0, 1e-9);
  EXPECT_NEAR(eidos::placeSimilarity(a, swapped, 2), 1.0, 1e-9);
  EXPECT_NEAR(eidos::placeSimilarity(a, other, 2), 0.0, 1e-9);  // each camera's best match is orthogonal
  EXPECT_LT(eidos::placeSimilarity(a, {}, 2), 0.0);
  EXPECT_LT(eidos::placeSimilarity(a, {1.0f, 0.0f, 0.0f}, 2), 0.0);  // not a whole number of descriptors
}

TEST(Formats, VisualFrameSetRoundTrip)
{
  VisualFrameSet set;
  set.stamp = 1234.5;
  set.frames = {frame("camera_pano_nn", 1234.5, 7), frame("camera_pano_ne", 1234.52, 9)};
  eidos::formats::VisualFrameSetBinary format;
  const auto bytes = format.serialize(std::any(set));
  const auto back = std::any_cast<VisualFrameSet>(format.deserialize(bytes));
  EXPECT_DOUBLE_EQ(back.stamp, set.stamp);
  ASSERT_EQ(back.frames.size(), 2u);
  EXPECT_EQ(back.frames[1]->frame_id, "camera_pano_ne");
  EXPECT_EQ(back.frames[1]->format, "jpeg");
  EXPECT_DOUBLE_EQ(back.frames[1]->stamp, 1234.52);
  EXPECT_EQ(back.frames[0]->data, (std::vector<uint8_t>{7, 1, 2, 3}));

  // a truncated blob is rejected, not half-read
  std::vector<uint8_t> truncated(bytes.begin(), bytes.end() - 3);
  EXPECT_FALSE(format.deserialize(truncated).has_value());
  EXPECT_FALSE(format.deserialize({}).has_value());
}

TEST(Formats, RawFloatVectorRoundTrip)
{
  const std::vector<float> values = {0.5f, -1.25f, 3.0f};
  eidos::formats::RawFloatVector format;
  const auto bytes = format.serialize(std::any(values));
  EXPECT_EQ(bytes.size(), 12u);
  EXPECT_EQ(std::any_cast<std::vector<float>>(format.deserialize(bytes)), values);
  EXPECT_FALSE(format.deserialize({1, 2, 3}).has_value());
}

}  // namespace
