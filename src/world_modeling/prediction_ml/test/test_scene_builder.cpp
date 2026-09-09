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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "prediction_ml/scene_builder.hpp"

namespace prediction_ml
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Fixed contract capacities the packer targets.
constexpr std::size_t kTargetCap = 8;
constexpr std::size_t kContextCap = 128;

void setStamp(builtin_interfaces::msg::Time & stamp, double seconds)
{
  const double sec_floor = std::floor(seconds);
  stamp.sec = static_cast<int32_t>(sec_floor);
  stamp.nanosec = static_cast<uint32_t>(std::llround((seconds - sec_floor) * 1e9));
}

geometry_msgs::msg::Quaternion yawQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

// Build a Detection3D with a class hypothesis and (optionally) a linear_velocity hypothesis.
vision_msgs::msg::Detection3D makeDetection(
  const std::string & id,
  const std::string & label,
  double x,
  double y,
  double z,
  double yaw,
  bool with_velocity = false,
  double vx = 0.0,
  double vy = 0.0,
  double vz = 0.0,
  double dx = 4.0,
  double dy = 2.0,
  double dz = 1.5)
{
  vision_msgs::msg::Detection3D det;
  det.id = id;
  det.bbox.center.position.x = x;
  det.bbox.center.position.y = y;
  det.bbox.center.position.z = z;
  det.bbox.center.orientation = yawQuat(yaw);
  det.bbox.size.x = dx;
  det.bbox.size.y = dy;
  det.bbox.size.z = dz;

  vision_msgs::msg::ObjectHypothesisWithPose cls;
  cls.hypothesis.class_id = label;
  cls.hypothesis.score = 0.9;
  det.results.push_back(cls);

  if (with_velocity) {
    vision_msgs::msg::ObjectHypothesisWithPose vel;
    vel.hypothesis.class_id = "linear_velocity";
    vel.hypothesis.score = 1.0;
    vel.pose.pose.position.x = vx;
    vel.pose.pose.position.y = vy;
    vel.pose.pose.position.z = vz;
    det.results.push_back(vel);
  }
  return det;
}

vision_msgs::msg::Detection3DArray makeArray(
  double stamp_s, const std::vector<vision_msgs::msg::Detection3D> & dets, const std::string & frame_id = "map")
{
  vision_msgs::msg::Detection3DArray arr;
  setStamp(arr.header.stamp, stamp_s);
  arr.header.frame_id = frame_id;
  arr.detections = dets;
  return arr;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = yawQuat(yaw);
  return pose;
}

MtrFrameContext makeFrame(const vision_msgs::msg::Detection3DArray & dets, const geometry_msgs::msg::PoseStamped & ego)
{
  MtrFrameContext frame;
  frame.detections = dets;
  frame.ego_pose = ego;
  frame.has_ego = true;
  return frame;
}

// One lanelet holding a centerline of scene-frame points (z defaults to 0).
lanelet_msgs::msg::Lanelet makeLanelet(const std::vector<std::array<double, 3>> & centerline)
{
  lanelet_msgs::msg::Lanelet lanelet;
  for (const auto & p : centerline) {
    geometry_msgs::msg::Point pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    lanelet.centerline.push_back(pt);
  }
  return lanelet;
}

MtrFrameContext makeFrameWithMap(
  const vision_msgs::msg::Detection3DArray & dets,
  const geometry_msgs::msg::PoseStamped & ego,
  const std::vector<lanelet_msgs::msg::Lanelet> & lanelets)
{
  auto frame = makeFrame(dets, ego);
  frame.has_map = true;
  frame.lanelet_ahead.lanelets = lanelets;
  return frame;
}

// Fixed map-tensor capacities the packer targets ([8, 768, 20, 9]).
constexpr std::size_t kNumSrcPolylines = 768;
constexpr std::size_t kNumPointsPerPolyline = 20;
constexpr std::size_t kMapFeatureDim = 9;

// Row-major index into map_polylines [T_cap, 768, 20, 9].
std::size_t mapIdx(std::size_t tgt, std::size_t poly, std::size_t point, std::size_t feat)
{
  return ((tgt * kNumSrcPolylines + poly) * kNumPointsPerPolyline + point) * kMapFeatureDim + feat;
}

std::size_t mapMaskIdx(std::size_t tgt, std::size_t poly, std::size_t point)
{
  return (tgt * kNumSrcPolylines + poly) * kNumPointsPerPolyline + point;
}

// Row-major index into obj_trajs given the shape [T_cap, C_cap, T, F].
std::size_t objIdx(const MtrInputTensors & t, std::size_t tgt, std::size_t ctx, std::size_t step, std::size_t feat)
{
  const std::size_t T = static_cast<std::size_t>(t.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(t.obj_trajs_shape[3]);
  return ((tgt * kContextCap + ctx) * T + step) * F + feat;
}

std::size_t maskIdx(const MtrInputTensors & t, std::size_t tgt, std::size_t ctx, std::size_t step)
{
  const std::size_t T = static_cast<std::size_t>(t.obj_trajs_shape[2]);
  return (tgt * kContextCap + ctx) * T + step;
}

// Number of context rows present at the current step for a target slot. In a single-frame build
// this equals (ego + count of retained-and-visible objects), so it exposes history retention.
int countPresentRows(const MtrInputTensors & t, std::size_t target_slot = 0)
{
  const std::size_t T = static_cast<std::size_t>(t.obj_trajs_shape[2]);
  int n = 0;
  for (std::size_t c = 0; c < kContextCap; ++c) {
    if (t.obj_trajs_mask[maskIdx(t, target_slot, c, T - 1)] == 1U) {
      ++n;
    }
  }
  return n;
}

// Number of context rows with at least one present slot (any timestep). Counts retained tracks,
// including ones no longer visible at the current step, so it exposes history retention/pruning.
int countActiveRows(const MtrInputTensors & t, std::size_t target_slot = 0)
{
  const std::size_t T = static_cast<std::size_t>(t.obj_trajs_shape[2]);
  int n = 0;
  for (std::size_t c = 0; c < kContextCap; ++c) {
    for (std::size_t s = 0; s < T; ++s) {
      if (t.obj_trajs_mask[maskIdx(t, target_slot, c, s)] == 1U) {
        ++n;
        break;
      }
    }
  }
  return n;
}

TEST(SceneBuilder, EmptyFrameProducesInvalidTensors)
{
  SceneBuilder builder{MtrConfig{}};
  MtrFrameContext frame;
  auto tensors = builder.build(frame);
  EXPECT_FALSE(tensors.valid);
  EXPECT_TRUE(tensors.sidecar.targets.empty());
}

// ---- History + validation -------------------------------------------------------------------

TEST(SceneBuilder, ValidSupportedDetectionCreatesRetainedHistory)
{
  SceneBuilder builder{MtrConfig{}};
  builder.addFrame(makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)}));

  auto tensors = builder.build(
    makeFrame(makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)}), makePose(0, 0, 0)));

  EXPECT_TRUE(tensors.valid);
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "veh_1");
}

TEST(SceneBuilder, EmptyIdDetectionIsSkipped)
{
  SceneBuilder builder{MtrConfig{}};
  const auto dets = makeArray(
    100.0, {makeDetection("", "vehicle", 5.0, 0.0, 0.0, 0.0), makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "veh_1");
  // Context = ego + the single valid vehicle = 2 rows; the empty-id detection was never retained.
  EXPECT_EQ(countPresentRows(tensors), 2);
}

TEST(SceneBuilder, UnsupportedClassIsSkipped)
{
  SceneBuilder builder{MtrConfig{}};
  const auto dets = makeArray(
    100.0,
    {makeDetection("obj_1", "traffic_cone", 5.0, 0.0, 0.0, 0.0),
     makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "veh_1");
  EXPECT_EQ(countPresentRows(tensors), 2);  // ego + veh_1 only
}

TEST(SceneBuilder, MalformedDimensionsAreSkipped)
{
  SceneBuilder builder{MtrConfig{}};
  const auto dets = makeArray(
    100.0,
    {makeDetection("bad", "vehicle", 5.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0.0, 2.0, 1.5),
     makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "veh_1");
  EXPECT_EQ(countPresentRows(tensors), 2);  // ego + veh_1 only
}

TEST(SceneBuilder, NaNPositionIsSkipped)
{
  SceneBuilder builder{MtrConfig{}};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto dets = makeArray(
    100.0,
    {makeDetection("bad", "vehicle", nan, 0.0, 0.0, 0.0), makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "veh_1");
  EXPECT_EQ(countPresentRows(tensors), 2);  // ego + veh_1 only
}

TEST(SceneBuilder, NonFiniteVelocityRetainsSampleWithZeroVelocity)
{
  // A bad velocity hypothesis must not erase a valid position observation.
  SceneBuilder builder{MtrConfig{}};
  const double inf = std::numeric_limits<double>::infinity();
  const auto dets = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0, true, inf, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));

  EXPECT_TRUE(tensors.valid);
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  // Velocity feature (rel vx, offset F-4) for the target's own current step must be zero.
  const int row = tensors.sidecar.targets[0].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, F - 4)], 0.0F);
}

// ---- Resampling + target selection ----------------------------------------------------------

TEST(SceneBuilder, MissingEgoReturnsInvalid)
{
  SceneBuilder builder{MtrConfig{}};
  const auto dets = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  MtrFrameContext frame;
  frame.detections = dets;
  frame.has_ego = false;
  auto tensors = builder.build(frame);
  EXPECT_FALSE(tensors.valid);
}

TEST(SceneBuilder, ResamplingMasksMissingSamples)
{
  // Single frame => only the current slot (index T-1) is present; older slots masked.
  SceneBuilder builder{MtrConfig{}};
  const auto dets = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(dets);
  auto tensors = builder.build(makeFrame(dets, makePose(0, 0, 0)));

  ASSERT_TRUE(tensors.valid);
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const int row = tensors.sidecar.targets[0].track_index;
  EXPECT_EQ(tensors.obj_trajs_mask[maskIdx(tensors, 0, static_cast<std::size_t>(row), T - 1)], 1U);
  EXPECT_EQ(tensors.obj_trajs_mask[maskIdx(tensors, 0, static_cast<std::size_t>(row), 0)], 0U);
}

TEST(SceneBuilder, ExactTieResamplingPrefersLaterSample)
{
  // Two stored samples equidistant from a slot must resolve to the later timestamp. Uses a 2 Hz /
  // 3-step config so all slot times (99.0, 99.5, 100.0) and sample times are exact doubles.
  MtrConfig cfg;
  cfg.history_steps = 3;
  cfg.history_rate_hz = 2.0;
  SceneBuilder builder{cfg};

  // Slot i=1 is t=99.5; samples at 99.25 and 99.75 are both 0.25 s away (== tolerance).
  builder.addFrame(makeArray(99.25, {makeDetection("veh_1", "vehicle", 10.0, 1.0, 0.0, 0.0)}));
  builder.addFrame(makeArray(99.75, {makeDetection("veh_1", "vehicle", 10.0, 2.0, 0.0, 0.0)}));
  const auto current = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(current);
  auto tensors = builder.build(makeFrame(current, makePose(0, 0, 0)));

  ASSERT_TRUE(tensors.valid);
  const int row = tensors.sidecar.targets[0].track_index;
  // Target heading 0 and center y=0, so slot-1 relative y equals the chosen sample's y.
  // The later sample (99.75, y=2.0) must win the tie, not the earlier (99.25, y=1.0).
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), 1, 1)], 2.0F);
}

TEST(SceneBuilder, DisappearedObjectRemainsInContextUntilPruned)
{
  SceneBuilder builder{MtrConfig{}};
  builder.addFrame(makeArray(
    100.0,
    {makeDetection("stay", "vehicle", 10.0, 0.0, 0.0, 0.0), makeDetection("gone", "vehicle", 12.0, 0.0, 0.0, 0.0)}));

  // 0.3 s later, "gone" is no longer visible but still inside the history window: ego + stay + gone.
  const auto near_frame = makeArray(100.3, {makeDetection("stay", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  auto retained = builder.build(makeFrame(near_frame, makePose(0, 0, 0)));
  ASSERT_TRUE(retained.valid);
  EXPECT_EQ(countActiveRows(retained), 3);

  // 1.5 s past "gone"'s last sample it falls out of the window and is pruned: only ego + stay left.
  const auto far_frame = makeArray(101.5, {makeDetection("stay", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(far_frame);
  auto pruned = builder.build(makeFrame(far_frame, makePose(0, 0, 0)));
  ASSERT_TRUE(pruned.valid);
  EXPECT_EQ(countActiveRows(pruned), 2);
}

TEST(SceneBuilder, TargetLimitIsRespected)
{
  SceneBuilder builder{MtrConfig{}};
  std::vector<vision_msgs::msg::Detection3D> dets;
  for (int i = 0; i < 12; ++i) {
    dets.push_back(makeDetection("veh_" + std::to_string(i), "vehicle", 5.0 + i, 0.0, 0.0, 0.0));
  }
  const auto arr = makeArray(100.0, dets);
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  EXPECT_EQ(tensors.sidecar.targets.size(), kTargetCap);
  EXPECT_EQ(tensors.track_index_to_predict.size(), kTargetCap);
}

TEST(SceneBuilder, ClosestForwardTargetSelectedFirst)
{
  SceneBuilder builder{MtrConfig{}};
  // near_front closest & forward; behind is closer in |x| but behind ego -> lower priority.
  const auto arr = makeArray(
    100.0,
    {makeDetection("behind", "vehicle", -5.0, 0.0, 0.0, 0.0),
     makeDetection("near_front", "vehicle", 8.0, 0.0, 0.0, 0.0),
     makeDetection("far_front", "vehicle", 30.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_GE(tensors.sidecar.targets.size(), 3U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "near_front");
  EXPECT_EQ(tensors.sidecar.targets[1].detection_id, "far_front");
  EXPECT_EQ(tensors.sidecar.targets[2].detection_id, "behind");
}

TEST(SceneBuilder, DuplicateDetectionIdsTakeOneTargetSlot)
{
  // Two detections sharing an id must not occupy two target slots.
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(
    100.0,
    {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0), makeDetection("veh_1", "vehicle", 11.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  int count = 0;
  for (const auto & target : tensors.sidecar.targets) {
    if (target.detection_id == "veh_1") {
      ++count;
    }
  }
  EXPECT_EQ(count, 1);
}

TEST(SceneBuilder, TrackIndexPointsToContextRowAndSidecarMatchesSource)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 12.0, 3.0, 0.0, 0.5)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  const auto & tgt = tensors.sidecar.targets[0];
  EXPECT_EQ(tensors.track_index_to_predict[0], static_cast<int64_t>(tgt.track_index));
  EXPECT_DOUBLE_EQ(tgt.center_x, 12.0);
  EXPECT_DOUBLE_EQ(tgt.center_y, 3.0);
  EXPECT_NEAR(tgt.center_heading, 0.5, 1e-9);
}

// ---- Object tensor packing ------------------------------------------------------------------

TEST(SceneBuilder, TensorShapesMatchFrozenContract)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_TRUE(tensors.valid);
  EXPECT_EQ(tensors.obj_trajs_shape, (std::vector<int64_t>{8, 128, 11, 29}));
  EXPECT_EQ(tensors.map_polylines_shape, (std::vector<int64_t>{8, 768, 20, 9}));
  EXPECT_EQ(tensors.obj_trajs.size(), 8U * 128U * 11U * 29U);
  EXPECT_EQ(tensors.obj_trajs_mask.size(), 8U * 128U * 11U);
  EXPECT_EQ(tensors.obj_trajs_last_pos.size(), 8U * 128U * 3U);
  EXPECT_EQ(tensors.track_index_to_predict.size(), 8U);
  EXPECT_EQ(tensors.center_type_ids.size(), 8U);
}

TEST(SceneBuilder, UnusedTargetSlotsUseZeroSentinels)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  for (std::size_t t = 1; t < kTargetCap; ++t) {
    EXPECT_EQ(tensors.track_index_to_predict[t], 0);  // NOT -1
    EXPECT_EQ(tensors.center_type_ids[t], 0);
  }
}

TEST(SceneBuilder, TargetCenteredTransformHeadingZero)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(
    100.0, {makeDetection("A", "vehicle", 10.0, 0.0, 0.0, 0.0), makeDetection("B", "vehicle", 13.0, 2.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  // Target A is closest -> slot 0; B is a context row in A's frame.
  ASSERT_EQ(tensors.sidecar.targets[0].detection_id, "A");
  // Find B's context row via its own target slot's track index is not it; instead locate the row
  // whose current-step relative pos matches. Easiest: B is target slot 1, its track_index is B's row.
  const int b_row = tensors.sidecar.targets[1].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, 0)], 3.0F);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, 1)], 2.0F);
}

TEST(SceneBuilder, TargetCenteredTransformHeadingHalfPi)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(
    100.0,
    {makeDetection("A", "vehicle", 10.0, 0.0, 0.0, kPi / 2.0), makeDetection("B", "vehicle", 13.0, 2.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets[0].detection_id, "A");
  const int b_row = tensors.sidecar.targets[1].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  // px=3, py=2, heading pi/2 => rx = py = 2, ry = -px = -3.
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, 0)], 2.0F, 1e-4);
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, 1)], -3.0F, 1e-4);
}

TEST(SceneBuilder, HeadingNormalizationWrapsToPi)
{
  SceneBuilder builder{MtrConfig{}};
  // Target heading = -3.0 rad, object B heading = 3.0 rad. rel = wrap(6.0) = 6.0 - 2pi ~= -0.2832.
  const auto arr = makeArray(
    100.0, {makeDetection("A", "vehicle", 10.0, 0.0, 0.0, -3.0), makeDetection("B", "vehicle", 13.0, 0.0, 0.0, 3.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets[0].detection_id, "A");
  const int b_row = tensors.sidecar.targets[1].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  const double expected = 6.0 - 2.0 * kPi;
  const std::size_t sin_off = F - 6;  // 12 + T
  const std::size_t cos_off = F - 5;  // 13 + T
  EXPECT_NEAR(
    tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, sin_off)], std::sin(expected), 1e-4);
  EXPECT_NEAR(
    tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, cos_off)], std::cos(expected), 1e-4);
}

TEST(SceneBuilder, VelocityIsRotatedIntoTargetFrame)
{
  SceneBuilder builder{MtrConfig{}};
  // B moves +x in scene; target A heading pi/2 => rel v = (vy_scene rotated): rvx = vy=0? compute.
  // v=(2,0). heading pi/2: rvx = vx*cos+vy*sin = 0; rvy = -vx*sin+vy*cos = -2.
  const auto arr = makeArray(
    100.0,
    {makeDetection("A", "vehicle", 10.0, 0.0, 0.0, kPi / 2.0),
     makeDetection("B", "vehicle", 13.0, 2.0, 0.0, 0.0, true, 2.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets[0].detection_id, "A");
  const int b_row = tensors.sidecar.targets[1].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  const std::size_t vx_off = F - 4;  // 14 + T
  const std::size_t vy_off = F - 3;  // 15 + T
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, vx_off)], 0.0F, 1e-4);
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(b_row), T - 1, vy_off)], -2.0F, 1e-4);
}

TEST(SceneBuilder, AccelerationZeroWhenSamplesMissing)
{
  // Single frame -> only current slot present -> no adjacent pair -> acceleration zero.
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0, true, 5.0, 1.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  const int row = tensors.sidecar.targets[0].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  const std::size_t ax_off = F - 2;  // 16 + T
  const std::size_t ay_off = F - 1;  // 17 + T
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, ax_off)], 0.0F);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, ay_off)], 0.0F);
}

TEST(SceneBuilder, AccelerationIsFiniteDifferencedAcrossFrames)
{
  // Velocity 0 -> 1 over 0.1 s (10 Hz) => target-frame ax = (1 - 0) * 10 = 10.
  SceneBuilder builder{MtrConfig{}};
  builder.addFrame(makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0, true, 0.0, 0.0, 0.0)}));
  const auto arr2 = makeArray(100.1, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0, true, 1.0, 0.0, 0.0)});
  builder.addFrame(arr2);
  auto tensors = builder.build(makeFrame(arr2, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  const int row = tensors.sidecar.targets[0].track_index;
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  // Current and previous slots both present -> nonzero acceleration.
  EXPECT_EQ(tensors.obj_trajs_mask[maskIdx(tensors, 0, static_cast<std::size_t>(row), T - 1)], 1U);
  EXPECT_EQ(tensors.obj_trajs_mask[maskIdx(tensors, 0, static_cast<std::size_t>(row), T - 2)], 1U);
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, F - 2)], 10.0F, 1e-3);
}

TEST(SceneBuilder, EgoVelocityIsFiniteDifferencedAcrossBuilds)
{
  // Ego moves (0,0)->(1,0) over 0.1 s across two builds => ego speed 10 m/s along +x.
  SceneBuilder builder{MtrConfig{}};
  const auto a1 = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(a1);
  builder.build(makeFrame(a1, makePose(0.0, 0.0, 0.0)));

  const auto a2 = makeArray(100.1, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(a2);
  auto tensors = builder.build(makeFrame(a2, makePose(1.0, 0.0, 0.0)));

  ASSERT_TRUE(tensors.valid);
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  // Ego is at distance 0 => context row 0; target heading 0 => rel vx == scene vx == 10.
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, 10)], 1.0F);  // is_sdc row
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, F - 4)], 10.0F, 1e-3);
}

TEST(SceneBuilder, CenterTypeIdsMapCorrectly)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(
    100.0,
    {makeDetection("v", "car", 8.0, 0.0, 0.0, 0.0),
     makeDetection("p", "pedestrian", 9.0, 0.0, 0.0, 0.0),
     makeDetection("c", "bicycle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  // Targets ranked by distance: v(64) < p(81) < c(100).
  ASSERT_GE(tensors.sidecar.targets.size(), 3U);
  EXPECT_EQ(tensors.center_type_ids[0], 0);  // vehicle
  EXPECT_EQ(tensors.center_type_ids[1], 1);  // pedestrian
  EXPECT_EQ(tensors.center_type_ids[2], 2);  // cyclist
}

TEST(SceneBuilder, EgoContextRowIsSdc)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  // Ego is at distance 0 -> context row 0. is_sdc is one-hot offset 10.
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t is_sdc_off = 10;
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, is_sdc_off)], 1.0F);
  // The vehicle target row must NOT be is_sdc.
  const int row = tensors.sidecar.targets[0].track_index;
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, is_sdc_off)], 0.0F);
}

TEST(SceneBuilder, EgoRowCarriesConfigFootprint)
{
  // The ego (SDC) row must carry the configured box dims, not a zero-size box. Box dims are the
  // untransformed features [3..5].
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  const MtrConfig defaults{};
  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, 3)], static_cast<float>(defaults.ego_length));
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, 4)], static_cast<float>(defaults.ego_width));
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, 5)], static_cast<float>(defaults.ego_height));
}

TEST(SceneBuilder, EgoVelocityFromOdomRotatedToScene)
{
  // Odom twist is body frame. With ego heading pi/2, a body-forward velocity (2, 0) rotates to
  // scene (0, 2). The target has heading 0, so its frame equals the scene frame and the ego row's
  // relative velocity features (F-4 = vx, F-3 = vy) read the scene-frame velocity directly.
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto frame = makeFrame(arr, makePose(0, 0, kPi / 2.0));
  frame.ego_velocity.linear.x = 2.0;
  frame.has_ego_velocity = true;
  auto tensors = builder.build(frame);

  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t F = static_cast<std::size_t>(tensors.obj_trajs_shape[3]);
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, F - 4)], 0.0F, 1e-4);
  EXPECT_NEAR(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, F - 3)], 2.0F, 1e-4);
}

TEST(SceneBuilder, IsCenterObjectSetForTargetRowOnly)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  const std::size_t T = static_cast<std::size_t>(tensors.obj_trajs_shape[2]);
  const std::size_t is_center_off = 9;
  const int row = tensors.sidecar.targets[0].track_index;
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, static_cast<std::size_t>(row), T - 1, is_center_off)], 1.0F);
  EXPECT_FLOAT_EQ(tensors.obj_trajs[objIdx(tensors, 0, 0, T - 1, is_center_off)], 0.0F);  // ego row
}

TEST(SceneBuilder, ObjTrajsLastPosIsTargetCentered)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(
    100.0, {makeDetection("A", "vehicle", 10.0, 0.0, 0.0, 0.0), makeDetection("B", "vehicle", 13.0, 2.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_EQ(tensors.sidecar.targets[0].detection_id, "A");
  const int b_row = tensors.sidecar.targets[1].track_index;
  const std::size_t pos_idx = (0U * kContextCap + static_cast<std::size_t>(b_row)) * 3U;
  EXPECT_FLOAT_EQ(tensors.obj_trajs_last_pos[pos_idx + 0], 3.0F);
  EXPECT_FLOAT_EQ(tensors.obj_trajs_last_pos[pos_idx + 1], 2.0F);
}

TEST(SceneBuilder, DeterministicAcrossRepeatedBuilds)
{
  MtrConfig cfg;
  SceneBuilder a{cfg};
  SceneBuilder b{cfg};
  const auto arr = makeArray(
    100.0,
    {makeDetection("A", "vehicle", 10.0, 1.0, 0.0, 0.2), makeDetection("B", "pedestrian", 12.0, -1.0, 0.0, 0.0)});
  a.addFrame(arr);
  b.addFrame(arr);
  auto ta = a.build(makeFrame(arr, makePose(0, 0, 0)));
  auto tb = b.build(makeFrame(arr, makePose(0, 0, 0)));

  EXPECT_EQ(ta.obj_trajs, tb.obj_trajs);
  EXPECT_EQ(ta.obj_trajs_mask, tb.obj_trajs_mask);
  EXPECT_EQ(ta.track_index_to_predict, tb.track_index_to_predict);
}

// ---- Map packing ----------------------------------------------------------------------------

TEST(SceneBuilder, EmptyMapStillValid)
{
  // An object-only scene stays valid with zeroed / masked map buffers.
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  auto tensors = builder.build(makeFrame(arr, makePose(0, 0, 0)));

  ASSERT_TRUE(tensors.valid);
  EXPECT_EQ(tensors.map_polylines.size(), 8U * 768U * 20U * 9U);
  EXPECT_EQ(tensors.map_polylines_mask.size(), 8U * 768U * 20U);
  EXPECT_EQ(tensors.map_polylines_center.size(), 8U * 768U * 3U);
  EXPECT_TRUE(
    std::all_of(tensors.map_polylines.begin(), tensors.map_polylines.end(), [](float v) { return v == 0.0F; }));
  EXPECT_TRUE(std::all_of(
    tensors.map_polylines_mask.begin(), tensors.map_polylines_mask.end(), [](uint8_t m) { return m == 0U; }));
}

TEST(SceneBuilder, StraightCenterlineProducesExpectedPointFeatures)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  // Straight centerline along +x, 5 points; single target at (10,0) heading 0.
  const auto lanelet = makeLanelet({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}});
  auto tensors = builder.build(makeFrameWithMap(arr, makePose(0, 0, 0), {lanelet}));

  ASSERT_TRUE(tensors.valid);
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  // Target-centered x = scene x - 10; single chunk lands in poly row 0.
  EXPECT_FLOAT_EQ(tensors.map_polylines[mapIdx(0, 0, 0, 0)], -10.0F);  // point 0 rx
  EXPECT_FLOAT_EQ(tensors.map_polylines[mapIdx(0, 0, 1, 0)], -9.0F);  // point 1 rx
  // Direction of point 1 = normalized (+x): dir_x=1, dir_y=0.
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 1, 3)], 1.0F, 1e-5);
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 1, 4)], 0.0F, 1e-5);
  // global_type constant.
  EXPECT_FLOAT_EQ(tensors.map_polylines[mapIdx(0, 0, 1, 6)], 0.0F);
  // pre_x of point 1 == point 0 rx == -10; point 0 pre copies itself.
  EXPECT_FLOAT_EQ(tensors.map_polylines[mapIdx(0, 0, 1, 7)], -10.0F);
  EXPECT_FLOAT_EQ(tensors.map_polylines[mapIdx(0, 0, 0, 7)], -10.0F);
}

TEST(SceneBuilder, MapMasksDistinguishPaddedPoints)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  const auto lanelet = makeLanelet({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}});
  auto tensors = builder.build(makeFrameWithMap(arr, makePose(0, 0, 0), {lanelet}));

  ASSERT_TRUE(tensors.valid);
  for (std::size_t j = 0; j < 5; ++j) {
    EXPECT_EQ(tensors.map_polylines_mask[mapMaskIdx(0, 0, j)], 1U) << "point " << j;
  }
  for (std::size_t j = 5; j < kNumPointsPerPolyline; ++j) {
    EXPECT_EQ(tensors.map_polylines_mask[mapMaskIdx(0, 0, j)], 0U) << "padded point " << j;
  }
}

TEST(SceneBuilder, MapCenterFromValidPointsOnly)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, 0.0)});
  builder.addFrame(arr);
  const auto lanelet = makeLanelet({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}});
  auto tensors = builder.build(makeFrameWithMap(arr, makePose(0, 0, 0), {lanelet}));

  ASSERT_TRUE(tensors.valid);
  // Mean of target-centered x over the 5 real points = mean(-10..-6) = -8; y = 0. Padded points
  // must not dilute the mean.
  const std::size_t center_idx = (0U * kNumSrcPolylines + 0U) * 3U;
  EXPECT_FLOAT_EQ(tensors.map_polylines_center[center_idx + 0], -8.0F);
  EXPECT_FLOAT_EQ(tensors.map_polylines_center[center_idx + 1], 0.0F);
}

TEST(SceneBuilder, MapTargetCenteredTransformHeadingHalfPi)
{
  SceneBuilder builder{MtrConfig{}};
  const auto arr = makeArray(100.0, {makeDetection("veh_1", "vehicle", 10.0, 0.0, 0.0, kPi / 2.0)});
  builder.addFrame(arr);
  // Point 0 at scene (13,2): px=3, py=2, heading pi/2 => rx=py=2, ry=-px=-3.
  // Point 1 at (13,3): scene direction (0,1) rotated by -pi/2 => (1,0).
  const auto lanelet = makeLanelet({{13, 2, 0}, {13, 3, 0}});
  auto tensors = builder.build(makeFrameWithMap(arr, makePose(0, 0, 0), {lanelet}));

  ASSERT_TRUE(tensors.valid);
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 0, 0)], 2.0F, 1e-4);
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 0, 1)], -3.0F, 1e-4);
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 1, 3)], 1.0F, 1e-4);
  EXPECT_NEAR(tensors.map_polylines[mapIdx(0, 0, 1, 4)], 0.0F, 1e-4);
}

TEST(SceneBuilder, UsesConfiguredEgoDimensionsAndRotatesBodyVelocityToMap)
{
  MtrConfig cfg;
  cfg.history_steps = 1;
  cfg.selected_target_agent_limit = 1;
  cfg.ego_length = 4.0936;
  cfg.ego_width = 1.3579;
  cfg.ego_height = 1.4090;
  SceneBuilder builder(cfg);

  vision_msgs::msg::Detection3DArray detections;
  detections.header.frame_id = "map";
  detections.header.stamp.sec = 10;
  vision_msgs::msg::Detection3D target;
  target.id = "target";
  target.bbox.center.position.x = 5.0;
  target.bbox.center.orientation.w = 1.0;
  target.bbox.size.x = 4.0;
  target.bbox.size.y = 2.0;
  target.bbox.size.z = 1.5;
  vision_msgs::msg::ObjectHypothesisWithPose classification;
  classification.hypothesis.class_id = "vehicle";
  classification.hypothesis.score = 1.0;
  target.results.push_back(classification);
  detections.detections.push_back(target);
  builder.addFrame(detections);

  MtrFrameContext frame;
  frame.detections = detections;
  frame.ego_pose.header.frame_id = "map";
  frame.ego_pose.pose.orientation.z = std::sqrt(0.5);
  frame.ego_pose.pose.orientation.w = std::sqrt(0.5);
  frame.ego_velocity.linear.x = 2.0;
  frame.has_ego = true;
  frame.has_ego_velocity = true;

  const auto tensors = builder.build(frame);
  ASSERT_TRUE(tensors.valid);
  ASSERT_GE(tensors.obj_trajs.size(), 19U);
  // Ego is context row 0. Its dimensions occupy features [3..5]. A +2 m/s
  // body-X velocity at 90-degree ego yaw becomes map velocity (0,+2), and the
  // target heading is zero, so packed relative velocity is also (0,+2).
  EXPECT_NEAR(tensors.obj_trajs[3], cfg.ego_length, 1.0e-5);
  EXPECT_NEAR(tensors.obj_trajs[4], cfg.ego_width, 1.0e-5);
  EXPECT_NEAR(tensors.obj_trajs[5], cfg.ego_height, 1.0e-5);
  EXPECT_NEAR(tensors.obj_trajs[15], 0.0, 1.0e-5);
  EXPECT_NEAR(tensors.obj_trajs[16], 2.0, 1.0e-5);
}

}  // namespace
}  // namespace prediction_ml
