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

#include "prediction_ml/mtr_scene_adapter.hpp"

namespace prediction_ml
{
TEST(MtrSceneAdapter, ConvertsLaneletGeometryWithoutPackingModelTensors)
{
  MtrSceneAdapter adapter;
  vision_msgs::msg::Detection3DArray detections;
  detections.header.frame_id = "map";
  detections.header.stamp.sec = 10;
  detections.detections.resize(1);
  detections.detections[0].id = "track-1";

  lanelet_msgs::msg::LaneletAhead ahead;
  ahead.lanelets.resize(1);
  ahead.lanelets[0].id = 7;
  ahead.lanelets[0].centerline.resize(2);
  ahead.lanelets[0].left_boundary.resize(2);
  ahead.lanelets[0].right_boundary.resize(2);

  const auto scene = adapter.build(detections, nullptr, &ahead);

  EXPECT_EQ(scene.header, detections.header);
  EXPECT_EQ(scene.detections.detections[0].id, "track-1");
  EXPECT_TRUE(scene.has_map);
  ASSERT_EQ(scene.map_polylines.size(), 3u);
  EXPECT_EQ(scene.map_polylines[0].semantic_type, deep_msgs::msg::MapPolyline::CENTERLINE);
  EXPECT_EQ(scene.map_polylines[1].semantic_type, deep_msgs::msg::MapPolyline::LEFT_BOUNDARY);
  EXPECT_EQ(scene.map_polylines[2].semantic_type, deep_msgs::msg::MapPolyline::RIGHT_BOUNDARY);
}

TEST(MtrSceneAdapter, GeneratesUniqueCorrelatedRequests)
{
  MtrSceneAdapter adapter;
  vision_msgs::msg::Detection3DArray detections;
  detections.header.stamp.sec = 4;
  const auto first = adapter.build(detections, nullptr, nullptr);
  const auto second = adapter.build(detections, nullptr, nullptr);
  EXPECT_NE(first.request_id, second.request_id);
  EXPECT_FALSE(first.has_ego_pose);
  EXPECT_FALSE(first.has_map);
}
}  // namespace prediction_ml
