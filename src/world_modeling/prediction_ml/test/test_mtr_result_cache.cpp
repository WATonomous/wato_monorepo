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

#include <limits>
#include <vector>

#include "prediction_ml/mtr_result_cache.hpp"

namespace prediction_ml
{
namespace
{
deep_msgs::msg::MtrScene request()
{
  deep_msgs::msg::MtrScene scene;
  scene.header.frame_id = "map";
  scene.header.stamp.sec = 10;
  scene.request_id = "req-1";
  scene.detections.detections.resize(1);
  scene.detections.detections[0].id = "track-1";
  return scene;
}

deep_msgs::msg::MtrPredictionArray validResult()
{
  deep_msgs::msg::MtrPredictionArray result;
  result.header.frame_id = "map";
  result.header.stamp.sec = 10;
  result.request_id = "req-1";
  result.objects.resize(1);
  result.objects[0].track_id = "track-1";
  result.objects[0].trajectories.resize(1);
  result.objects[0].trajectories[0].confidence = 0.8F;
  result.objects[0].trajectories[0].poses.resize(1);
  result.objects[0].trajectories[0].poses[0].header.frame_id = "map";
  result.objects[0].trajectories[0].poses[0].pose.orientation.w = 1.0;
  return result;
}
}  // namespace

TEST(MtrResultCache, ReplacesFallbackOnlyForFreshCorrelatedResult)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());
  EXPECT_TRUE(cache.accept(validResult(), 10.2));
  std::vector<world_model_msgs::msg::WorldObject> fallback(1);
  fallback[0].detection.id = "track-1";
  const auto output = cache.select(fallback, 10.2);
  ASSERT_EQ(output[0].predictions.size(), 1u);
  EXPECT_FLOAT_EQ(output[0].predictions[0].conf, 0.8F);
}

TEST(MtrResultCache, RejectsUnknownStaleAndMalformedResults)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());
  auto unknown = validResult();
  unknown.request_id = "unknown";
  EXPECT_FALSE(cache.accept(unknown, 10.1));
  EXPECT_FALSE(cache.accept(validResult(), 10.6));
  auto malformed = validResult();
  malformed.objects[0].trajectories[0].poses.clear();
  EXPECT_FALSE(cache.accept(malformed, 10.1));
}

TEST(MtrResultCache, RejectsWrongFramesUnknownTracksAndNonFiniteValues)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());

  auto wrong_frame = validResult();
  wrong_frame.header.frame_id = "odom";
  EXPECT_FALSE(cache.accept(wrong_frame, 10.1));

  auto unknown_track = validResult();
  unknown_track.objects[0].track_id = "track-2";
  EXPECT_FALSE(cache.accept(unknown_track, 10.1));

  auto non_finite = validResult();
  non_finite.objects[0].trajectories[0].confidence = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(cache.accept(non_finite, 10.1));
}

TEST(MtrResultCache, EvictsPendingRequestsOutsideTheCorrelationWindow)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());

  auto newer_request = request();
  newer_request.header.stamp.sec = 11;
  newer_request.request_id = "req-2";
  cache.rememberRequest(newer_request);

  EXPECT_FALSE(cache.accept(validResult(), 10.1));
}
}  // namespace prediction_ml
