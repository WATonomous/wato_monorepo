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

#include <array>
#include <chrono>
#include <cmath>
#include <future>
// #include <iostream>
#include <memory>
// #include <optional>
#include <string>
// #include <thread>
// #include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <wato_test/wato_test.hpp>

#include "tracking/tracking.hpp"

// [cx, cy, cz, yaw, l, w, h, score]
vision_msgs::msg::Detection3DArray arraysToDetection3DArray(const std::vector<std::array<float, 8>> & det_info, const std::vector<std::string> & class_ids)
{
  vision_msgs::msg::Detection3DArray dets;
  dets.header.frame_id = "base_link";
  dets.header.stamp = rclcpp::Clock().now();

  dets.detections.resize(det_info.size());

  for (size_t i = 0; i < det_info.size(); ++i) {
    const auto & d = det_info[i];
    auto & det = dets.detections[i];

    tf2::Quaternion q;
    q.setRPY(0, 0, d[3]);

    det.bbox.center.position.x = d[0];
    det.bbox.center.position.y = d[1];
    det.bbox.center.position.z = d[2];
    det.bbox.center.orientation = tf2::toMsg(q);
    det.bbox.size.x = d[4];
    det.bbox.size.y = d[5];
    det.bbox.size.z = d[6];
    det.results.resize(1);
    det.results[0].hypothesis.class_id = class_ids[i];
    det.results[0].hypothesis.score = d[7];
  }

  return dets;
}

std::vector<vision_msgs::msg::Detection3DArray> getTestInput(
  const std::vector<std::vector<std::array<float, 8>>> &det_info,
  const std::vector<std::vector<std::string>> &class_ids)
{
  std::vector<vision_msgs::msg::Detection3DArray> test_case;
  for (size_t i = 0; i < class_ids.size(); ++i) {
    test_case.push_back(arraysToDetection3DArray(det_info[i], class_ids[i]));
  }
  return test_case;
}

// [cx, cy, cz, yaw, l, w, h, score]
std::vector<byte_track::BYTETracker::STrackPtr> arraysToSTrackPtrs(const std::vector<std::array<float, 8>> & det_info, const std::vector<int> & class_ids)
{
  std::vector<byte_track::BYTETracker::STrackPtr> strks;

  strks.reserve(det_info.size());

  for (size_t i = 0; i < det_info.size(); ++i) {
    const auto & d = det_info[i];
    byte_track::Rect<float> rect(d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
    strks.push_back(
      std::make_shared<byte_track::STrack>(rect, d[7], class_ids[i])
    );
  }

  return strks;
}

float principleAngle(float a) {
  static const float PI_2 = 2.0f * std::acos(-1.0f);
  a = std::fmod(a, PI_2);
  if (a < 0) a += PI_2;
  return a;
}

// Test 1: Detection3DArray to Object conversion
TEST_CASE("Detection3DArray to Object conversion working", "[conv_1]")
{
  std::vector<std::array<float, 8>> det_info = {
    {1.0f, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f, 6.0f, 0.8f},
    {9.3f, -2.4f, 0.1f, 3.14159f, 1.0f, 1.0f, 1.0f, 0.5f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {-5.5f, -1.2f, -3.3f, 2.28f, 2.0f, 2.0f, 1.5f, 0.9f},
    {0.0f, -20.0f, 0.0f, 0.5f, 1.0f, 1.0f, 0.5f, 0.7f}
  };
  std::vector<std::string> class_ids = {"car", "bus", "truck", "bicycle", "pedestrian"};
  vision_msgs::msg::Detection3DArray msg = arraysToDetection3DArray(det_info, class_ids);

  std::vector<byte_track::Object> objs = tracking::detsToObjects(msg);

  SECTION("No detections missed") {
    REQUIRE(objs.size() == det_info.size());
  }

  SECTION("Required data matches") {
    for (size_t i = 0; i < objs.size(); ++i) {
      const auto &obj = objs[i];
      const auto &exp = det_info[i];
      for (size_t j = 0; j < 7; ++j) {
        if (j == 3) {
          REQUIRE_THAT(
            principleAngle(obj.rect.xyzolwh[j]),
            Catch::Matchers::WithinRel(principleAngle(exp[j]))
          );
        } else {
          REQUIRE_THAT(obj.rect.xyzolwh[j], Catch::Matchers::WithinRel(exp[j]));
        }
      }
      REQUIRE_THAT(obj.prob, Catch::Matchers::WithinRel(exp[7]));
      REQUIRE(obj.label == tracking::classLookup(class_ids[i]));
    }
  }
}

std::vector<vision_msgs::msg::Detection3DArray> runTrackingNode(
  const std::shared_ptr<wato::test::PublisherTestNode<vision_msgs::msg::Detection3DArray>> &test_pub,
  const std::shared_ptr<wato::test::SubscriberTestNode<vision_msgs::msg::Detection3DArray>> &test_sub,
  const std::vector<vision_msgs::msg::Detection3DArray> &test_case,
  const std::string & timeout_flag = "timeout")
{
  using namespace std::literals::chrono_literals;

  std::vector<vision_msgs::msg::Detection3DArray> results;

  for (size_t i = 0; i < test_case.size(); ++i) {
    auto future = test_sub->expect_next_message();
    test_pub->publish(test_case[i]);

    if (future.wait_for(1s) == std::future_status::ready) {
      results.push_back(future.get());
    } else {
      vision_msgs::msg::Detection3DArray dummy;
      dummy.header.frame_id = timeout_flag;
      results.push_back(dummy);
    }
  }
  return results;
}

// Test 2: STrack to Detection3DArray conversion
TEST_CASE("STrack to Detection3DArray conversion before publishing", "[conv_2]")
{
  std_msgs::msg::Header h;
  h.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  h.frame_id = "map";

  std::vector<std::array<float, 8>> det_info = {
    {1.0f, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f, 6.0f, 0.8f},
    {9.3f, -2.4f, 0.1f, 3.14159f, 1.0f, 1.0f, 1.0f, 0.5f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {-5.5f, -1.2f, -3.3f, 2.28f, 2.0f, 2.0f, 1.5f, 0.9f},
    {0.0f, -20.0f, 0.0f, 0.5f, 1.0f, 1.0f, 0.5f, 0.7f}
  };
  std::vector<int> class_ids = {0, 3, 2, 4, 1};
  auto strks = arraysToSTrackPtrs(det_info, class_ids);

  vision_msgs::msg::Detection3DArray trks = tracking::STracksToTracks(strks, h);

  SECTION("No tracks missed") {
    REQUIRE(trks.detections.size() == det_info.size());
  }

  SECTION("Header matches") {
    REQUIRE(trks.header.stamp == h.stamp);
    REQUIRE(trks.header.frame_id == h.frame_id);
  }

  SECTION("Required data matches") {
    for (size_t i = 0; i < trks.detections.size(); ++i) {
      const auto &trk = trks.detections[i];
      const auto &trk_box = trk.bbox;
      const auto &trk_pos = trk_box.center;
      const auto &exp = det_info[i];
      
      REQUIRE_THAT(trk_pos.position.x, Catch::Matchers::WithinRel(exp[0]));
      REQUIRE_THAT(trk_pos.position.y, Catch::Matchers::WithinRel(exp[1]));
      REQUIRE_THAT(trk_pos.position.z, Catch::Matchers::WithinRel(exp[2]));
      REQUIRE_THAT(
        principleAngle(tf2::getYaw(trk_pos.orientation)),
        Catch::Matchers::WithinRel(principleAngle(exp[3]))
      );
      REQUIRE_THAT(trk_box.size.x, Catch::Matchers::WithinRel(exp[4]));
      REQUIRE_THAT(trk_box.size.y, Catch::Matchers::WithinRel(exp[5]));
      REQUIRE_THAT(trk_box.size.z, Catch::Matchers::WithinRel(exp[6]));

      REQUIRE_THAT(trk.results[0].hypothesis.score, Catch::Matchers::WithinRel(exp[7]));
      REQUIRE(trk.results[0].hypothesis.class_id == tracking::reverseClassLookup(class_ids[i]));
      
      REQUIRE(trk.header.stamp == h.stamp);
      REQUIRE(trk.header.frame_id == h.frame_id);
    }
  }
}

// Test 3: Tracking node
TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Node tests", "[ros]") {
  std::string timeout_flag = "timeout";
  std::vector<std::vector<std::array<float, 8>>> vdet_info = {
    {{1.0f, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f, 6.0f, 0.8f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
    {{-1.0f, 3.0f, 3.1f, 0.2f, 4.05f, 4.9f, 6.1f, 0.77f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
    {{-2.0f, 5.0f, 3.0f, 0.4f, 4.02f, 5.01f, 6.0f, 0.82f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
    {{-2.0f, 7.0f, 3.1f, 0.5f, 4.0f, 5.02f, 6.7f, 0.81f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
    {{-1.0f, 9.0f, 3.2f, 0.5f, 4.06f, 5.03f, 6.0f, 0.82f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}}
  };
  std::vector<std::vector<std::string>> vclass_ids = {
    {"car", "car"},
    {"car", "car"},
    {"bus", "car"},
    {"car", "car"},
    {"car", "car"}
  };
  std::vector<vision_msgs::msg::Detection3DArray> msgs = getTestInput(vdet_info, vclass_ids);

  auto node = std::make_shared<tracking>();
  add_node(node);

  auto test_pub = std::make_shared<wato::test::PublisherTestNode<vision_msgs::msg::Detection3DArray>>(
    tracking::kDetectionsTopic
  );
  auto test_sub = std::make_shared<wato::test::SubscriberTestNode<vision_msgs::msg::Detection3DArray>>(
    tracking::kTracksTopic
  );

  add_node(test_pub);
  add_node(test_sub);
  start_spinning();

  // Node initialized
  REQUIRE_FALSE(!node);

  std::vector<vision_msgs::msg::Detection3DArray> trks;

  for (size_t i = 0; i < msgs.size(); ++i) {
    auto future = test_sub->expect_next_message();
    test_pub->publish(msgs[i]);

    if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready) {
      trks.push_back(future.get());
    } else {
      vision_msgs::msg::Detection3DArray dummy;
      dummy.header.frame_id = timeout_flag;
      trks.push_back(dummy);
    }
  }

  SECTION("Non-empty tracks; No timeouts; IDs are unique and stay constant") {
    for (size_t i = 0; i < trks.size(); ++i) {
      REQUIRE(!trks[i].detections.empty());
      REQUIRE(trks[i].header.frame_id != timeout_flag);
      if (trks[i].detections.size() >= 2) {
        REQUIRE(trks[i].detections[0].id != trks[i].detections[1].id);
        if (i > 0) {
          if (trks[i-1].detections.size() >= 2) {
            REQUIRE(trks[i].detections[0].id == trks[i-1].detections[0].id);
            REQUIRE(trks[i].detections[1].id == trks[i-1].detections[1].id);
          }
        }
      }
    }
  }
}
