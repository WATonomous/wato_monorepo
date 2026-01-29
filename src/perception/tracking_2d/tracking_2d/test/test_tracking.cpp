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
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wato_test/wato_test.hpp>

#include "tracking_2d/tracking_2d.hpp"

// using wato::perception::patchworkpp::GroundRemovalCore;
// using wato::perception::patchworkpp::GroundRemovalNode;

namespace
{

struct RclcppGuard
{
  RclcppGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// [cx, cy, cz, yaw, l, w, h, score]
sensor_msgs::msg::PointCloud2 vectsToDetection3DArray(const std::vector<std::array<float, 8>> & det_info, const std::vector<std::string> & class_ids)
{
  vision_msgs::msg::Detection3DArray dets;
  cloud.header.frame_id = "test_frame";
  cloud.header.stamp = rclcpp::Clock().now();

  dets.detections.resize(det_info.size());

  for (int i = 0; i < det_info.size(); ++i) {
    const auto & d = det_info[i];
    auto & det = dets.detections[i];
    det.bbox.center.position.x = d[0];
    det.bbox.center.position.y = d[1];
    det.bbox.center.position.z = d[2];
    det.bbox.center.orientation = tf2::toMsg(
        tf2::Quaternion(0, 0, std::sin(d[3]/2), std::cos(d[3]/2))
    );
    det.bbox.size.x = d[4];
    det.bbox.size.y = d[5];
    det.bbox.size.z = d[6];
    det.results.resize(1);
    det.results[0].hypothesis.class_id = class_ids[i];
    det.results[0].hypothesis.score = d[7];
  }

  return dets;
}

template <typename Predicate>
bool waitForCondition(Predicate && condition, std::chrono::milliseconds timeout, std::chrono::milliseconds interval)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return condition();
}

template <typename MessageType>
std::optional<MessageType> waitForFuture(std::future<MessageType> future, std::chrono::milliseconds timeout)
{
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  return std::nullopt;
}

}  // namespace

// =============================================================================
// TEST 1: Detection3DArray to Object conversion
// WHY: This conversion is required for ByteTrack to function.
// =============================================================================
TEST_CASE("Detection3DArray to Object conversion works", "[core][fast]")
{
  std::vector<std::array<float, 8>> det_info = {
    {1.0f, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f, 6.0f, 0.8f},
    {9.3f, -2.4f, 0.1f, 3.14159f, 1.0f, 1.0f, 1.0f, 0.5f}
  };
  std::vector<std::string> class_ids = {"car", "bus"};
  auto msg = std::make_shared<vision_msgs::msg::Detection3DArray>(vectToDetection3DArray(det_info));

  std::vector<byte_track::Object> objs = tracking_2d::detsToObjects(msg);

  REQUIRE(objs.size() == 2);
  REQUIRE_THAT(objs[0].rect.xyzolwh[0], Catch::Matchers::WithinRel(1.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[1], Catch::Matchers::WithinRel(2.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[2], Catch::Matchers::WithinRel(3.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[3], Catch::Matchers::WithinRel(0.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[4], Catch::Matchers::WithinRel(4.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[5], Catch::Matchers::WithinRel(5.0f));
  REQUIRE_THAT(objs[0].rect.xyzolwh[6], Catch::Matchers::WithinRel(6.0f));
  REQUIRE_THAT(objs[0].label, Catch::Matchers::WithinRel(0));
  REQUIRE_THAT(objs[0].score, Catch::Matchers::WithinRel(0.8f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[0], Catch::Matchers::WithinRel(9.3f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[1], Catch::Matchers::WithinRel(-2.4f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[2], Catch::Matchers::WithinRel(0.1f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[3], Catch::Matchers::WithinRel(3.14159f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[4], Catch::Matchers::WithinRel(1.0f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[5], Catch::Matchers::WithinRel(1.0f));
  REQUIRE_THAT(objs[1].rect.xyzolwh[6], Catch::Matchers::WithinRel(1.0f));
  REQUIRE_THAT(objs[0].label, Catch::Matchers::WithinRel(4));
  REQUIRE_THAT(objs[0].score, Catch::Matchers::WithinRel(0.5f));
}
