// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "prediction_ml/mtr_scene_adapter.hpp"

#include <string>
#include <utility>
#include <vector>

#include "deep_msgs/msg/map_polyline.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace prediction_ml
{
deep_msgs::msg::MtrScene MtrSceneAdapter::build(const vision_msgs::msg::Detection3DArray & detections,
                                                const geometry_msgs::msg::PoseStamped * ego_pose,
                                                const lanelet_msgs::msg::LaneletAhead * lanelets)
{
  deep_msgs::msg::MtrScene scene;
  scene.header = detections.header;
  scene.detections = detections;
  scene.request_id = std::to_string(detections.header.stamp.sec) + "-" +
                     std::to_string(detections.header.stamp.nanosec) + "-" + std::to_string(sequence_++);

  if (ego_pose != nullptr) {
    scene.ego_pose = *ego_pose;
    scene.has_ego_pose = true;
  }

  if (lanelets != nullptr) {
    const auto append = [&scene](const int64_t id, const uint8_t semantic_type,
                                 const std::vector<geometry_msgs::msg::Point> & points) {
      if (points.empty()) {
        return;
      }
      deep_msgs::msg::MapPolyline polyline;
      polyline.lanelet_id = id;
      polyline.semantic_type = semantic_type;
      polyline.points = points;
      scene.map_polylines.push_back(std::move(polyline));
    };
    for (const auto & lanelet : lanelets->lanelets) {
      append(lanelet.id, deep_msgs::msg::MapPolyline::CENTERLINE, lanelet.centerline);
      append(lanelet.id, deep_msgs::msg::MapPolyline::LEFT_BOUNDARY, lanelet.left_boundary);
      append(lanelet.id, deep_msgs::msg::MapPolyline::RIGHT_BOUNDARY, lanelet.right_boundary);
    }
    scene.has_map = !scene.map_polylines.empty();
  }

  return scene;
}
}  // namespace prediction_ml
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

@@
