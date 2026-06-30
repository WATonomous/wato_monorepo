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

#ifndef PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_
#define PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_

#include <cstdint>

#include "deep_msgs/msg/mtr_scene.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace prediction_ml
{
class MtrSceneAdapter
{
public:
  deep_msgs::msg::MtrScene build(
    const vision_msgs::msg::Detection3DArray & detections,
    const geometry_msgs::msg::PoseStamped * ego_pose,
    const lanelet_msgs::msg::LaneletAhead * lanelets);

private:
  uint64_t sequence_{0};
};
}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_
