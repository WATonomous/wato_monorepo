// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.

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
  deep_msgs::msg::MtrScene build(const vision_msgs::msg::Detection3DArray & detections,
                                 const geometry_msgs::msg::PoseStamped * ego_pose,
                                 const lanelet_msgs::msg::LaneletAhead * lanelets);

private:
  uint64_t sequence_{0};
};
}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_
