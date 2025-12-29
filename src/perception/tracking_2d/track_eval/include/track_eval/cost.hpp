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

#ifndef COST_HPP
#define COST_HPP

#include <vector>

#include <vision_msgs/msg/detection2_d_array.hpp>

/**
 * @brief Calculates IoU cost between two bounding boxes.
 *
 * @param box_a, box_b 2D bounding boxes
 * @return double IoU cost = 1 - IoU
 */
double costIoU(const vision_msgs::msg::BoundingBox2D & box_a, const vision_msgs::msg::BoundingBox2D & box_b);

/**
 * @brief Computes the IoU cost matrix between ground truths and tracks.
 *
 * @param gts Ground truths
 * @param trks Tracked detections
 * @return std::vector<std::vector<double>> IoU cost matrix
 */
std::vector<std::vector<double>> costMtx(
  const std::vector<vision_msgs::msg::Detection2D> & gts, const std::vector<vision_msgs::msg::Detection2D> & trks);

#endif
