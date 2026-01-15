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

#ifndef PREDICTION__TRACKING_PLACEHOLDER_HPP_
#define PREDICTION__TRACKING_PLACEHOLDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

/**
 * @brief Placeholder tracking node that publishes Detection3DArray messages.
 *
 * This node serves as a placeholder for the actual tracking implementation.
 * It publishes static Detection3DArray messages to /perception/detections_3D_tracked.
 */
class TrackingPlaceholder
{
public:
  TrackingPlaceholder() = default;
  ~TrackingPlaceholder() = default;
};

#endif  // PREDICTION__TRACKING_PLACEHOLDER_HPP_
