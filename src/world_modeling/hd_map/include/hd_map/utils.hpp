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

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "vision_msgs/msg/detection3_d.hpp"

namespace utils
{
lanelet::Polygon3d boundingBox3dToPolygon3d(const lanelet::BoundingBox3d & bbox);
lanelet::BoundingBox3d detection3dToLaneletBBox(const vision_msgs::msg::BoundingBox3D & detection3d_bbox);
}  // namespace utils
