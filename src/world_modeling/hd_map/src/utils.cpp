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

#include "utils.hpp"

lanelet::Polygon3d utils::boundingBox3dToPolygon3d(const lanelet::BoundingBox3d & bbox)
{
  auto min = bbox.min();
  auto max = bbox.max();

  lanelet::Polygon3d polygon{
    lanelet::utils::getId(),
    {lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), min.z()),
     lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), min.z()),
     lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), min.z()),
     lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), min.z()),
     lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), max.z()),
     lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), max.z()),
     lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), max.z()),
     lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), max.z())}};

  return polygon;
}

lanelet::BoundingBox3d utils::detection3dToLaneletBBox(const vision_msgs::msg::BoundingBox3D & bbox)
{
  return lanelet::BoundingBox3d(
    lanelet::BasicPoint3d(
      bbox.center.position.x - bbox.size.x / 2,
      bbox.center.position.y - bbox.size.y / 2,
      bbox.center.position.z - bbox.size.z / 2),
    lanelet::BasicPoint3d(
      bbox.center.position.x + bbox.size.x / 2,
      bbox.center.position.y + bbox.size.y / 2,
      bbox.center.position.z + bbox.size.z / 2));
}
