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

#ifndef WORLD_MODEL__MAP_PROJECTOR_HPP_
#define WORLD_MODEL__MAP_PROJECTOR_HPP_

#include <lanelet2_io/Projection.h>

#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/transform.hpp"

namespace world_model
{

/** Geographic projection and the frame in which its output is expressed. */
struct MapConfig
{
  std::string projector_type{"utm"};
  std::string alignment_mode{"fixed"};
  std::string projected_frame{"utm"};
  double origin_lat{0.0};
  double origin_lon{0.0};
  double origin_alt{0.0};
};

/** Throws std::invalid_argument for unsupported modes or invalid origins. */
void validateMapConfig(const MapConfig & config);

/**
 * Projects OSM points into the ROS map frame before Lanelet2 builds its indices.
 *
 * Fixed UTM subtracts the configured geographic origin (including altitude).
 * TF UTM uses absolute easting/northing and applies projected_frame -> map.
 * Local Cartesian uses the configured ENU origin in either alignment mode.
 */
class MapProjector : public lanelet::Projector
{
public:
  explicit MapProjector(
    const MapConfig & config, const std::optional<geometry_msgs::msg::Transform> & projected_to_map = std::nullopt);

  lanelet::BasicPoint3d forward(const lanelet::GPSPoint & point) const override;
  lanelet::GPSPoint reverse(const lanelet::BasicPoint3d & point) const override;

private:
  std::unique_ptr<lanelet::Projector> projector_;
  Eigen::Isometry3d projected_to_map_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_to_projected_{Eigen::Isometry3d::Identity()};
};

}  // namespace world_model

#endif  // WORLD_MODEL__MAP_PROJECTOR_HPP_
