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

#include "world_model/map_projector.hpp"

#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace world_model
{

void validateMapConfig(const MapConfig & config)
{
  if (config.projector_type != "utm" && config.projector_type != "local_cartesian") {
    throw std::invalid_argument("projector_type must be 'utm' or 'local_cartesian'");
  }
  if (config.alignment_mode != "fixed" && config.alignment_mode != "tf") {
    throw std::invalid_argument("alignment_mode must be 'fixed' or 'tf'");
  }
  if (
    !std::isfinite(config.origin_lat) || std::abs(config.origin_lat) > 90.0 || !std::isfinite(config.origin_lon) ||
    std::abs(config.origin_lon) > 180.0 || !std::isfinite(config.origin_alt))
  {
    throw std::invalid_argument("Map origin must contain finite, valid latitude, longitude, and altitude");
  }
  if (config.alignment_mode == "tf" && config.projected_frame.empty()) {
    throw std::invalid_argument("projected_frame is required for alignment_mode=tf");
  }
}

MapProjector::MapProjector(
  const MapConfig & config, const std::optional<geometry_msgs::msg::Transform> & projected_to_map)
: lanelet::Projector(lanelet::Origin({config.origin_lat, config.origin_lon, config.origin_alt}))
{
  validateMapConfig(config);
  const bool use_tf = config.alignment_mode == "tf";
  if (use_tf != projected_to_map.has_value()) {
    throw std::invalid_argument("Provide a projected-to-map transform exactly when alignment_mode=tf");
  }

  if (config.projector_type == "utm") {
    // Eidos TF is defined on absolute UTM coordinates. Subtracting a geographic
    // origin here as well would apply the origin offset twice.
    projector_ = std::make_unique<lanelet::projection::UtmProjector>(origin(), !use_tf);
    if (!use_tf) {
      projected_to_map_.translation().z() = -config.origin_alt;
    }
  } else {
    projector_ = std::make_unique<lanelet::projection::LocalCartesianProjector>(origin());
  }

  if (use_tf) {
    const auto & t = projected_to_map->translation;
    const auto & r = projected_to_map->rotation;
    const Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);
    const Eigen::Vector3d translation(t.x, t.y, t.z);
    if (!translation.allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
      throw std::invalid_argument("Alignment transform must contain finite translation and a nonzero quaternion");
    }
    projected_to_map_.linear() = rotation.normalized().toRotationMatrix();
    projected_to_map_.translation() = translation;
  }
  map_to_projected_ = projected_to_map_.inverse();
}

lanelet::BasicPoint3d MapProjector::forward(const lanelet::GPSPoint & point) const
{
  return projected_to_map_ * projector_->forward(point);
}

lanelet::GPSPoint MapProjector::reverse(const lanelet::BasicPoint3d & point) const
{
  return projector_->reverse(map_to_projected_ * point);
}

}  // namespace world_model
