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

#ifndef BEHAVIOUR__UTILS__GEOMETRY_UTILS_HPP_
#define BEHAVIOUR__UTILS__GEOMETRY_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "lanelet_msgs/msg/way.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour::geometry
{
inline double distanceXY(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

inline double distanceXY(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return distanceXY(a.position, b.position);
}

inline const geometry_msgs::msg::Point & objectCenter(const world_model_msgs::msg::WorldObject & object)
{
  return object.detection.bbox.center.position;
}

inline double distanceXY(const world_model_msgs::msg::WorldObject & a, const world_model_msgs::msg::WorldObject & b)
{
  return distanceXY(objectCenter(a), objectCenter(b));
}

inline double pointToSegmentDistanceXY(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & segment_start,
  const geometry_msgs::msg::Point & segment_end)
{
  const double abx = segment_end.x - segment_start.x;
  const double aby = segment_end.y - segment_start.y;
  const double apx = point.x - segment_start.x;
  const double apy = point.y - segment_start.y;

  const double ab_len_sq = (abx * abx) + (aby * aby);
  if (ab_len_sq <= 1e-12) {
    return distanceXY(point, segment_start);
  }

  double t = ((apx * abx) + (apy * aby)) / ab_len_sq;
  t = std::clamp(t, 0.0, 1.0);

  geometry_msgs::msg::Point projection;
  projection.x = segment_start.x + (t * abx);
  projection.y = segment_start.y + (t * aby);
  projection.z = 0.0;
  return distanceXY(point, projection);
}

inline double pointToPolylineDistanceXY(
  const geometry_msgs::msg::Point & point, const std::vector<geometry_msgs::msg::Point> & polyline)
{
  if (polyline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polyline.size() == 1) {
    return distanceXY(point, polyline.front());
  }

  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    const double d = pointToSegmentDistanceXY(point, polyline[i - 1], polyline[i]);
    if (d < min_dist) {
      min_dist = d;
    }
  }
  return min_dist;
}

inline double pointToWayDistanceXY(const geometry_msgs::msg::Point & point, const lanelet_msgs::msg::Way & way)
{
  return pointToPolylineDistanceXY(point, way.points);
}

inline double objectToWayDistanceXY(
  const world_model_msgs::msg::WorldObject & object, const lanelet_msgs::msg::Way & way)
{
  return pointToWayDistanceXY(objectCenter(object), way);
}

inline std::optional<geometry_msgs::msg::Point> wayCenterPoint(const lanelet_msgs::msg::Way & way)
{
  if (way.points.empty()) {
    return std::nullopt;
  }

  if (way.points.size() == 1) {
    return way.points.front();
  }

  double total_length = 0.0;
  for (std::size_t i = 1; i < way.points.size(); ++i) {
    total_length += distanceXY(way.points[i - 1], way.points[i]);
  }

  if (total_length <= 1e-9) {
    return way.points[way.points.size() / 2];
  }

  const double target = 0.5 * total_length;
  double accumulated = 0.0;

  for (std::size_t i = 1; i < way.points.size(); ++i) {
    const auto & p0 = way.points[i - 1];
    const auto & p1 = way.points[i];
    const double segment_length = distanceXY(p0, p1);

    if (segment_length <= 1e-12) {
      continue;
    }

    if ((accumulated + segment_length) >= target) {
      const double t = (target - accumulated) / segment_length;
      geometry_msgs::msg::Point center;
      center.x = p0.x + ((p1.x - p0.x) * t);
      center.y = p0.y + ((p1.y - p0.y) * t);
      center.z = p0.z + ((p1.z - p0.z) * t);
      return center;
    }

    accumulated += segment_length;
  }

  return way.points.back();
}
}  // namespace behaviour::geometry

#endif  // BEHAVIOUR__UTILS__GEOMETRY_UTILS_HPP_
