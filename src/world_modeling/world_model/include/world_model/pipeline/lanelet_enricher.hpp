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

#ifndef WORLD_MODEL__PIPELINE__LANELET_ENRICHER_HPP_
#define WORLD_MODEL__PIPELINE__LANELET_ENRICHER_HPP_

#include <cmath>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/lanelet_handler.hpp"
#include "world_model/types/entity_3d.hpp"

namespace world_model
{

/**
 * @brief Enriches entities with lanelet context using per-entity strategies.
 *
 * Vehicles (Car, Bicycle, Motorcycle) use tracked lane finding: the
 * entity's previous lanelet_id is passed as a BFS hint so that lane
 * assignment is stable across frames — the same pattern used by
 * LaneContextPublisher and LaneletAheadPublisher.
 *
 * TrafficLight is matched to the nearest traffic light regulatory element.
 * All other entity types receive no enrichment by default.
 */
class LaneletEnricher
{
public:
  LaneletEnricher(
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer,
    const std::string & map_frame,
    double route_priority_threshold_m = 10.0,
    double heading_search_radius_m = 15.0)
  : lanelet_(lanelet_handler)
  , tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , route_priority_threshold_m_(route_priority_threshold_m)
  , heading_search_radius_m_(heading_search_radius_m)
  {}

  /**
   * @brief Cache the transform from source_frame to map frame.
   *
   * Call this BEFORE entering the batch so that enrich() does no I/O
   * while the write mutex is held.
   */
  void updateTransform(const std::string & source_frame)
  {
    if (source_frame == map_frame_) {
      tf_valid_ = true;
      tf_is_identity_ = true;
      return;
    }
    try {
      cached_tf_ = tf_buffer_->lookupTransform(map_frame_, source_frame, tf2::TimePointZero);
      tf_valid_ = true;
      tf_is_identity_ = false;
    } catch (const tf2::TransformException &) {
      tf_valid_ = false;
    }
  }

  /**
   * @brief Enrich all entities in a map with lanelet context.
   *
   * Iterates each entity and dispatches to the type-specific enrich()
   * overload. Requires updateTransform() to have been called first.
   *
   * @tparam EntityT Entity class to enrich.
   * @param map Entity map to enrich in-place.
   */
  template <typename EntityT>
  void enrich(std::unordered_map<std::string, EntityT> & map)
  {
    if (!lanelet_->isMapLoaded() || !tf_valid_) {
      return;
    }

    for (auto & [id, entity] : map) {
      if (entity.empty()) {
        continue;
      }
      enrich(entity);
    }
  }

private:
  /// @brief No enrichment for Unknown entities.
  void enrich(Unknown & /*entity*/)
  {}

  /// @brief No enrichment for Human entities.
  void enrich(Human & /*entity*/)
  {}

  /**
   * @brief TrafficLight: match to nearest traffic light way and its parent reg elem.
   */
  void enrich(TrafficLight & entity)
  {
    auto map_pose = toMapFrame(entity);
    auto match = lanelet_->findNearestTrafficLightMatch(map_pose.pose.position);
    if (match.has_value()) {
      entity.way_id = match->way_id;
      entity.reg_elem_id = match->reg_elem_id;
    }
  }

  /**
   * @brief Vehicles: tracked lane finding.
   *
   * Uses the entity's existing lanelet_id as a BFS hint so that
   * lane assignment is stable across frames, especially at intersections.
   */
  /// @brief Vehicles use tracked lane finding with BFS hint.
  void enrich(Car & entity)
  {
    enrichTracked(entity);
  }

  /// @copydoc enrich(Car&)
  void enrich(Bicycle & entity)
  {
    enrichTracked(entity);
  }

  /// @copydoc enrich(Car&)
  void enrich(Motorcycle & entity)
  {
    enrichTracked(entity);
  }

  /**
   * @brief Tracked lane finding for vehicle-type entities.
   *
   * Transforms the entity pose to map frame, extracts heading, and uses
   * heading-aligned lanelet search with the entity's previous lanelet_id
   * as a BFS hint for frame-to-frame stability.
   *
   * @tparam EntityT Vehicle entity type (Car, Bicycle, Motorcycle).
   * @param entity Entity to enrich with a lanelet_id.
   */
  template <typename EntityT>
  void enrichTracked(EntityT & entity)
  {
    auto map_pose = toMapFrame(entity);
    double yaw = extractYaw(map_pose.pose.orientation);
    entity.lanelet_id = lanelet_->findCurrentLaneletId(
      map_pose.pose.position, yaw, route_priority_threshold_m_, heading_search_radius_m_, entity.lanelet_id);
  }

  /**
   * @brief Transform entity pose to map frame using the cached transform.
   *
   * No I/O — uses the transform looked up by updateTransform().
   */
  geometry_msgs::msg::PoseStamped toMapFrame(const Entity3D & entity) const
  {
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = entity.detection().header;
    pose_in.pose = entity.pose();

    if (tf_is_identity_) {
      return pose_in;
    }

    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose_in, pose_out, cached_tf_);
    return pose_out;
  }

  /**
   * @brief Extract yaw (heading) angle from a quaternion orientation.
   *
   * @param q Quaternion to extract yaw from.
   * @return Yaw angle in radians.
   */
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  const LaneletHandler * lanelet_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  double route_priority_threshold_m_;
  double heading_search_radius_m_;

  geometry_msgs::msg::TransformStamped cached_tf_;
  bool tf_valid_{false};
  bool tf_is_identity_{false};
};

}  // namespace world_model

#endif  // WORLD_MODEL__PIPELINE__LANELET_ENRICHER_HPP_
