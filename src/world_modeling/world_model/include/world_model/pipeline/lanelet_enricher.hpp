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
    const std::string & map_frame)
  : lanelet_(lanelet_handler)
  , tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
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

  template <typename EntityT>
  void enrich(std::unordered_map<int64_t, EntityT> & map)
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
  // --- Per-entity enrichment strategies ---

  // Default: no enrichment.
  void enrich(Unknown & /*entity*/) {}
  void enrich(Human & /*entity*/) {}
  /**
   * @brief TrafficLight: match to nearest traffic light regulatory element.
   */
  void enrich(TrafficLight & entity)
  {
    auto map_pose = toMapFrame(entity);
    auto reg_id = lanelet_->findNearestTrafficLightRegElemId(map_pose.pose.position);
    if (reg_id.has_value()) {
      entity.reg_elem_id = *reg_id;
    }
  }

  /**
   * @brief Vehicles: tracked lane finding.
   *
   * Uses the entity's existing lanelet_id as a BFS hint so that
   * lane assignment is stable across frames, especially at intersections.
   */
  void enrich(Car & entity) { enrichTracked(entity); }
  void enrich(Bicycle & entity) { enrichTracked(entity); }
  void enrich(Motorcycle & entity) { enrichTracked(entity); }

  // --- Shared implementation ---

  template <typename EntityT>
  void enrichTracked(EntityT & entity)
  {
    auto map_pose = toMapFrame(entity);
    double yaw = extractYaw(map_pose.pose.orientation);
    entity.lanelet_id = lanelet_->findCurrentLaneletId(
      map_pose.pose.position, yaw, 10.0, 15.0, entity.lanelet_id);
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

  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  const LaneletHandler * lanelet_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;

  geometry_msgs::msg::TransformStamped cached_tf_;
  bool tf_valid_{false};
  bool tf_is_identity_{false};
};

}  // namespace world_model

#endif  // WORLD_MODEL__PIPELINE__LANELET_ENRICHER_HPP_
