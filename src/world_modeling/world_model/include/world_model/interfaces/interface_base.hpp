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

#ifndef WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_
#define WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/lanelet_handler.hpp"
#include "world_model/world_state.hpp"

namespace world_model
{

/**
 * @brief Read-only accessor for WorldState.
 *
 * Provides const-only access to entity buffers. The underlying EntityBuffer
 * handles reader locking internally (shared_lock).
 */
class WorldStateReader
{
public:
  explicit WorldStateReader(const WorldState * state)
  : state_(state)
  {}

  template <typename T>
  const EntityBuffer<T> & buffer() const
  {
    return state_->buffer<T>();
  }

private:
  const WorldState * state_;
};

/**
 * @brief Read-write accessor for WorldState.
 *
 * Provides full access to entity buffers. The underlying EntityBuffer
 * handles writer locking internally (unique_lock).
 */
class WorldStateWriter
{
public:
  explicit WorldStateWriter(WorldState * state)
  : state_(state)
  {}

  template <typename T>
  EntityBuffer<T> & buffer()
  {
    return state_->buffer<T>();
  }

  template <typename T>
  const EntityBuffer<T> & buffer() const
  {
    return state_->buffer<T>();
  }

private:
  WorldState * state_;
};

/**
 * @brief Helper for TF-based ego pose lookup.
 *
 * Common utility for interfaces that need to get ego vehicle position
 * from TF transforms. Encapsulates the TF buffer lookup and conversion
 * to PoseStamped.
 */
class EgoPoseHelper
{
public:
  EgoPoseHelper(tf2_ros::Buffer * tf_buffer, const std::string & map_frame, const std::string & base_frame)
  : tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , base_frame_(base_frame)
  {}

  /**
   * @brief Get the current ego pose in map frame.
   * @return Ego pose if TF lookup succeeds, nullopt otherwise.
   */
  std::optional<geometry_msgs::msg::PoseStamped> getEgoPose() const
  {
    try {
      auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = transform.header.stamp;
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      return pose;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  /**
   * @brief Get ego position as a Point (convenience for lanelet queries).
   * @return Ego position if TF lookup succeeds, nullopt otherwise.
   */
  std::optional<geometry_msgs::msg::Point> getEgoPoint() const
  {
    auto pose = getEgoPose();
    if (!pose.has_value()) {
      return std::nullopt;
    }
    return pose->pose.position;
  }

  const std::string & mapFrame() const
  {
    return map_frame_;
  }

  const std::string & baseFrame() const
  {
    return base_frame_;
  }

private:
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  std::string base_frame_;
};

/**
 * @brief Base interface for all ROS interface components.
 *
 * Provides common lifecycle management for publishers, subscribers,
 * services, and workers.
 *
 * Derived classes should use:
 * - WorldStateReader/WorldStateWriter for entity buffer access
 * - const LaneletHandler* for read-only lanelet access (thread-safe after map load)
 * - LaneletHandler* for write access (route caching)
 */
class InterfaceBase
{
public:
  virtual ~InterfaceBase() = default;

  /**
   * @brief Activate the interface (start publishing, subscribe, etc.)
   */
  virtual void activate()
  {}

  /**
   * @brief Deactivate the interface (stop timers, cancel subscriptions, etc.)
   */
  virtual void deactivate()
  {}
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_
