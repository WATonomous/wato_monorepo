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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__DYNAMIC_OBJECTS_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__DYNAMIC_OBJECTS_PUBLISHER_HPP_

#include <chrono>
#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace world_model
{

/**
 * @brief Publishes all tracked dynamic objects at a fixed rate.
 *
 * Collects entities from all entity types (Car, Human, Bicycle, Motorcycle, etc.)
 * and publishes them as a WorldObjectArray message with lanelet context and history.
 */
class DynamicObjectsPublisher : public InterfaceBase
{
public:
  DynamicObjectsPublisher(
    rclcpp_lifecycle::LifecycleNode * node, const WorldState * world_state, const LaneletHandler * lanelet_handler)
  : node_(node)
  , world_state_(world_state)
  , lanelet_handler_(lanelet_handler)
  {
    frame_id_ = node_->get_parameter("map_frame").as_string();
    rate_hz_ = node_->declare_parameter<double>("dynamic_objects_publish_rate_hz", 10.0);
    radius_m_ = node_->declare_parameter<double>("dynamic_objects_lanelet_ahead_radius_m", 30.0);

    pub_ = node_->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_objects", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DynamicObjectsPublisher::publish, this));
    }
  }

  void deactivate() override
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    pub_->on_deactivate();
  }

private:
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /**
   * @brief Timer callback that collects all tracked entities and publishes them.
   *
   * Iterates over all entity type buffers (Unknown, Car, Human, Bicycle, Motorcycle,
   * TrafficLight), converts each to a WorldObject message, and publishes the array.
   */
  void publish()
  {
    world_model_msgs::msg::WorldObjectArray msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Collect from all entity types
    collectEntities<Unknown>(msg.objects);
    collectEntities<Car>(msg.objects);
    collectEntities<Human>(msg.objects);
    collectEntities<Bicycle>(msg.objects);
    collectEntities<Motorcycle>(msg.objects);
    collectEntities<TrafficLight>(msg.objects);

    pub_->publish(msg);
  }

  /**
   * @brief Collects all entities of a given type into the output vector.
   *
   * Reads a snapshot of the entity buffer, converts each entity to a WorldObject
   * message with detection, lanelet_ahead, history, and predictions.
   *
   * @tparam EntityType Entity class (e.g. Car, Human).
   * @param objects Output vector to append WorldObject messages to.
   */
  template <typename EntityType>
  void collectEntities(std::vector<world_model_msgs::msg::WorldObject> & objects)
  {
    auto entities = world_state_.buffer<EntityType>().getAll();
    for (const auto & entity : entities) {
      if (entity.empty()) {
        continue;
      }

      world_model_msgs::msg::WorldObject obj;
      obj.header.stamp = node_->get_clock()->now();
      obj.header.frame_id = entity.frameId();
      obj.detection = entity.detection();
      obj.predictions = entity.predictions;

      // Populate lanelet_ahead from entity's cached lanelet_id
      if (entity.lanelet_id.has_value()) {
        double heading = extractYaw(entity.pose().orientation);
        obj.lanelet_ahead =
          lanelet_handler_->getLaneletAhead(entity.pose().position, heading, radius_m_, entity.lanelet_id);
      }

      // History: convert Detection3D deque to PoseStamped[]
      for (const auto & det : entity.history) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = det.header;
        ps.pose = det.bbox.center;
        obj.history.push_back(ps);
      }

      if constexpr (std::is_same_v<EntityType, TrafficLight>) {
        if (entity.way_id > 0) {
          obj.matched_way_id = entity.way_id;
        }
        if (entity.reg_elem_id > 0) {
          auto re = lanelet_handler_->getRegulatoryElementMsg(entity.reg_elem_id);
          if (re.has_value()) {
            obj.regulatory_element = *re;
          }
        }
      }

      objects.push_back(obj);
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateReader world_state_;
  const LaneletHandler * lanelet_handler_;
  std::string frame_id_;
  double rate_hz_;
  double radius_m_;

  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__DYNAMIC_OBJECTS_PUBLISHER_HPP_
