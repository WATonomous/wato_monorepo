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
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

namespace world_model
{

/**
 * @brief Publishes all tracked dynamic objects at a fixed rate.
 *
 * Collects entities from all 4 entity types (Car, Human, Bicycle, Motorcycle)
 * and publishes them as a DynamicObjectArray message.
 */
class DynamicObjectsPublisher : public InterfaceBase
{
public:
  DynamicObjectsPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const WorldState * world_state)
  : node_(node)
  , world_state_(world_state)
  {
    frame_id_ = node_->get_parameter("map_frame").as_string();
    rate_hz_ = node_->declare_parameter<double>("dynamic_objects_publish_rate_hz", 10.0);

    pub_ = node_->create_publisher<world_model_msgs::msg::DynamicObjectArray>("dynamic_objects", 10);
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
  /**
   * @brief Timer callback that collects all tracked entities and publishes them.
   *
   * Iterates over all entity type buffers (Unknown, Car, Human, Bicycle, Motorcycle,
   * TrafficLight), converts each to a DynamicObject message, and publishes the array.
   */
  void publish()
  {
    world_model_msgs::msg::DynamicObjectArray msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Collect from all entity types
    collectEntities<Unknown>(world_model_msgs::msg::DynamicObject::TYPE_UNKNOWN, msg.objects);
    collectEntities<Car>(world_model_msgs::msg::DynamicObject::TYPE_CAR, msg.objects);
    collectEntities<Human>(world_model_msgs::msg::DynamicObject::TYPE_HUMAN, msg.objects);
    collectEntities<Bicycle>(world_model_msgs::msg::DynamicObject::TYPE_BICYCLE, msg.objects);
    collectEntities<Motorcycle>(world_model_msgs::msg::DynamicObject::TYPE_MOTORCYCLE, msg.objects);
    collectEntities<TrafficLight>(world_model_msgs::msg::DynamicObject::TYPE_TRAFFIC_LIGHT, msg.objects);

    pub_->publish(msg);
  }

  /**
   * @brief Collects all entities of a given type into the output vector.
   *
   * Reads a snapshot of the entity buffer, converts each entity to a DynamicObject
   * message with pose, size, lanelet ID, history, and predictions.
   *
   * @tparam EntityType Entity class (e.g. Car, Human).
   * @param entity_type DynamicObject type constant for the output message.
   * @param objects Output vector to append DynamicObject messages to.
   */
  template <typename EntityType>
  void collectEntities(uint8_t entity_type, std::vector<world_model_msgs::msg::DynamicObject> & objects)
  {
    auto entities = world_state_.buffer<EntityType>().getAll();
    for (const auto & entity : entities) {
      if (entity.empty()) {
        continue;
      }

      world_model_msgs::msg::DynamicObject obj;
      obj.header.stamp = node_->get_clock()->now();
      obj.header.frame_id = entity.frameId();
      obj.id = entity.id();
      obj.entity_type = entity_type;
      obj.pose = entity.pose();
      obj.size = entity.size();
      obj.lanelet_id = entity.lanelet_id.value_or(-1);
      obj.detection_timestamp = entity.detection().header.stamp;
      obj.predictions = entity.predictions;

      // Populate historical path from detection history
      for (const auto & det : entity.history) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = det.header;
        pose_stamped.pose = det.bbox.center;
        obj.history.push_back(pose_stamped);
      }

      objects.push_back(obj);
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateReader world_state_;
  std::string frame_id_;
  double rate_hz_;

  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::DynamicObjectArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__DYNAMIC_OBJECTS_PUBLISHER_HPP_
