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

#ifndef WORLD_MODEL__INTERFACES__SUBSCRIBERS__DETECTION_SUBSCRIBER_HPP_
#define WORLD_MODEL__INTERFACES__SUBSCRIBERS__DETECTION_SUBSCRIBER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Subscribes to 3D detections and updates entity buffers.
 *
 * Classifies each detection by type and routes to the appropriate
 * entity buffer (cars, humans, bicycles, motorcycles).
 * Also enriches entities with lanelet context on each update.
 */
class DetectionSubscriber : public InterfaceBase
{
public:
  DetectionSubscriber(
    rclcpp_lifecycle::LifecycleNode * node,
    WorldState * world_state,
    const LaneletHandler * lanelet_handler,
    double history_duration_sec)
  : node_(node)
  , world_state_(world_state)
  , lanelet_(lanelet_handler)
  , history_duration_(history_duration_sec)
  {
    sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
      "detections", 10, std::bind(&DetectionSubscriber::onMessage, this, std::placeholders::_1));
  }

private:
  void onMessage(vision_msgs::msg::Detection3DArray::ConstSharedPtr msg)
  {
    for (const auto & det : msg->detections) {
      EntityType type = classify(det);
      int64_t id = std::stoll(det.id);

      switch (type) {
        case EntityType::CAR:
          updateEntity<Car>(id, det);
          break;
        case EntityType::HUMAN:
          updateEntity<Human>(id, det);
          break;
        case EntityType::BICYCLE:
          updateEntity<Bicycle>(id, det);
          break;
        case EntityType::MOTORCYCLE:
          updateEntity<Motorcycle>(id, det);
          break;
        default:
          break;
      }
    }
  }

  EntityType classify(const vision_msgs::msg::Detection3D & det) const
  {
    if (det.results.empty()) {
      return EntityType::UNKNOWN;
    }

    const std::string & class_id = det.results[0].hypothesis.class_id;

    if (class_id == "car" || class_id == "vehicle" || class_id == "truck") {
      return EntityType::CAR;
    } else if (class_id == "person" || class_id == "pedestrian" || class_id == "human") {
      return EntityType::HUMAN;
    } else if (class_id == "bicycle" || class_id == "cyclist") {
      return EntityType::BICYCLE;
    } else if (class_id == "motorcycle" || class_id == "motorbike") {
      return EntityType::MOTORCYCLE;
    }

    return EntityType::UNKNOWN;
  }

  template <typename EntityT>
  void updateEntity(int64_t id, const vision_msgs::msg::Detection3D & det)
  {
    auto & buffer = world_state_.buffer<EntityT>();

    EntityT default_entity;
    buffer.upsert(id, default_entity, [&det, this](EntityT & entity) {
      entity.history.push_front(det);

      // Trim history by duration
      while (entity.history.size() > 1) {
        auto oldest = rclcpp::Time(entity.history.back().header.stamp);
        auto newest = rclcpp::Time(entity.history.front().header.stamp);
        if ((newest - oldest).seconds() > history_duration_) {
          entity.history.pop_back();
        } else {
          break;
        }
      }

      // Enrich with lanelet context
      if (lanelet_.isMapLoaded() && !entity.empty()) {
        geometry_msgs::msg::Point pt;
        pt.x = entity.pose().position.x;
        pt.y = entity.pose().position.y;
        pt.z = entity.pose().position.z;
        entity.lanelet_id = lanelet_.findNearestLaneletId(pt);
      }
    });
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateWriter world_state_;
  LaneletReader lanelet_;
  double history_duration_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SUBSCRIBERS__DETECTION_SUBSCRIBER_HPP_
