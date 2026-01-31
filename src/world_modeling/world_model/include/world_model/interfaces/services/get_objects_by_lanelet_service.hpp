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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__GET_OBJECTS_BY_LANELET_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__GET_OBJECTS_BY_LANELET_SERVICE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"
#include "world_model_msgs/srv/get_dynamic_objects_by_lanelet.hpp"

namespace world_model
{

/**
 * @brief Service to get dynamic objects by lanelet ID.
 *
 * Queries all entity buffers (Car, Human, Bicycle, Motorcycle)
 * and returns objects currently on the specified lanelet.
 */
class GetObjectsByLaneletService : public InterfaceBase
{
public:
  GetObjectsByLaneletService(
    rclcpp_lifecycle::LifecycleNode * node,
    const WorldState * world_state,
    const LaneletHandler * lanelet_handler,
    const std::string & frame_id)
  : node_(node)
  , world_state_(world_state)
  , lanelet_(lanelet_handler)
  , frame_id_(frame_id)
  {
    srv_ = node_->create_service<world_model_msgs::srv::GetDynamicObjectsByLanelet>(
      "get_objects_by_lanelet",
      std::bind(&GetObjectsByLaneletService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    world_model_msgs::srv::GetDynamicObjectsByLanelet::Request::ConstSharedPtr request,
    world_model_msgs::srv::GetDynamicObjectsByLanelet::Response::SharedPtr response)
  {
    // Validate lanelet exists
    if (lanelet_->isMapLoaded()) {
      auto ll = lanelet_->getLaneletById(request->lanelet_id);
      if (!ll.has_value()) {
        response->success = false;
        response->error_message = "Lanelet not found: " + std::to_string(request->lanelet_id);
        return;
      }
    }

    // Query all entity buffers
    collectEntities<Unknown>(request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_UNKNOWN, response->objects);
    collectEntities<Car>(request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_CAR, response->objects);
    collectEntities<Human>(request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_HUMAN, response->objects);
    collectEntities<Bicycle>(
      request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_BICYCLE, response->objects);
    collectEntities<Motorcycle>(
      request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_MOTORCYCLE, response->objects);
    collectEntities<TrafficLight>(
      request->lanelet_id, world_model_msgs::msg::DynamicObject::TYPE_TRAFFIC_LIGHT, response->objects);

    response->success = true;
  }

  template <typename EntityType>
  void collectEntities(
    int64_t lanelet_id, uint8_t entity_type, std::vector<world_model_msgs::msg::DynamicObject> & objects)
  {
    auto entities = world_state_.buffer<EntityType>().getByLanelet(lanelet_id);
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
  const LaneletHandler * lanelet_;
  std::string frame_id_;

  rclcpp::Service<world_model_msgs::srv::GetDynamicObjectsByLanelet>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__GET_OBJECTS_BY_LANELET_SERVICE_HPP_
