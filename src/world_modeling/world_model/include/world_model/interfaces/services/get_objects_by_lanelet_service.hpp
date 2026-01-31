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

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/srv/get_objects_by_lanelet.hpp"

namespace world_model
{

/**
 * @brief Service to get objects by lanelet ID.
 *
 * Queries all entity buffers (Car, Human, Bicycle, Motorcycle, etc.)
 * and returns objects currently on the specified lanelet.
 */
class GetObjectsByLaneletService : public InterfaceBase
{
public:
  GetObjectsByLaneletService(
    rclcpp_lifecycle::LifecycleNode * node, const WorldState * world_state, const LaneletHandler * lanelet_handler)
  : node_(node)
  , world_state_(world_state)
  , lanelet_(lanelet_handler)
  , frame_id_(node->get_parameter("map_frame").as_string())
  , radius_m_(node->declare_parameter<double>("get_objects_lanelet_ahead_radius_m", 30.0))
  {
    srv_ = node_->create_service<world_model_msgs::srv::GetObjectsByLanelet>(
      "get_objects_by_lanelet",
      std::bind(&GetObjectsByLaneletService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /**
   * @brief Handles a GetObjectsByLanelet service request.
   *
   * Validates the lanelet ID exists on the map, then queries all entity type
   * buffers for objects whose lanelet_id matches the requested ID.
   *
   * @param request Contains the lanelet_id to query.
   * @param response Populated with matching WorldObject messages and success flag.
   */
  void handleRequest(
    world_model_msgs::srv::GetObjectsByLanelet::Request::ConstSharedPtr request,
    world_model_msgs::srv::GetObjectsByLanelet::Response::SharedPtr response)
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
    collectEntities<Unknown>(request->lanelet_id, response->objects);
    collectEntities<Car>(request->lanelet_id, response->objects);
    collectEntities<Human>(request->lanelet_id, response->objects);
    collectEntities<Bicycle>(request->lanelet_id, response->objects);
    collectEntities<Motorcycle>(request->lanelet_id, response->objects);
    collectEntities<TrafficLight>(request->lanelet_id, response->objects);

    response->success = true;
  }

  /**
   * @brief Collects entities of a given type that are on the specified lanelet.
   *
   * @tparam EntityType Entity class to query (e.g. Car, Human).
   * @param lanelet_id Lanelet ID to filter entities by.
   * @param objects Output vector to append matching WorldObject messages to.
   */
  template <typename EntityType>
  void collectEntities(int64_t lanelet_id, std::vector<world_model_msgs::msg::WorldObject> & objects)
  {
    auto entities = world_state_.buffer<EntityType>().getByLanelet(lanelet_id);
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
        obj.lanelet_ahead = lanelet_->getLaneletAhead(entity.pose().position, heading, radius_m_, entity.lanelet_id);
      }

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
  double radius_m_;

  rclcpp::Service<world_model_msgs::srv::GetObjectsByLanelet>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__GET_OBJECTS_BY_LANELET_SERVICE_HPP_
