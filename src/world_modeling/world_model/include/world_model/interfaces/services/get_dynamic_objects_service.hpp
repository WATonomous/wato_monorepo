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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__GET_DYNAMIC_OBJECTS_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__GET_DYNAMIC_OBJECTS_SERVICE_HPP_

#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/srv/get_dynamic_objects.hpp"

namespace world_model
{

/**
 * @brief Service that returns all tracked dynamic objects on demand.
 *
 * Mirrors the data published by DynamicObjectsPublisher but as a
 * request/response service for on-demand snapshots.
 */
class GetDynamicObjectsService : public InterfaceBase
{
public:
  GetDynamicObjectsService(
    rclcpp_lifecycle::LifecycleNode * node, const WorldState * world_state, const LaneletHandler * lanelet_handler)
  : node_(node)
  , world_state_(world_state)
  , lanelet_handler_(lanelet_handler)
  , frame_id_(node->get_parameter("map_frame").as_string())
  , radius_m_(node->get_parameter("dynamic_objects_lanelet_ahead_radius_m").as_double())
  {
    srv_ = node_->create_service<world_model_msgs::srv::GetDynamicObjects>(
      "get_dynamic_objects",
      std::bind(&GetDynamicObjectsService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /**
   * @brief Extract the yaw angle from a quaternion orientation.
   * @param q Quaternion to extract yaw from.
   * @return Yaw angle in radians.
   */
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /**
   * @brief Handle an incoming GetDynamicObjects service request.
   *
   * Collects all tracked entities of every type (Unknown, Car, Human, Bicycle,
   * Motorcycle, TrafficLight) into the response's objects list.
   *
   * @param request  The service request (unused).
   * @param response The service response populated with all dynamic objects.
   */
  void handleRequest(
    world_model_msgs::srv::GetDynamicObjects::Request::ConstSharedPtr /*request*/,
    world_model_msgs::srv::GetDynamicObjects::Response::SharedPtr response)
  {
    response->header.stamp = node_->get_clock()->now();
    response->header.frame_id = frame_id_;

    collectEntities<Unknown>(response->objects);
    collectEntities<Car>(response->objects);
    collectEntities<Human>(response->objects);
    collectEntities<Bicycle>(response->objects);
    collectEntities<Motorcycle>(response->objects);
    collectEntities<TrafficLight>(response->objects);
  }

  /**
   * @brief Collect all tracked entities of a given type into the output list.
   *
   * Iterates over every entity of EntityType in the world state buffer, builds
   * a WorldObject message for each non-empty entity (including detection,
   * predictions, lanelet-ahead info, and pose history), and appends it to the
   * output. For TrafficLight entities, also attaches matched way ID and
   * regulatory element data when available.
   *
   * @tparam EntityType The world-model entity type to collect (e.g. Car, Human).
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

      if (entity.lanelet_id.has_value()) {
        double heading = extractYaw(entity.pose().orientation);
        obj.lanelet_ahead =
          lanelet_handler_->getLaneletAhead(entity.pose().position, heading, radius_m_, entity.lanelet_id);
      }

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
  double radius_m_;

  rclcpp::Service<world_model_msgs::srv::GetDynamicObjects>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__GET_DYNAMIC_OBJECTS_SERVICE_HPP_
