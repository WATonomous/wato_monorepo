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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__SHORTEST_ROUTE_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__SHORTEST_ROUTE_SERVICE_HPP_

#include <string>
#include <utility>

#include "lanelet_msgs/srv/get_shortest_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to get the entire shortest route from ego to goal.
 *
 * Requires SetRoute to be called first to establish the destination.
 * Gets ego position from TF and returns ALL lanelets along the cached route
 * from ego's current position to the goal.
 */
class ShortestRouteService : public InterfaceBase
{
public:
  ShortestRouteService(
    rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler, tf2_ros::Buffer * tf_buffer)
  : node_(node)
  , lanelet_(lanelet_handler)
  , ego_pose_(tf_buffer, node->get_parameter("map_frame").as_string(), node->get_parameter("base_frame").as_string())
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetShortestRoute>(
      "get_shortest_route",
      std::bind(&ShortestRouteService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /**
   * @brief Handles a GetShortestRoute service request.
   *
   * Looks up ego position via TF and delegates to LaneletHandler::getShortestRoute
   * to return all lanelets from ego's current position to the cached goal, along
   * with transition types (successor, left, right) and total route length.
   *
   * @param request Unused (ego position is obtained from TF).
   * @param response Populated with route lanelets, transitions, and total distance.
   */
  void handleRequest(
    lanelet_msgs::srv::GetShortestRoute::Request::ConstSharedPtr /*request*/,
    lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr response)
  {
    auto ego_point = ego_pose_.getEgoPoint();
    if (!ego_point.has_value()) {
      response->success = false;
      response->error_message = "tf_lookup_failed";
      RCLCPP_WARN(node_->get_logger(), "GetShortestRoute TF lookup failed");
      return;
    }

    auto result = lanelet_->getShortestRoute(*ego_point);
    *response = std::move(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;

  rclcpp::Service<lanelet_msgs::srv::GetShortestRoute>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__SHORTEST_ROUTE_SERVICE_HPP_
