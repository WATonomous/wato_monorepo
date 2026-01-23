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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_

#include <utility>

#include "lanelet_msgs/srv/get_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to compute routes between lanelets.
 *
 * Handles GetRoute requests by querying LaneletHandler for
 * the shortest route between two lanelet IDs.
 */
class RouteService : public InterfaceBase
{
public:
  RouteService(rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler)
  : node_(node)
  , lanelet_(lanelet_handler)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetRoute>(
      "get_route", std::bind(&RouteService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    lanelet_msgs::srv::GetRoute::Request::ConstSharedPtr request,
    lanelet_msgs::srv::GetRoute::Response::SharedPtr response)
  {
    auto result = lanelet_.getRoute(request->from_lanelet_id, request->to_lanelet_id);
    *response = std::move(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  LaneletReader lanelet_;

  rclcpp::Service<lanelet_msgs::srv::GetRoute>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_
