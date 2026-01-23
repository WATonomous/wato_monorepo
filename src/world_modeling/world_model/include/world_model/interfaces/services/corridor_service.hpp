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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__CORRIDOR_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__CORRIDOR_SERVICE_HPP_

#include <utility>

#include "lanelet_msgs/srv/get_corridor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to compute driving corridors between lanelets.
 *
 * Handles GetCorridor requests by querying LaneletHandler for
 * a corridor (sequence of lanelets with boundaries) between two lanelet IDs.
 */
class CorridorService : public InterfaceBase
{
public:
  CorridorService(rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler)
  : node_(node)
  , lanelet_(lanelet_handler)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetCorridor>(
      "get_corridor", std::bind(&CorridorService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    lanelet_msgs::srv::GetCorridor::Request::ConstSharedPtr request,
    lanelet_msgs::srv::GetCorridor::Response::SharedPtr response)
  {
    auto result = lanelet_->getCorridor(
      request->from_lanelet_id, request->to_lanelet_id, request->max_length_m, request->sample_spacing_m);
    *response = std::move(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;

  rclcpp::Service<lanelet_msgs::srv::GetCorridor>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__CORRIDOR_SERVICE_HPP_
