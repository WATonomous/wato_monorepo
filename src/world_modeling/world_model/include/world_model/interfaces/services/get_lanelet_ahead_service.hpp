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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__GET_LANELET_AHEAD_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__GET_LANELET_AHEAD_SERVICE_HPP_

#include <string>

#include "lanelet_msgs/srv/get_lanelet_ahead.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model {

/**
 * @brief Service to get reachable lanelets from an arbitrary position and
 * heading.
 *
 * Uses LaneletHandler::getLaneletAhead() to find the nearest lanelet to the
 * query point and return all BFS-reachable lanelets within the requested
 * radius.
 */
class GetLaneletAheadService : public InterfaceBase {
public:
  GetLaneletAheadService(rclcpp_lifecycle::LifecycleNode *node,
                         const LaneletHandler *lanelet_handler)
      : node_(node), lanelet_(lanelet_handler) {
    srv_ = node_->create_service<lanelet_msgs::srv::GetLaneletAhead>(
        "get_lanelet_ahead",
        std::bind(&GetLaneletAheadService::handleRequest, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
      lanelet_msgs::srv::GetLaneletAhead::Request::ConstSharedPtr request,
      lanelet_msgs::srv::GetLaneletAhead::Response::SharedPtr response) {
    response->success = false;

    if (!lanelet_->isMapLoaded()) {
      response->error_message = "map_not_loaded";
      return;
    }

    auto result = lanelet_->getLaneletAhead(
        request->position, request->heading_rad, request->radius_m);

    if (result.current_lanelet_id < 0) {
      response->error_message = "lanelet_not_found";
      return;
    }

    response->success = true;
    response->lanelet_ahead = result;
  }

  rclcpp_lifecycle::LifecycleNode *node_;
  const LaneletHandler *lanelet_;

  rclcpp::Service<lanelet_msgs::srv::GetLaneletAhead>::SharedPtr srv_;
};

} // namespace world_model

#endif // WORLD_MODEL__INTERFACES__SERVICES__GET_LANELET_AHEAD_SERVICE_HPP_
