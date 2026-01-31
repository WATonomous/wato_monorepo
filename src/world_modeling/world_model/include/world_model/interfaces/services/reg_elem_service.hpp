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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__REG_ELEM_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__REG_ELEM_SERVICE_HPP_

#include <utility>

#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to find lanelets by regulatory element.
 *
 * Handles GetLaneletsByRegElem requests by querying LaneletHandler for
 * all lanelets associated with a given regulatory element ID.
 */
class RegElemService : public InterfaceBase
{
public:
  RegElemService(rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler)
  : node_(node)
  , lanelet_(lanelet_handler)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetLaneletsByRegElem>(
      "get_lanelets_by_reg_elem",
      std::bind(&RegElemService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /**
   * @brief Handles a GetLaneletsByRegElem service request.
   *
   * Delegates to LaneletHandler::getLaneletsByRegElem to find all lanelets
   * that reference the given regulatory element ID.
   *
   * @param request Contains the reg_elem_id to query.
   * @param response Populated with matching lanelet messages and success flag.
   */
  void handleRequest(
    lanelet_msgs::srv::GetLaneletsByRegElem::Request::ConstSharedPtr request,
    lanelet_msgs::srv::GetLaneletsByRegElem::Response::SharedPtr response)
  {
    auto result = lanelet_->getLaneletsByRegElem(request->reg_elem_id);
    *response = std::move(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;

  rclcpp::Service<lanelet_msgs::srv::GetLaneletsByRegElem>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__REG_ELEM_SERVICE_HPP_
