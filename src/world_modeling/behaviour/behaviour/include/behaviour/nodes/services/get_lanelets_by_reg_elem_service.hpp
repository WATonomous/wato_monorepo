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

#ifndef BEHAVIOUR__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_
#define BEHAVIOUR__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"

namespace behaviour
{

/**
   * @class GetLaneletsByRegElemService
   * @brief BT node to fetch lanelets associated with a regulatory element.
   */
class GetLaneletsByRegElemService : public BT::RosServiceNode<lanelet_msgs::srv::GetLaneletsByRegElem>
{
public:
  GetLaneletsByRegElemService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<lanelet_msgs::srv::GetLaneletsByRegElem>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int64_t>("id", "ID of the regulatory element"),
       BT::OutputPort<types::LaneletArrayPtr>("lanelets_id", "Pointer to the list of lanelets")});
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    int64_t id;
    try {
      id = ports::get<int64_t>(*this, "id");
    } catch (const BT::RuntimeError & e) {
      return false;
    }

    request->reg_elem_id = id;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    if (!response->success) {
      return BT::NodeStatus::FAILURE;
    }

    auto lanelets_ptr = std::make_shared<types::LaneletArray>(std::move(response->lanelets));

    setOutput("lanelets", lanelets_ptr);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    (void)error;
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_
