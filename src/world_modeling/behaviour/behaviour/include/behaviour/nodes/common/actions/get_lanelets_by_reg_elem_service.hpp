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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"

namespace behaviour
{
/**
 * @class GetLaneletsByRegElemService
 * @brief RosServiceNode to request GetLaneletsByRegElem service.
 */
class GetLaneletsByRegElemService : public BT::RosServiceNode<lanelet_msgs::srv::GetLaneletsByRegElem>
{
public:
  GetLaneletsByRegElemService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<lanelet_msgs::srv::GetLaneletsByRegElem>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int64_t>("reg_elem_id"),
      BT::OutputPort<std::vector<lanelet_msgs::msg::Lanelet>>("lanelets"),
      BT::OutputPort<std::string>("error_message"),
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_ERROR(logger(), "[%s] Missing input port: %s", name().c_str(), port_name);
      setOutput("error_message", std::string("missing_port:") + port_name);
    };

    auto reg_elem_id = ports::tryGet<int64_t>(*this, "reg_elem_id");
    if (!ports::require(reg_elem_id, "reg_elem_id", missing_input_callback)) {
      return false;
    }

    request->reg_elem_id = *reg_elem_id;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    if (!response->success) {
      setOutput("error_message", response->error_message);
      return BT::NodeStatus::FAILURE;
    }

    // Move into a heap-owned response so downstream nodes can safely keep a shared ptr
    auto lanelets_resp = std::make_shared<lanelet_msgs::srv::GetLaneletsByRegElem::Response>(std::move(*response));

    setOutput("lanelets", lanelets_resp->lanelets);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    setOutput("error_message", std::string(BT::toStr(error)));
    RCLCPP_ERROR(logger(), "GetLaneletsByRegElem service failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELETS_BY_REG_ELEM_SERVICE_HPP_
