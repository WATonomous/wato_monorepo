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

#ifndef BEHAVIOUR__GET_ROUTE_SERVICE_HPP_
#define BEHAVIOUR__GET_ROUTE_SERVICE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>

// srv
#include "world_modeling_msgs/srv/get_route.hpp"

namespace behaviour
{
  using GetRoute = world_modeling_msgs::srv::GetRoute;

  /**
   * @class GetRouteService
   * @brief BT node to request a global route between two lanelets.
   */
  class GetRouteService : public BT::RosServiceNode<GetRoute>
  {
  public:
    GetRouteService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
        : BT::RosServiceNode<GetRoute>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {
              BT::InputPort<int64_t>("from_lanelet_id", "Starting lanelet ID"),
              BT::InputPort<int64_t>("to_lanelet_id", "Destination lanelet ID"),
              BT::OutputPort<std::shared_ptr<std::vector<world_modeling_msgs::msg::Lanelet>>>("route"),
              BT::OutputPort<std::shared_ptr<std::vector<uint8_t>>>("transitions"),
          });
    }

    bool setRequest(Request::SharedPtr &request) override
    {
      auto from_lanelet_id = getInput<int64_t>("from_lanelet_id");
      auto to_lanelet_id = getInput<int64_t>("to_lanelet_id");

      if (!from_lanelet_id || !to_lanelet_id)
      {
        return false;
      }

      request->from_lanelet_id = from_lanelet_id.value();
      request->to_lanelet_id = to_lanelet_id.value();
      return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
    {
      if (!response->success)
      {
        return BT::NodeStatus::FAILURE;
      }

      auto lanelets_ptr = std::make_shared<std::vector<world_modeling_msgs::msg::Lanelet>>(std::move(response->lanelets));
      auto transitions_ptr = std::make_shared<std::vector<uint8_t>>(std::move(response->transitions));

      setOutput("route", lanelets_ptr);
      setOutput("transitions", transitions_ptr);

      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
      (void)error;
      return BT::NodeStatus::FAILURE;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__GET_ROUTE_SERVICE_HPP_
