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

#ifndef BEHAVIOUR__GET_SHORTEST_ROUTE_SERVICE_HPP_
#define BEHAVIOUR__GET_SHORTEST_ROUTE_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
   * @class GetShortestRouteService
   * @brief BT node to request a global route between two lanelets.
   */
class GetShortestRouteService : public BT::RosServiceNode<lanelet_msgs::srv::GetShortestRoute>
{
public:
  GetShortestRouteService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<lanelet_msgs::srv::GetShortestRoute>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr>("route"),
      BT::OutputPort<std::shared_ptr<std::unordered_map<int64_t, size_t>>>("route_index_map"), // New Map Port
      BT::OutputPort<std::string>("error_message"),
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    if (!response->success) {
      setOutput("error_message", response->error_message);
      return BT::NodeStatus::FAILURE;
    }

    auto route = std::make_shared<lanelet_msgs::srv::GetShortestRoute::Response>(std::move(*response));
    
    // 2. Build the Index Map (Lanelet ID -> Index in Vector)
    auto index_map = std::make_shared<std::unordered_map<int64_t, size_t>>>();
    for (size_t i = 0; i < route->lanelets.size(); ++i) {
      // Map the lanelet ID to its sequence position
      (*index_map)[route->lanelets[i].id] = i;
    }

    // 3. Set the Blackboard outputs
    setOutput("route", route);
    setOutput("route_index_map", index_map);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GetShortestRoute service failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__GET_SHORTEST_ROUTE_SERVICE_HPP_
