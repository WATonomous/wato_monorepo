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

#include <memory>
#include <string>
#include <utility>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/get_route.hpp"

namespace behaviour
{
/**
   * @class GetRouteService
   * @brief BT node to request a global route between two lanelets.
   */
class GetRouteService : public BT::RosServiceNode<lanelet_msgs::srv::GetRoute>
{
public:
  GetRouteService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<lanelet_msgs::srv::GetRoute>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("distance_m"),
      BT::OutputPort<types::PathPtr>("path"),
      BT::OutputPort<std::string>("error_message"),
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    auto distance_m = ports::get<double>(*this, "distance_m");

    request->distance_m = distance_m;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    if (!response->success) {
      setOutput("error_message", response->error_message);
      return BT::NodeStatus::FAILURE;
    }

    auto path_ptr = std::make_shared<types::Path>(std::move(*response));
    setOutput("path", path_ptr);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GetRoute service failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__GET_ROUTE_SERVICE_HPP_
