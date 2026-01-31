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

#ifndef BEHAVIOUR__SET_ROUTE_SERVICE_HPP_
#define BEHAVIOUR__SET_ROUTE_SERVICE_HPP_

#include <string>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/set_route.hpp"

namespace behaviour
{
/**
   * @class GetRouteService
   * @brief BT node to request a global route between two lanelets.
   */
class SetRouteService : public BT::RosServiceNode<lanelet_msgs::srv::SetRoute>
{
public:
  SetRouteService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<lanelet_msgs::srv::SetRoute>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("goal_point"),
      BT::OutputPort<std::string>("error_message"),
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    auto gp = ports::tryGetPtr<geometry_msgs::msg::Point>(*this, "goal_point");

    if (gp == nullptr) {
      RCLCPP_WARN(logger(), "[%s] No goal point provided on port", name().c_str());
      return false;
    }

    request->goal_point = *gp;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    if (!response->success) {
      setOutput("error_message", response->error_message);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "Route set: lanelet %ld -> %ld", response->current_lanelet_id, response->goal_lanelet_id);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "SetRoute service failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__SET_ROUTE_SERVICE_HPP_
