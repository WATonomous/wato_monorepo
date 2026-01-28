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

#ifndef BEHAVIOUR__NODES__SERVICES__SET_WALL_SERVICE_HPP_
#define BEHAVIOUR__NODES__SERVICES__SET_WALL_SERVICE_HPP_

#include <string>

#include <behaviortree_ros2/bt_service_node.hpp>

// TODO(wato): to improve
// #include "world_model_msgs/srv/set_wall.hpp"

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class SetWallService
   * @brief The wall of doom to stop the car! Node to spawn or despawn a wall at a stop line.
   *
   * TODO(wato): create world model service that accepts this service
   *
   */
class SetWallService : public BT::RosServiceNode<world_model_msgs::srv::SetWall>
{
public:
  SetWallService(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<world_model_msgs::srv::SetWall>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int64_t>("stop_line_id"),
      BT::InputPort<uint8_t>("command"),
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    try {
      request->stop_line_id = ports::get<int64_t>(*this, "stop_line_id");
      request->command = ports::get<uint8_t>(*this, "command");
    } catch (const BT::RuntimeError & e) {
      return false;
    }
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "SetWall service failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__SERVICES__SET_WALL_SERVICE_HPP_
