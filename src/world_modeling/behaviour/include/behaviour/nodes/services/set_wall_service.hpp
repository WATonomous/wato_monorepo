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

#ifndef BEHAVIOUR__SET_WALL_SERVICE_HPP_
#define BEHAVIOUR__SET_WALL_SERVICE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>

// srv
#include "world_modeling_msgs/srv/set_wall.hpp"

namespace behaviour
{
  using SetWall = world_modeling_msgs::srv::SetWall;

  /**
   * @class SetWallService
   * @brief BT node to spawn or despawn a wall at a stop line.
   */
  class SetWallService : public BT::RosServiceNode<SetWall>
  {
  public:
    SetWallService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
        : BT::RosServiceNode<SetWall>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {
              BT::InputPort<int64_t>("stop_line_id"),
              BT::InputPort<uint8_t>("command"),
          });
    }

    bool setRequest(Request::SharedPtr &request) override
    {
      auto stop_line_id = getInput<int64_t>("stop_line_id");
      auto command = getInput<uint8_t>("command");

      if (!stop_line_id || !command)
      {
        return false;
      }

      request->stop_line_id = stop_line_id.value();
      request->command = command.value();
      return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
    {
      return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
      (void)error;
      return BT::NodeStatus::FAILURE;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__SET_WALL_SERVICE_HPP_
