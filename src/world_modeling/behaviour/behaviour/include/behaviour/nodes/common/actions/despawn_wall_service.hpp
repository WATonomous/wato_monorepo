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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__DESPAWN_WALL_SERVICE_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__DESPAWN_WALL_SERVICE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/utils/ports.hpp"

#include "costmap_msgs/srv/despawn_wall.hpp"

namespace behaviour
{
    /**
     * @class DespawnWallService
     * @brief BT node to request despawning a virtual wall by wall_id.
     *
     */
    class DespawnWallService : public BT::RosServiceNode<costmap_msgs::srv::DespawnWall>
    {
    public:
        DespawnWallService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<costmap_msgs::srv::DespawnWall>(name, conf, params)
        {
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<int32_t>("wall_id"),
                BT::OutputPort<std::string>("error_message"),
            });
        }

        bool setRequest(Request::SharedPtr &request) override
        {
            auto wall_id = ports::tryGet<int32_t>(*this, "wall_id");
            if (!wall_id)
            {
                setOutput("error_message", "missing_port:wall_id");
                RCLCPP_ERROR(logger(), "[DespawnWallService] Missing input port: wall_id");
                return false;
            }

            RCLCPP_INFO(logger(), "[DespawnWallService] Despawning wall with wall_id: %d", *wall_id);

            request->wall_id = *wall_id;
            return true;
        }

        BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
        {
            if (!response->success)
            {
                setOutput("error_message", response->error_message);
                if (response->error_message == "wall_not_found")
                {
                    // if wall not found then assuming it is not there, so return SUCCESS
                    RCLCPP_WARN(logger(), "[DespawnWallService] Wall not found");
                    return BT::NodeStatus::SUCCESS;
                }
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(logger(), "[DespawnWallService] service response received");
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
        {
            setOutput("error_message", std::string(BT::toStr(error)));
            RCLCPP_ERROR(logger(), "[DespawnWallService] service failed: %d", error);
            return BT::NodeStatus::FAILURE;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__COMMON__ACTIONS__DESPAWN_WALL_SERVICE_HPP_
