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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_AREA_OCCUPANCY_SERVICE_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_AREA_OCCUPANCY_SERVICE_HPP_

#include <string>
#include <vector>

#include <behaviortree_ros2/bt_service_node.hpp>

#include "world_model_msgs/msg/area_occupancy_info.hpp"
#include "world_model_msgs/srv/get_area_occupancy.hpp"

namespace behaviour
{
  /**
   * @class GetAreaOccupancyService
   * @brief RosServiceNode to request area occupancy snapshot.
   */
  class GetAreaOccupancyService : public BT::RosServiceNode<world_model_msgs::srv::GetAreaOccupancy>
  {
  public:
    GetAreaOccupancyService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
        : BT::RosServiceNode<world_model_msgs::srv::GetAreaOccupancy>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({
          BT::OutputPort<std::vector<world_model_msgs::msg::AreaOccupancyInfo>>("areas"),
          BT::OutputPort<std::string>("error_message"),
      });
    }

    bool setRequest(Request::SharedPtr &request) override
    {
      (void)request;
      return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
    {
      setOutput("areas", response->areas);
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
      setOutput("error_message", std::string(BT::toStr(error)));
      RCLCPP_ERROR(logger(), "GetAreaOccupancy service failed: %d", error);
      return BT::NodeStatus::FAILURE;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_AREA_OCCUPANCY_SERVICE_HPP_
