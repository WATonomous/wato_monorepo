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

#ifndef BEHAVIOUR__GET_OBJECTS_BY_LANELET_SERVICE_HPP_
#define BEHAVIOUR__GET_OBJECTS_BY_LANELET_SERVICE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>

// srv
#include "world_modeling_msgs/srv/get_objects_by_lanelet.hpp"

namespace behaviour
{
  using GetObjectsByLanelet = world_modeling_msgs::srv::GetObjectsByLanelet;

  /**
   * @class GetObjectsByLaneletService
   * @brief BT node to fetch dynamic objects associated with a specific lanelet.
   *
   * Currently uses a placeholder service type (world_modeling_msgs::srv::GetObjectsByLanelet).
   */
  class GetObjectsByLaneletService : public BT::RosServiceNode<GetObjectsByLanelet>
  {
  public:
    GetObjectsByLaneletService(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
        : BT::RosServiceNode<GetObjectsByLanelet>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {BT::InputPort<int64_t>("lanelet_id"), BT::OutputPort<std::shared_ptr<std::vector<std::string>>>("objects")});
    }

    bool setRequest(Request::SharedPtr &request) override
    {
      auto lanelet_id = getInput<int64_t>("lanelet_id");
      if (!lanelet_id)
        return false;

      request->lanelet_id = lanelet_id.value();
      return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override
    {
      if (!response->success)
        return BT::NodeStatus::FAILURE;

      auto objects_ptr = std::make_shared<std::vector<std::string>>(std::move(response->objects));
      setOutput("objects", objects_ptr);

      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
      (void)error;
      return BT::NodeStatus::FAILURE;
    }
  };
} // namespace behaviour

#endif
