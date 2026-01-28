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

#ifndef BEHAVIOUR__NODES__SERVICES__GET_OBJECTS_BY_LANELET_ACTION_HPP_
#define BEHAVIOUR__NODES__SERVICES__GET_OBJECTS_BY_LANELET_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>
#include <utility>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class GetObjectsByLaneletAction
   * @brief BT node to fetch dynamic objects associated with a specific lanelet from the local store.
   */
class GetObjectsByLaneletAction : public BT::SyncActionNode
{
public:
  GetObjectsByLaneletAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  , store_(nullptr)
  {}

  void init(DynamicObjectStore * store)
  {
    if (!store) {
      throw std::runtime_error("GetObjectsByLaneletAction: DynamicObjectStore pointer is null during initialization!");
    }
    store_ = store;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int64_t>("lanelet_id"), BT::OutputPort<types::DynamicObjectArray>("objects")};
  }

  BT::NodeStatus tick() override
  {
    if (!store_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GetObjectsByLaneletAction"),
        "DynamicObjectStore is null! Node was not initialized correctly.");
      return BT::NodeStatus::FAILURE;
    }

    int64_t lanelet_id;
    try {
      lanelet_id = ports::get<int64_t>(*this, "lanelet_id");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    auto objects = store_->getObjectsOnLanelet(lanelet_id);
    setOutput("objects", objects);

    return BT::NodeStatus::SUCCESS;
  }

private:
  DynamicObjectStore * store_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__SERVICES__GET_OBJECTS_BY_LANELET_ACTION_HPP_
