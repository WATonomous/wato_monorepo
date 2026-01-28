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

#ifndef BEHAVIOUR__NODES__ACTIONS__GET_OBJECTS_BY_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_OBJECTS_BY_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class GetObjectsByLaneletsAction
   * @brief BT node to fetch dynamic objects associated with multiple lanelets from the local store.
   */
class GetObjectsByLaneletsAction : public BT::SyncActionNode
{
public:
  GetObjectsByLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  , store_(nullptr)
  {}

  void init(DynamicObjectStore * store)
  {
    if (!store) {
      throw std::runtime_error("GetObjectsByLaneletsAction: DynamicObjectStore pointer is null during initialization!");
    }
    store_ = store;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<std::vector<int64_t>>>("lanelet_ids"),
      BT::OutputPort<types::DynamicObjectArray>("objects")};
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<std::vector<int64_t>> lanelet_ids;

    try {
      lanelet_ids = ports::getPtr<std::vector<int64_t>>(*this, "lanelet_ids");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    types::DynamicObjectArray all_objects;
    for (int64_t id : *lanelet_ids) {
      auto objects = store_->getObjectsOnLanelet(id);
      all_objects.insert(all_objects.end(), objects.begin(), objects.end());
    }

    // Remove duplicates if any
    std::sort(all_objects.begin(), all_objects.end(), [](const auto & a, const auto & b) { return a.id < b.id; });
    all_objects.erase(
      std::unique(all_objects.begin(), all_objects.end(), [](const auto & a, const auto & b) { return a.id == b.id; }),
      all_objects.end());

    setOutput("objects", all_objects);

    return BT::NodeStatus::SUCCESS;
  }

private:
  DynamicObjectStore * store_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__GET_OBJECTS_BY_LANELETS_ACTION_HPP_
