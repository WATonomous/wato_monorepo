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

#ifndef BEHAVIOUR__INTERFACES__SUBSCRIBERS__DYNAMIC_OBJECTS_SUBSCRIBER_HPP_
#define BEHAVIOUR__INTERFACES__SUBSCRIBERS__DYNAMIC_OBJECTS_SUBSCRIBER_HPP_

#include <memory>
#include <utility>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/interfaces/interface_base.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

namespace behaviour::interfaces
{
class DynamicObjectsSubscriber final : public InterfaceBase
{
public:
  DynamicObjectsSubscriber(rclcpp_lifecycle::LifecycleNode * node, std::shared_ptr<DynamicObjectStore> store)
  : node_(node)
  , store_(std::move(store))
  {}

  void activate() override
  {
    if (subscription_) {
      return;
    }

    subscription_ = node_->create_subscription<world_model_msgs::msg::DynamicObjectArray>(
      "dynamic_objects", 10, [this](const world_model_msgs::msg::DynamicObjectArray::SharedPtr msg) {
        store_->update(msg);
      });
  }

  void deactivate() override
  {
    subscription_.reset();
  }

private:
  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<DynamicObjectStore> store_;
  rclcpp::Subscription<world_model_msgs::msg::DynamicObjectArray>::SharedPtr subscription_;
};
}  // namespace behaviour::interfaces

#endif  // BEHAVIOUR__INTERFACES__SUBSCRIBERS__DYNAMIC_OBJECTS_SUBSCRIBER_HPP_
