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

#ifndef PREDICTION__OBJECT_ADAPTER_NODE_HPP_
#define PREDICTION__OBJECT_ADAPTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include "prediction/object_adapter_core.hpp"

class ObjectAdapter : public rclcpp::Node
{
public:
  ObjectAdapter();

private:
  void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr publisher_;

  wato::world_modeling::prediction::ObjectAdapterCore object_adapter_;
};

#endif  // PREDICTION__OBJECT_ADAPTER_NODE_HPP_
