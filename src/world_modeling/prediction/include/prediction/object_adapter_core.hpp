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

#ifndef PREDICTION__OBJECT_ADAPTER_CORE_HPP_
#define PREDICTION__OBJECT_ADAPTER_CORE_HPP_

#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace wato {
namespace world_modeling {
namespace prediction {

class ObjectAdapterCore
{
public:
  ObjectAdapterCore();

  void processMarkerArray(
    const visualization_msgs::msg::MarkerArray::SharedPtr & msg,
    autoware_perception_msgs::msg::TrackedObjects & tracked_objects);

private:
  std::unordered_map<int, autoware_perception_msgs::msg::TrackedObject> prev_objects_;
  rclcpp::Time prev_time_;
  bool is_first_ = true;
};

}  // namespace prediction
}  // namespace world_modeling
}  // namespace wato

#endif  // PREDICTION__OBJECT_ADAPTER_CORE_HPP_
