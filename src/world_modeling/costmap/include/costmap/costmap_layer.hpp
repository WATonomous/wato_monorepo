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

#ifndef COSTMAP__COSTMAP_LAYER_HPP_
#define COSTMAP__COSTMAP_LAYER_HPP_

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace costmap
{

class CostmapLayer
{
public:
  virtual ~CostmapLayer() = default;

  virtual void configure(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer) = 0;

  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void cleanup() = 0;

  virtual void update(
    nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & map_to_costmap) = 0;
};

}  // namespace costmap

#endif  // COSTMAP__COSTMAP_LAYER_HPP_
