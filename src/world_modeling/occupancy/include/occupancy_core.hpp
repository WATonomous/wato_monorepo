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

#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include <memory>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OccupancyCore
{
public:
  // Resolution of the costmap (in cells/m)
  int CELLS_PER_METER;

  /**
   * OccupancyCore constructor.
   */
  OccupancyCore();
  OccupancyCore(int resolution);

  /**
   * Removes the z-axis dimension from the given PointCloud2 message.
   *
   * @param msg The input PointCloud2 message
   * @returns the processed point cloud
   */
  nav_msgs::msg::OccupancyGrid remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif  // OCCUPANCY_HPP
