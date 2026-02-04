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

#ifndef COSTMAP__LAYERS__VIRTUAL_WALL_LAYER_HPP_
#define COSTMAP__LAYERS__VIRTUAL_WALL_LAYER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>

#include "costmap/costmap_layer.hpp"
#include "costmap_msgs/srv/despawn_wall.hpp"
#include "costmap_msgs/srv/spawn_wall.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/service.hpp"

namespace costmap
{

struct Wall
{
  geometry_msgs::msg::PoseStamped pose;
  double length;
  double width;
};

class VirtualWallLayer : public CostmapLayer
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void update(
    nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & map_to_costmap) override;

private:
  void spawnWallCallback(
    const costmap_msgs::srv::SpawnWall::Request::SharedPtr request,
    costmap_msgs::srv::SpawnWall::Response::SharedPtr response);

  void despawnWallCallback(
    const costmap_msgs::srv::DespawnWall::Request::SharedPtr request,
    costmap_msgs::srv::DespawnWall::Response::SharedPtr response);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};
  std::string layer_name_;

  rclcpp::Service<costmap_msgs::srv::SpawnWall>::SharedPtr spawn_srv_;
  rclcpp::Service<costmap_msgs::srv::DespawnWall>::SharedPtr despawn_srv_;

  std::mutex walls_mutex_;
  std::unordered_map<int32_t, Wall> walls_;
  int32_t next_wall_id_{1};
};

}  // namespace costmap

#endif  // COSTMAP__LAYERS__VIRTUAL_WALL_LAYER_HPP_
