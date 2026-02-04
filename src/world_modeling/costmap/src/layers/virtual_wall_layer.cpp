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

#include "costmap/layers/virtual_wall_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "costmap/costmap_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void VirtualWallLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;
}

void VirtualWallLayer::activate()
{
  spawn_srv_ = node_->create_service<costmap_msgs::srv::SpawnWall>(
    "spawn_wall", std::bind(&VirtualWallLayer::spawnWallCallback, this, std::placeholders::_1, std::placeholders::_2));

  despawn_srv_ = node_->create_service<costmap_msgs::srv::DespawnWall>(
    "despawn_wall",
    std::bind(&VirtualWallLayer::despawnWallCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void VirtualWallLayer::deactivate()
{
  spawn_srv_.reset();
  despawn_srv_.reset();
}

void VirtualWallLayer::cleanup()
{
  spawn_srv_.reset();
  despawn_srv_.reset();
  std::lock_guard<std::mutex> lock(walls_mutex_);
  walls_.clear();
}

void VirtualWallLayer::spawnWallCallback(
  const costmap_msgs::srv::SpawnWall::Request::SharedPtr request,
  costmap_msgs::srv::SpawnWall::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  Wall wall;
  wall.pose = request->pose;
  wall.length = request->length;
  wall.width = request->width;

  int32_t id = next_wall_id_++;
  walls_[id] = wall;

  response->success = true;
  response->wall_id = id;
}

void VirtualWallLayer::despawnWallCallback(
  const costmap_msgs::srv::DespawnWall::Request::SharedPtr request,
  costmap_msgs::srv::DespawnWall::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  auto it = walls_.find(request->wall_id);
  if (it != walls_.end()) {
    walls_.erase(it);
    response->success = true;
  } else {
    response->success = false;
    response->error_message = "Wall ID " + std::to_string(request->wall_id) + " not found";
  }
}

void VirtualWallLayer::update(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & /*map_to_costmap*/)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  for (const auto & [id, wall] : walls_) {
    // Transform the wall pose from its source frame into the costmap frame
    geometry_msgs::msg::PoseStamped pose_out;
    try {
      auto tf = tf_buffer_->lookupTransform(grid.header.frame_id, wall.pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(wall.pose, pose_out, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "VirtualWallLayer TF lookup failed for wall %d: %s",
        id,
        ex.what());
      continue;
    }

    double cx = pose_out.pose.position.x;
    double cy = pose_out.pose.position.y;
    double wall_yaw = yawFromQuat(pose_out.pose.orientation);

    markBox(grid, cx, cy, wall_yaw, wall.length / 2.0, wall.width / 2.0, 100);
  }
}

}  // namespace costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap::VirtualWallLayer, costmap::CostmapLayer)
