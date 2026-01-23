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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_

#include <string>
#include <utility>

#include "lanelet_msgs/srv/get_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to get route lanelets within a distance from ego position.
 *
 * Requires SetRoute to be called first to establish the destination.
 * Gets ego position from TF and returns lanelets along the cached route
 * within the requested distance.
 */
class RouteService : public InterfaceBase
{
public:
  RouteService(
    rclcpp_lifecycle::LifecycleNode * node,
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer,
    const std::string & map_frame,
    const std::string & base_frame)
  : node_(node)
  , lanelet_(lanelet_handler)
  , tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , base_frame_(base_frame)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetRoute>(
      "get_route", std::bind(&RouteService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    lanelet_msgs::srv::GetRoute::Request::ConstSharedPtr request,
    lanelet_msgs::srv::GetRoute::Response::SharedPtr response)
  {
    // Get ego pose from TF
    geometry_msgs::msg::Point ego_point;
    try {
      auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      ego_point.x = transform.transform.translation.x;
      ego_point.y = transform.transform.translation.y;
      ego_point.z = transform.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      response->success = false;
      response->error_message = "tf_lookup_failed";
      RCLCPP_WARN(node_->get_logger(), "GetRoute TF lookup failed: %s", ex.what());
      return;
    }

    // Get route from current position
    auto result = lanelet_.getRouteFromPosition(ego_point, request->distance_m);
    *response = std::move(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  LaneletReader lanelet_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  std::string base_frame_;

  rclcpp::Service<lanelet_msgs::srv::GetRoute>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__ROUTE_SERVICE_HPP_
