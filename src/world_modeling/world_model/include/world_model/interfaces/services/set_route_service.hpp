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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__SET_ROUTE_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__SET_ROUTE_SERVICE_HPP_

#include <string>
#include <utility>

#include "lanelet_msgs/srv/set_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Service to set a route destination.
 *
 * Gets ego position from TF, finds nearest lanelets to ego and goal,
 * computes the route, and caches it in LaneletHandler for GetRoute queries.
 */
class SetRouteService : public InterfaceBase
{
public:
  SetRouteService(
    rclcpp_lifecycle::LifecycleNode * node,
    LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer,
    const std::string & map_frame,
    const std::string & base_frame)
  : node_(node)
  , lanelet_(lanelet_handler)
  , tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , base_frame_(base_frame)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::SetRoute>(
      "set_route", std::bind(&SetRouteService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    lanelet_msgs::srv::SetRoute::Request::ConstSharedPtr request,
    lanelet_msgs::srv::SetRoute::Response::SharedPtr response)
  {
    response->success = false;
    response->current_lanelet_id = -1;
    response->goal_lanelet_id = -1;

    // Check if map is loaded
    if (!lanelet_.isMapLoaded()) {
      response->error_message = "map_not_loaded";
      return;
    }

    // Get ego pose from TF
    geometry_msgs::msg::Point ego_point;
    try {
      auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      ego_point.x = transform.transform.translation.x;
      ego_point.y = transform.transform.translation.y;
      ego_point.z = transform.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      response->error_message = "tf_lookup_failed";
      RCLCPP_WARN(node_->get_logger(), "SetRoute TF lookup failed: %s", ex.what());
      return;
    }

    // Find nearest lanelet to ego position
    auto ego_lanelet_id = lanelet_.findNearestLaneletId(ego_point);
    if (!ego_lanelet_id.has_value()) {
      response->error_message = "ego_lanelet_not_found";
      return;
    }
    response->current_lanelet_id = *ego_lanelet_id;

    // Find nearest lanelet to goal point
    auto goal_lanelet_id = lanelet_.findNearestLaneletId(request->goal_point);
    if (!goal_lanelet_id.has_value()) {
      response->error_message = "goal_lanelet_not_found";
      return;
    }
    response->goal_lanelet_id = *goal_lanelet_id;

    // Compute and cache the route
    if (!lanelet_.setActiveRoute(*ego_lanelet_id, *goal_lanelet_id)) {
      response->error_message = "no_route_exists";
      return;
    }

    response->success = true;
    RCLCPP_INFO(
      node_->get_logger(), "Route set: lanelet %ld -> %ld", response->current_lanelet_id, response->goal_lanelet_id);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  LaneletWriter lanelet_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  std::string base_frame_;

  rclcpp::Service<lanelet_msgs::srv::SetRoute>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__SET_ROUTE_SERVICE_HPP_
