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

#ifndef WORLD_MODEL__INTERFACES__SERVICES__GET_AREA_OCCUPANCY_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__GET_AREA_OCCUPANCY_SERVICE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model/types/detection_area.hpp"
#include "world_model_msgs/msg/area_occupancy_info.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/srv/get_area_occupancy.hpp"

namespace world_model
{

/**
 * @brief Service that returns current area occupancy on demand.
 *
 * Mirrors the data published by AreaOccupancyPublisher but as a
 * request/response service for on-demand snapshots.
 */
class GetAreaOccupancyService : public InterfaceBase
{
public:
  GetAreaOccupancyService(
    rclcpp_lifecycle::LifecycleNode * node,
    const WorldState * world_state,
    tf2_ros::Buffer * tf_buffer,
    const LaneletHandler * lanelet_handler)
  : node_(node)
  , world_state_(world_state)
  , tf_buffer_(tf_buffer)
  , lanelet_handler_(lanelet_handler)
  , map_frame_(node->get_parameter("map_frame").as_string())
  , area_frame_(node->get_parameter("area_occupancy_frame").as_string())
  , radius_m_(node->get_parameter("area_occupancy_lanelet_ahead_radius_m").as_double())
  , areas_(parseOccupancyAreas())
  {
    srv_ = node_->create_service<world_model_msgs::srv::GetAreaOccupancy>(
      "get_area_occupancy",
      std::bind(&GetAreaOccupancyService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /**
   * @brief Extract the yaw angle from a quaternion orientation.
   * @param q Quaternion to extract yaw from.
   * @return Yaw angle in radians.
   */
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /**
   * @brief Handle an incoming GetAreaOccupancy service request.
   *
   * Looks up the map-to-area transform, then iterates over each configured
   * detection area and checks all entity types for occupancy. Populates the
   * response with per-area occupancy info and any world objects found inside.
   *
   * @param request  The service request (unused).
   * @param response The service response populated with area occupancy data.
   */
  void handleRequest(
    world_model_msgs::srv::GetAreaOccupancy::Request::ConstSharedPtr /*request*/,
    world_model_msgs::srv::GetAreaOccupancy::Response::SharedPtr response)
  {
    // Cache the map -> area_frame transform once per request
    geometry_msgs::msg::TransformStamped cached_tf;
    bool tf_valid = false;

    if (map_frame_ != area_frame_) {
      try {
        cached_tf = tf_buffer_->lookupTransform(area_frame_, map_frame_, tf2::TimePointZero);
        tf_valid = true;
      } catch (const tf2::TransformException &) {
        tf_valid = false;
        return;
      }
    } else {
      tf_valid = true;
    }

    response->header.stamp = node_->get_clock()->now();
    response->header.frame_id = area_frame_;

    for (const auto & area : areas_) {
      world_model_msgs::msg::AreaOccupancyInfo area_info;
      area_info.header.stamp = response->header.stamp;
      area_info.header.frame_id = area_frame_;
      area_info.name = area.name();
      area_info.area = area.toMsg();
      area_info.is_occupied = false;

      checkEntitiesInArea<Unknown>(area, area_info, cached_tf, tf_valid);
      checkEntitiesInArea<Car>(area, area_info, cached_tf, tf_valid);
      checkEntitiesInArea<Human>(area, area_info, cached_tf, tf_valid);
      checkEntitiesInArea<Bicycle>(area, area_info, cached_tf, tf_valid);
      checkEntitiesInArea<Motorcycle>(area, area_info, cached_tf, tf_valid);
      checkEntitiesInArea<TrafficLight>(area, area_info, cached_tf, tf_valid);

      response->areas.push_back(area_info);
    }
  }

  /**
   * @brief Check whether any entities of a given type occupy a detection area.
   *
   * Iterates over all tracked entities of EntityType, transforms their pose
   * into the area frame if necessary, and tests containment. Matching entities
   * are appended to area_info.objects and area_info.is_occupied is set to true.
   *
   * @tparam EntityType The world-model entity type to check (e.g. Car, Human).
   * @param area       The detection area to test against.
   * @param area_info  Output message to populate with occupancy results.
   * @param cached_tf  Pre-looked-up transform from map frame to area frame.
   * @param tf_valid   Whether cached_tf is valid; if false, this is a no-op.
   */
  template <typename EntityType>
  void checkEntitiesInArea(
    const DetectionArea & area,
    world_model_msgs::msg::AreaOccupancyInfo & area_info,
    const geometry_msgs::msg::TransformStamped & cached_tf,
    bool tf_valid)
  {
    if (!tf_valid) {
      return;
    }

    world_state_.buffer<EntityType>().forEachConst([&](const EntityType & entity) {
      if (entity.empty()) {
        return;
      }

      geometry_msgs::msg::Point area_pos;
      if (map_frame_ == area_frame_) {
        area_pos = entity.pose().position;
      } else {
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header = entity.detection().header;
        pose_in.pose = entity.pose();
        geometry_msgs::msg::PoseStamped pose_out;
        tf2::doTransform(pose_in, pose_out, cached_tf);
        area_pos = pose_out.pose.position;
      }

      if (area.contains(area_pos.x, area_pos.y)) {
        area_info.is_occupied = true;

        world_model_msgs::msg::WorldObject obj;
        obj.header.stamp = node_->get_clock()->now();
        obj.header.frame_id = entity.frameId();
        obj.detection = entity.detection();
        obj.predictions = entity.predictions;

        if (entity.lanelet_id.has_value()) {
          double heading = extractYaw(entity.pose().orientation);
          obj.lanelet_ahead =
            lanelet_handler_->getLaneletAhead(entity.pose().position, heading, radius_m_, entity.lanelet_id);
        }

        for (const auto & det : entity.history) {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header = det.header;
          pose_stamped.pose = det.bbox.center;
          obj.history.push_back(pose_stamped);
        }

        area_info.objects.push_back(obj);
      }
    });
  }

  /**
   * @brief Parse configured occupancy areas from ROS parameters.
   *
   * Reads the "occupancy_areas" parameter for area names, then for each area
   * loads its type, center, radius, angle range, and dimensions from the
   * corresponding "occupancy_area.<name>.*" parameters.
   *
   * @return Vector of DetectionArea objects parsed from parameters.
   */
  std::vector<DetectionArea> parseOccupancyAreas()
  {
    std::vector<DetectionArea> areas;

    auto area_names = node_->get_parameter("occupancy_areas").as_string_array();

    for (const auto & area_name : area_names) {
      std::string prefix = "occupancy_area." + area_name + ".";

      std::string type_str = node_->get_parameter(prefix + "type").as_string();
      double center_x = node_->get_parameter(prefix + "center_x").as_double();
      double center_y = node_->get_parameter(prefix + "center_y").as_double();
      double radius = node_->get_parameter(prefix + "radius").as_double();
      double start_angle_deg = node_->get_parameter(prefix + "start_angle_deg").as_double();
      double end_angle_deg = node_->get_parameter(prefix + "end_angle_deg").as_double();
      double length = node_->get_parameter(prefix + "length").as_double();
      double width = node_->get_parameter(prefix + "width").as_double();

      double start_angle = start_angle_deg * M_PI / 180.0;
      double end_angle = end_angle_deg * M_PI / 180.0;

      DetectionArea::Type type;
      if (type_str == "rectangle") {
        type = DetectionArea::Type::Rectangle;
      } else if (type_str == "partial_circle") {
        type = DetectionArea::Type::PartialCircle;
      } else {
        type = DetectionArea::Type::Circle;
      }

      areas.emplace_back(area_name, type, center_x, center_y, radius, start_angle, end_angle, length, width);
    }

    return areas;
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateReader world_state_;
  tf2_ros::Buffer * tf_buffer_;
  const LaneletHandler * lanelet_handler_;
  std::string map_frame_;
  std::string area_frame_;
  double radius_m_;
  std::vector<DetectionArea> areas_;

  rclcpp::Service<world_model_msgs::srv::GetAreaOccupancy>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__GET_AREA_OCCUPANCY_SERVICE_HPP_
