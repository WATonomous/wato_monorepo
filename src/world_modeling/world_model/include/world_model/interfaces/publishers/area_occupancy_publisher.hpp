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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__AREA_OCCUPANCY_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__AREA_OCCUPANCY_PUBLISHER_HPP_

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/types/detection_area.hpp"
#include "world_model_msgs/msg/area_occupancy.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"

namespace world_model
{

/**
 * @brief Publishes area-based dynamic object detection results.
 *
 * Checks configured geometric areas around ego (in base_link frame) and
 * publishes occupancy status with the objects in each area.
 */
class AreaOccupancyPublisher : public InterfaceBase
{
public:
  AreaOccupancyPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const WorldState * world_state,
    tf2_ros::Buffer * tf_buffer,
    const std::string & map_frame,
    const std::string & area_frame,
    double rate_hz,
    std::vector<DetectionArea> areas)
  : node_(node)
  , world_state_(world_state)
  , tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , area_frame_(area_frame)
  , rate_hz_(rate_hz)
  , areas_(std::move(areas))
  {
    pub_ = node_->create_publisher<world_model_msgs::msg::AreaOccupancy>("area_occupancy", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&AreaOccupancyPublisher::publish, this));
    }
  }

  void deactivate() override
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    pub_->on_deactivate();
  }

private:
  void publish()
  {
    // Get transform from map to area frame (typically base_link)
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(area_frame_, map_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;  // Transform not available yet
    }

    world_model_msgs::msg::AreaOccupancy msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = area_frame_;

    // For each area, check which objects are inside
    for (const auto & area : areas_) {
      world_model_msgs::msg::AreaOccupancyInfo area_info;
      area_info.header.stamp = msg.header.stamp;
      area_info.header.frame_id = area_frame_;
      area_info.name = area.name();
      area_info.is_occupied = false;

      // Check all entity types
      checkEntitiesInArea<Car>(area, transform, world_model_msgs::msg::DynamicObject::TYPE_CAR, area_info);
      checkEntitiesInArea<Human>(area, transform, world_model_msgs::msg::DynamicObject::TYPE_HUMAN, area_info);
      checkEntitiesInArea<Bicycle>(area, transform, world_model_msgs::msg::DynamicObject::TYPE_BICYCLE, area_info);
      checkEntitiesInArea<Motorcycle>(
        area, transform, world_model_msgs::msg::DynamicObject::TYPE_MOTORCYCLE, area_info);

      msg.areas.push_back(area_info);
    }

    pub_->publish(msg);
  }

  template <typename EntityType>
  void checkEntitiesInArea(
    const DetectionArea & area,
    const geometry_msgs::msg::TransformStamped & transform,
    uint8_t entity_type,
    world_model_msgs::msg::AreaOccupancyInfo & area_info)
  {
    auto entities = world_state_.buffer<EntityType>().getAll();
    for (const auto & entity : entities) {
      if (entity.empty()) {
        continue;
      }

      // Transform object position to area frame
      geometry_msgs::msg::Point obj_pos = entity.pose().position;
      double local_x = obj_pos.x * transform.transform.rotation.w * transform.transform.rotation.w +
                       obj_pos.y * 2 * transform.transform.rotation.w * transform.transform.rotation.z +
                       transform.transform.translation.x;
      double local_y = obj_pos.y * transform.transform.rotation.w * transform.transform.rotation.w -
                       obj_pos.x * 2 * transform.transform.rotation.w * transform.transform.rotation.z +
                       transform.transform.translation.y;

      // Simplified transform: apply full transform
      // For proper transform, we should use tf2::doTransform, but for efficiency
      // we'll use the simplified version assuming small rotation
      double tx = transform.transform.translation.x;
      double ty = transform.transform.translation.y;
      double qw = transform.transform.rotation.w;
      double qz = transform.transform.rotation.z;

      // 2D rotation: cos(theta) = qw^2 - qz^2, sin(theta) = 2*qw*qz
      double cos_theta = qw * qw - qz * qz;
      double sin_theta = 2 * qw * qz;

      local_x = cos_theta * obj_pos.x + sin_theta * obj_pos.y + tx;
      local_y = -sin_theta * obj_pos.x + cos_theta * obj_pos.y + ty;

      if (area.contains(local_x, local_y)) {
        area_info.is_occupied = true;

        // Create DynamicObject message
        world_model_msgs::msg::DynamicObject obj;
        obj.header.stamp = node_->get_clock()->now();
        obj.header.frame_id = entity.frameId();
        obj.id = entity.id();
        obj.entity_type = entity_type;
        obj.pose = entity.pose();
        obj.size = entity.size();
        obj.lanelet_id = entity.lanelet_id.value_or(-1);
        obj.detection_timestamp = entity.detection().header.stamp;
        obj.predictions = entity.predictions;

        // Populate historical path from detection history
        for (const auto & det : entity.history) {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header = det.header;
          pose_stamped.pose = det.bbox.center;
          obj.history.push_back(pose_stamped);
        }

        area_info.objects.push_back(obj);
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateReader world_state_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  std::string area_frame_;
  double rate_hz_;
  std::vector<DetectionArea> areas_;

  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::AreaOccupancy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__AREA_OCCUPANCY_PUBLISHER_HPP_
