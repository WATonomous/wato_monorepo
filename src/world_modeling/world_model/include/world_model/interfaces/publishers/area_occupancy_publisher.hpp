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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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
  , timer_cb_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
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
        std::bind(&AreaOccupancyPublisher::publish, this),
        timer_cb_group_);
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
  /**
   * @brief Timer callback that evaluates all detection areas and publishes occupancy.
   *
   * Caches the map-to-area-frame transform once, then iterates over all configured
   * detection areas, checking which entities fall within each area's geometry.
   */
  void publish()
  {
    // Cache the map → area_frame transform once per publish cycle
    if (map_frame_ != area_frame_) {
      try {
        cached_tf_ = tf_buffer_->lookupTransform(area_frame_, map_frame_, tf2::TimePointZero);
        tf_valid_ = true;
      } catch (const tf2::TransformException &) {
        tf_valid_ = false;
        return;
      }
    } else {
      tf_valid_ = true;
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
      area_info.area = area.toMsg();
      area_info.is_occupied = false;

      // Check all entity types
      checkEntitiesInArea<Unknown>(area, world_model_msgs::msg::DynamicObject::TYPE_UNKNOWN, area_info);
      checkEntitiesInArea<Car>(area, world_model_msgs::msg::DynamicObject::TYPE_CAR, area_info);
      checkEntitiesInArea<Human>(area, world_model_msgs::msg::DynamicObject::TYPE_HUMAN, area_info);
      checkEntitiesInArea<Bicycle>(area, world_model_msgs::msg::DynamicObject::TYPE_BICYCLE, area_info);
      checkEntitiesInArea<Motorcycle>(
        area, world_model_msgs::msg::DynamicObject::TYPE_MOTORCYCLE, area_info);
      checkEntitiesInArea<TrafficLight>(
        area, world_model_msgs::msg::DynamicObject::TYPE_TRAFFIC_LIGHT, area_info);

      msg.areas.push_back(area_info);
    }

    pub_->publish(msg);
  }

  /**
   * @brief Checks all entities of a given type for containment within a detection area.
   *
   * Transforms each entity's position from map frame to the area frame using the
   * cached transform, then tests containment against the area geometry. Matching
   * entities are added to the area_info output.
   *
   * @tparam EntityType Entity class (e.g. Car, Human, Bicycle).
   * @param area Detection area to test against.
   * @param entity_type DynamicObject type constant for the output message.
   * @param area_info Output area info to populate with matching objects.
   */
  template <typename EntityType>
  void checkEntitiesInArea(
    const DetectionArea & area,
    uint8_t entity_type,
    world_model_msgs::msg::AreaOccupancyInfo & area_info)
  {
    world_state_.buffer<EntityType>().forEachConst(
      [&](const EntityType & entity) {
        if (entity.empty()) {
          return;
        }

        // Transform entity pose from map frame to area frame for containment check
        geometry_msgs::msg::Point area_pos;
        if (map_frame_ == area_frame_) {
          area_pos = entity.pose().position;
        } else {
          geometry_msgs::msg::PoseStamped pose_in;
          pose_in.header = entity.detection().header;
          pose_in.pose = entity.pose();
          geometry_msgs::msg::PoseStamped pose_out;
          tf2::doTransform(pose_in, pose_out, cached_tf_);
          area_pos = pose_out.pose.position;
        }

        if (area.contains(area_pos.x, area_pos.y)) {
          area_info.is_occupied = true;

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

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateReader world_state_;
  tf2_ros::Buffer * tf_buffer_;
  std::string map_frame_;
  std::string area_frame_;
  double rate_hz_;
  std::vector<DetectionArea> areas_;

  geometry_msgs::msg::TransformStamped cached_tf_;
  bool tf_valid_{false};

  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::AreaOccupancy>::SharedPtr pub_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__AREA_OCCUPANCY_PUBLISHER_HPP_
