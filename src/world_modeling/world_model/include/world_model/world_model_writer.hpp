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

#ifndef WORLD_MODEL__WORLD_MODEL_WRITER_HPP_
#define WORLD_MODEL__WORLD_MODEL_WRITER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "world_model/pipeline/entity_permanence.hpp"
#include "world_model/pipeline/entity_populator.hpp"
#include "world_model/pipeline/lanelet_enricher.hpp"
#include "world_model/world_state.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace world_model
{

/**
 * @brief Single inbound pipeline that writes to the world model.
 *
 * Subscribes to WorldObjectArray messages, classifies each object
 * by type, and runs three stages inside a single COW batch per type:
 *   1. Populate   — upsert detections, store predictions
 *   2. Enrich     — set lanelet context on each entity
 *   3. Permanence — trim history, prune stale entities
 */
class WorldModelWriter
{
public:
  WorldModelWriter(
    rclcpp_lifecycle::LifecycleNode * node,
    WorldState * world_state,
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer)
  : node_(node)
  , world_state_(world_state)
  , clock_(node->get_clock())
  , enricher_(
      lanelet_handler,
      tf_buffer,
      node->get_parameter("map_frame").as_string(),
      node->declare_parameter<double>("enricher_route_priority_threshold_m", 10.0),
      node->declare_parameter<double>("enricher_heading_search_radius_m", 15.0))
  , permanence_(
      node->declare_parameter<double>("entity_history_duration_sec", 5.0),
      node->declare_parameter<double>("entity_prune_timeout_sec", 2.0))
  , populator_(node->declare_parameter<int64_t>("traffic_light_state_hypothesis_index", 1))
  , hypothesis_idx_(node->declare_parameter<int64_t>("entity_class_hypothesis_index", 0))
  , cb_group_(node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cb_group_;
    sub_ = node_->create_subscription<world_model_msgs::msg::WorldObjectArray>(
      "world_object_seeds", 10, std::bind(&WorldModelWriter::onMessage, this, std::placeholders::_1), opts);
  }

private:
  using ObjectPtrs = std::vector<const world_model_msgs::msg::WorldObject *>;

  /**
   * @brief Subscription callback for incoming WorldObjectArray messages.
   *
   * Classifies each detection by entity type, then runs the three-stage
   * pipeline (populate, enrich, permanence) for each type in a single COW batch.
   * Uses the message timestamp for permanence checks to avoid sim-time mismatches.
   *
   * @param msg Incoming array of world objects with detections and predictions.
   */
  void onMessage(world_model_msgs::msg::WorldObjectArray::ConstSharedPtr msg)
  {
    ObjectPtrs unknowns, cars, humans, bicycles, motorcycles, traffic_lights;

    for (const auto & obj : msg->objects) {
      switch (classify(obj.detection)) {
        case EntityType::UNKNOWN:
          unknowns.push_back(&obj);
          break;
        case EntityType::CAR:
          cars.push_back(&obj);
          break;
        case EntityType::HUMAN:
          humans.push_back(&obj);
          break;
        case EntityType::BICYCLE:
          bicycles.push_back(&obj);
          break;
        case EntityType::MOTORCYCLE:
          motorcycles.push_back(&obj);
          break;
        case EntityType::TRAFFIC_LIGHT:
          traffic_lights.push_back(&obj);
          break;
      }
    }

    RCLCPP_DEBUG(
      node_->get_logger(),
      "onMessage: %zu objects -> cars=%zu tl=%zu unknown=%zu",
      msg->objects.size(), cars.size(), traffic_lights.size(), unknowns.size());

    // Use the message timestamp for permanence checks instead of the node clock.
    // This avoids sim-time vs wall-time mismatches that would cause entities to be
    // instantly pruned when use_sim_time is not configured.
    auto now =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? clock_->now() : rclcpp::Time(msg->header.stamp);
    enricher_.updateTransform(msg->header.frame_id);

    runPipeline<Unknown>(unknowns, msg->header, now);
    runPipeline<Car>(cars, msg->header, now);
    runPipeline<Human>(humans, msg->header, now);
    runPipeline<Bicycle>(bicycles, msg->header, now);
    runPipeline<Motorcycle>(motorcycles, msg->header, now);
    runPipeline<TrafficLight>(traffic_lights, msg->header, now);

  }

  /**
   * @brief Run the pipeline for one entity type.
   *
   * When objects are present, all three stages run inside a single COW batch.
   * When empty, only permanence runs — but skips the lock if the buffer
   * is already empty (nothing to prune).
   */
  template <typename EntityT>
  void runPipeline(const ObjectPtrs & objects, const std_msgs::msg::Header & msg_header, const rclcpp::Time & now)
  {
    auto & buffer = world_state_.buffer<EntityT>();

    if (!objects.empty()) {
      buffer.batch([&](std::unordered_map<std::string, EntityT> & map) {
        populator_.populate(map, objects, msg_header);
        enricher_.enrich(map);
        permanence_.apply(map, now);
      });
    } else if (!buffer.empty()) {
      buffer.batch([&](std::unordered_map<std::string, EntityT> & map) { permanence_.apply(map, now); });
    }
  }

  /**
   * @brief Classify a detection into an entity type based on its class ID.
   *
   * Maps class ID strings (e.g. "car", "pedestrian", "bicycle") to EntityType
   * enum values. Returns UNKNOWN for unrecognized or empty class IDs.
   *
   * @param det Detection containing classification results.
   * @return Corresponding EntityType for the detection's class ID.
   */
  EntityType classify(const vision_msgs::msg::Detection3D & det) const
  {
    if (det.results.empty() || static_cast<size_t>(hypothesis_idx_) >= det.results.size()) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "classify: det id='%s' has %zu results, need idx %ld -> UNKNOWN",
        det.id.c_str(), det.results.size(), hypothesis_idx_);
      return EntityType::UNKNOWN;
    }

    const std::string & class_id = det.results[hypothesis_idx_].hypothesis.class_id;

    if (class_id == "car" || class_id == "vehicle" || class_id == "truck") {
      return EntityType::CAR;
    } else if (class_id == "person" || class_id == "pedestrian" || class_id == "human") {
      return EntityType::HUMAN;
    } else if (class_id == "bicycle" || class_id == "cyclist") {
      return EntityType::BICYCLE;
    } else if (class_id == "motorcycle" || class_id == "motorbike") {
      return EntityType::MOTORCYCLE;
    } else if (class_id == "traffic_light") {
      return EntityType::TRAFFIC_LIGHT;
    }

    RCLCPP_DEBUG(
      node_->get_logger(),
      "classify: det id='%s' unrecognized class_id='%s' at idx %ld -> UNKNOWN",
      det.id.c_str(), class_id.c_str(), hypothesis_idx_);
    return EntityType::UNKNOWN;
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateWriter world_state_;
  rclcpp::Clock::SharedPtr clock_;

  // Pipeline stages
  EntityPopulator populator_;
  LaneletEnricher enricher_;
  EntityPermanence permanence_;
  int64_t hypothesis_idx_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<world_model_msgs::msg::WorldObjectArray>::SharedPtr sub_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__WORLD_MODEL_WRITER_HPP_
