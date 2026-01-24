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

#ifndef WORLD_MODEL__INTERFACES__SUBSCRIBERS__TRAFFIC_LIGHT_SUBSCRIBER_HPP_
#define WORLD_MODEL__INTERFACES__SUBSCRIBERS__TRAFFIC_LIGHT_SUBSCRIBER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Subscribes to 2D traffic light detections and updates WorldState.
 *
 * Classifies traffic light state and updates the traffic light buffer.
 */
class TrafficLightSubscriber : public InterfaceBase
{
public:
  TrafficLightSubscriber(rclcpp_lifecycle::LifecycleNode * node, WorldState * world_state)
  : node_(node)
  , world_state_(world_state)
  {
    sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      "traffic_light_detections", 10, std::bind(&TrafficLightSubscriber::onMessage, this, std::placeholders::_1));
  }

private:
  void onMessage(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    auto & buffer = world_state_.buffer<TrafficLight>();

    for (const auto & det : msg->detections) {
      int64_t id = std::stoll(det.id);

      TrafficLight default_tl;
      buffer.upsert(id, default_tl, [&det, this](TrafficLight & tl) {
        tl.history.push_front(det);

        // Keep limited history for traffic lights
        while (tl.history.size() > 10) {
          tl.history.pop_back();
        }

        // Classify state
        tl.state = classifyState(det);
        if (!det.results.empty()) {
          tl.confidence = det.results[0].hypothesis.score;
        }
      });
    }
  }

  TrafficLightState classifyState(const vision_msgs::msg::Detection2D & det) const
  {
    if (det.results.empty()) {
      return TrafficLightState::UNKNOWN;
    }

    const std::string & class_id = det.results[0].hypothesis.class_id;

    if (class_id == "red" || class_id == "RED") {
      return TrafficLightState::RED;
    } else if (class_id == "yellow" || class_id == "YELLOW" || class_id == "amber") {
      return TrafficLightState::YELLOW;
    } else if (class_id == "green" || class_id == "GREEN") {
      return TrafficLightState::GREEN;
    }

    return TrafficLightState::UNKNOWN;
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateWriter world_state_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SUBSCRIBERS__TRAFFIC_LIGHT_SUBSCRIBER_HPP_
