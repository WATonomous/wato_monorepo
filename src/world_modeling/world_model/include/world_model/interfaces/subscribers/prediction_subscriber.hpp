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

#ifndef WORLD_MODEL__INTERFACES__SUBSCRIBERS__PREDICTION_SUBSCRIBER_HPP_
#define WORLD_MODEL__INTERFACES__SUBSCRIBERS__PREDICTION_SUBSCRIBER_HPP_

#include "prediction_msgs/msg/prediction_hypotheses_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Subscribes to prediction hypotheses and updates entity predictions.
 *
 * Associates predicted trajectories with tracked entities by ID.
 */
class PredictionSubscriber : public InterfaceBase
{
public:
  PredictionSubscriber(
    rclcpp_lifecycle::LifecycleNode * node,
    WorldState * world_state)
  : node_(node),
    world_state_(world_state)
  {
    sub_ = node_->create_subscription<prediction_msgs::msg::PredictionHypothesesArray>(
      "predictions", 10,
      std::bind(&PredictionSubscriber::onMessage, this, std::placeholders::_1));
  }

private:
  void onMessage(prediction_msgs::msg::PredictionHypothesesArray::ConstSharedPtr msg)
  {
    for (const auto & pred_h : msg->pred_h_arr) {
      int64_t id = pred_h.id;

      // Try each buffer - predictions can apply to any 3D entity type
      if (tryUpdatePredictions<Car>(id, pred_h.preds)) continue;
      if (tryUpdatePredictions<Human>(id, pred_h.preds)) continue;
      if (tryUpdatePredictions<Bicycle>(id, pred_h.preds)) continue;
      if (tryUpdatePredictions<Motorcycle>(id, pred_h.preds)) continue;
    }
  }

  template<typename EntityT>
  bool tryUpdatePredictions(
    int64_t id,
    const std::vector<prediction_msgs::msg::Prediction> & preds)
  {
    auto & buffer = world_state_.buffer<EntityT>();
    return buffer.modify(id, [&preds](EntityT & entity) {
      entity.predictions = preds;
    });
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  WorldStateWriter world_state_;

  rclcpp::Subscription<prediction_msgs::msg::PredictionHypothesesArray>::SharedPtr sub_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SUBSCRIBERS__PREDICTION_SUBSCRIBER_HPP_
