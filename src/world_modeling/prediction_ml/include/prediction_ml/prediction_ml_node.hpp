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

#ifndef PREDICTION_ML__PREDICTION_ML_NODE_HPP_
#define PREDICTION_ML__PREDICTION_ML_NODE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "prediction_ml/mtr_runtime.hpp"
#include "prediction_ml/scene_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace prediction_ml
{

class PredictionMlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PredictionMlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PredictionMlNode() override = default;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);

  // Constant-velocity straight-line fallback (ported from simple_prediction).
  std::vector<world_model_msgs::msg::WorldObject> buildFallback(
    const vision_msgs::msg::Detection3DArray & msg) const;

  MtrConfig loadMtrConfig();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr
    world_objects_pub_;

  std::unique_ptr<SceneBuilder> scene_builder_;
  std::unique_ptr<MtrRuntime> runtime_;

  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;
  lanelet_msgs::msg::LaneletAhead::SharedPtr lanelet_ahead_;

  double prediction_horizon_{3.0};
  double prediction_time_step_{0.2};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__PREDICTION_ML_NODE_HPP_
