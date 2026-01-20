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

#ifndef WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define WORLD_MODEL__WORLD_MODEL_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "prediction_msgs/msg/prediction_hypotheses_array.hpp"
#include "lanelet_msgs/msg/map_visualization.hpp"
#include "lanelet_msgs/srv/get_corridor.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"
#include "lanelet_msgs/srv/get_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model/agent_tracker.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model/traffic_light_tracker.hpp"

namespace world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
  explicit WorldModelNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WorldModelNode() = default;

private:
  // Components
  std::unique_ptr<LaneletHandler> lanelet_handler_;
  std::unique_ptr<AgentTracker> agent_tracker_;
  std::unique_ptr<TrafficLightTracker> tl_tracker_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Services
  rclcpp::Service<lanelet_msgs::srv::GetRoute>::SharedPtr route_srv_;
  rclcpp::Service<lanelet_msgs::srv::GetCorridor>::SharedPtr corridor_srv_;
  rclcpp::Service<lanelet_msgs::srv::GetLaneletsByRegElem>::SharedPtr reg_elem_srv_;

  // Publishers
  rclcpp::Publisher<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr lane_context_pub_;
  rclcpp::Publisher<lanelet_msgs::msg::MapVisualization>::SharedPtr map_viz_pub_;

  // Subscriptions
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr tl_detections_sub_;
  rclcpp::Subscription<prediction_msgs::msg::PredictionHypothesesArray>::SharedPtr predictions_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr lane_context_timer_;
  rclcpp::TimerBase::SharedPtr map_viz_timer_;

  // State
  geometry_msgs::msg::PoseStamped current_ego_pose_;
  bool ego_pose_received_;

  // Parameters
  std::string map_frame_;
  std::string base_link_frame_;
  double map_viz_radius_m_;

  // Service callbacks
  void handleGetRoute(
    const std::shared_ptr<lanelet_msgs::srv::GetRoute::Request> request,
    std::shared_ptr<lanelet_msgs::srv::GetRoute::Response> response);

  void handleGetCorridor(
    const std::shared_ptr<lanelet_msgs::srv::GetCorridor::Request> request,
    std::shared_ptr<lanelet_msgs::srv::GetCorridor::Response> response);

  void handleGetLaneletsByRegElem(
    const std::shared_ptr<lanelet_msgs::srv::GetLaneletsByRegElem::Request> request,
    std::shared_ptr<lanelet_msgs::srv::GetLaneletsByRegElem::Response> response);

  // Subscription callbacks
  void detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  void tlDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void predictionsCallback(const prediction_msgs::msg::PredictionHypothesesArray::SharedPtr msg);
  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Timer callbacks
  void publishLaneContext();
  void publishMapVisualization();

  // Helper methods
  lanelet_msgs::msg::CurrentLaneContext buildLaneContext();
  bool getEgoPoseFromTF(geometry_msgs::msg::PoseStamped & pose);
};

}  // namespace world_model

#endif  // WORLD_MODEL__WORLD_MODEL_NODE_HPP_
