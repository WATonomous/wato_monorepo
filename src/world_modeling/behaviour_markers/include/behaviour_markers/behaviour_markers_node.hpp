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

#ifndef BEHAVIOUR_MARKERS__BEHAVIOUR_MARKERS_NODE_HPP_
#define BEHAVIOUR_MARKERS__BEHAVIOUR_MARKERS_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace behaviour_markers
{

  /**
   * @brief Visualizes behaviour tree decisions.
   *
   * Subscribes to execute_behaviour to get the current behaviour and preferred lanelets,
   * and to route_ahead to get lanelet geometry. Displays:
   * - Text marker above ego showing current behaviour
   * - Centerlines for preferred lanelets
   */
  class BehaviourMarkersNode : public rclcpp::Node
  {
  public:
    BehaviourMarkersNode();

  private:
    void executeBehaviourCallback(const behaviour_msgs::msg::ExecuteBehaviour::SharedPtr msg);
    void laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);
    void publishMarkers();

    // Subscribers
    rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr execute_behaviour_sub_;
    rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    // TF for ego position
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Cached lanelet geometry from lanelet_ahead (id -> lanelet)
    std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> ahead_lanelets_;

    // Latest behaviour message
    behaviour_msgs::msg::ExecuteBehaviour latest_behaviour_;
    bool has_behaviour_{false};

    // Parameters
    std::string map_frame_;
    std::string base_frame_;
    double centerline_width_{0.4};
    double text_height_{1.0};
    double text_offset_z_{3.0};
  };

} // namespace behaviour_markers

#endif // BEHAVIOUR_MARKERS__BEHAVIOUR_MARKERS_NODE_HPP_
