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

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace behaviour_markers
{

  class BehaviourMarkersNode : public rclcpp::Node
  {
  public:
    BehaviourMarkersNode();

  private:
    void executeBehaviourCallback(const behaviour_msgs::msg::ExecuteBehaviour::SharedPtr msg);
    void routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg);
    void laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);
    void laneContextCallback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);

    void cacheLanelets(const std::vector<lanelet_msgs::msg::Lanelet> &lanelets);
    void publishMarkers();

    static std::string toLower(std::string text);
    std_msgs::msg::ColorRGBA colorForBehaviour(const std::string &behaviour) const;

    rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr execute_behaviour_sub_;
    rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr route_ahead_sub_;
    rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
    rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr lane_context_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> lanelet_cache_;
    lanelet_msgs::msg::Lanelet current_lanelet_;

    behaviour_msgs::msg::ExecuteBehaviour latest_execute_behaviour_;
    bool has_latest_execute_behaviour_{false};
    bool has_current_lane_{false};

    // Visual parameters
    std::string frame_id_;
    std::string frame_hint_;
    double preferred_centerline_line_width_{0.45};
    double preferred_boundary_line_width_{0.18};
    double current_lane_line_width_{0.2};
    double text_height_{0.8};
    bool show_boundaries_{true};
    bool show_lanelet_ids_{false};
    bool show_current_lane_{true};
    bool show_info_{true};
  };

} // namespace behaviour_markers

#endif // BEHAVIOUR_MARKERS__BEHAVIOUR_MARKERS_NODE_HPP_
