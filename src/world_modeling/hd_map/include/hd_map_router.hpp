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

#ifndef WORLD_MODELING_HD_MAP_ROUTER_
#define WORLD_MODELING_HD_MAP_ROUTER_

#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "hd_map/pedestrian_reg_elem.hpp"
#include "hd_map/traffic_light_reg_elem.hpp"
#include "hd_map/traffic_sign_reg_elem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

class HDMapRouter
{
public:
  HDMapRouter();

  bool set_lanelet(const lanelet::LaneletMapPtr & lanelet_ptr);

  bool set_projector(std::shared_ptr<lanelet::Projector> projector);

  lanelet::LaneletMapPtr get_lanelet();

  lanelet::BasicPoint3d project_gps_to_point3d(lanelet::GPSPoint gps_point);
  lanelet::GPSPoint project_point3d_to_gps(lanelet::BasicPoint3d point3d);

  lanelet::ConstLanelet get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point);

  lanelet::ConstLanelet get_nearest_lanelet_to_xy(float x, float y, float width_x, float height_y);
  lanelet::ConstLanelet get_nearest_lanelet_to_xyz(float x, float y, float z);

  lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);

  lanelet::Optional<lanelet::routing::LaneletPath> route(
    lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);

  std::string get_detection3d_class(const vision_msgs::msg::Detection3D::SharedPtr reg_elem_msg_ptr);

  // Mock functions to get reg elem state/subtype
  TrafficLightState get_traffic_light_state(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);

  void process_traffic_light_msg(const vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg_ptr);
  void process_traffic_sign_msg(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
  void process_pedestrian_msg(const vision_msgs::msg::Detection3DArray::SharedPtr obstacle_msg_ptr);

  void add_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
  void add_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);
  void add_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr obstacle_msg_ptr);

  void update_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr);
  void update_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);
  void update_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr obstacle_msg_ptr);

  void remove_traffic_light(uint64_t traffic_light_id);
  void remove_pedestrian(uint64_t pedestrian_id);

private:
  lanelet::LaneletMapPtr lanelet_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  std::shared_ptr<lanelet::Projector> projector_;

  std::unordered_set<uint64_t> traffic_sign_list_;
  std::unordered_set<uint64_t> traffic_light_list_;
  std::unordered_set<uint64_t> pedestrian_list_;
};

#endif
