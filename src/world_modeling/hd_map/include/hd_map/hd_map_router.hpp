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
#include <vector>

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

  /**
  * Set the lanelet map for routing.
  *
  * @param lanelet_ptr pointer to the lanelet map
  * @return whether the lanelet was successfully set
  */
  bool set_lanelet(const lanelet::LaneletMapPtr & lanelet_ptr);

  /**
  * Set the projector to convert GPS to local XYZ coordinates.
  *
  * @param projector projector for the cordinate conversions
  * @return whether the projector was successfully set
  */
  bool set_projector(std::shared_ptr<lanelet::Projector> projector);

  /**
  * Get the current lanelet map.
  *
  * @return pointer to the lanelet map
  */
  lanelet::LaneletMapPtr get_lanelet();

  /**
  * Convert GPS coordinates to local XYZ coordinates.
  *
  * @param gps_point GPS coordinates
  * @return local XYZ corrdinates in meters
  */
  lanelet::BasicPoint3d project_gps_to_point3d(lanelet::GPSPoint gps_point);

  /**
  * Convert local XYZ coordinates to GPS coordinates.
  *
  * @param point3d the filename of the map
  * @return GPS coordinates
  */
  lanelet::GPSPoint project_point3d_to_gps(lanelet::BasicPoint3d point3d);

  /**
  * Find the nearest lanelet to a GPS point.
  *
  * @param gps_point GPS points to search for
  * @return the lanelet cloest to the given GPS point
  */
  lanelet::ConstLanelet get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point);

  /**
  * Find the nearest lanelet to the center of a bounding box.
  *
  * @param x left edge x-coordinate of the boundng box
  * @param y top edge y-coordinate of the bounding box
  * @param width_x width of the bounding box
  * @param width_y height of the bounding box
  * @return the lanelet closest to the center of the bounding box
  */
  lanelet::ConstLanelet get_nearest_lanelet_to_xy(float x, float y, float width_x, float height_y);

  /**
  * Find the nearest lanelet to a XYZ point.
  *
  * @param x the x-coordinate
  * @param y the y-coordinate
  * @param z the z-coordinate
  * @return the lanelet closest to the point
  */
  lanelet::ConstLanelet get_nearest_lanelet_to_xyz(float x, float y, float z);

  /**
  * Calculate a route between two GPS points.
  *
  * @param from_point starting GPS coordinates
  * @param to_point destination GPS coordinates
  * @return the shortest path between the points, or empty if no route exists
  */
  lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);

  /**
  * Calculate a route between two lanelets.
  *
  * @param from_lanelet starting lanelet
  * @param to_lanelet destination lanelet
  * @return the shortest path between the lanelets, or empty if no route exists
  */
  lanelet::Optional<lanelet::routing::LaneletPath> route(
    lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);

  /**
  * Get the class ID from a 3D detection message.
  *
  * @param reg_elem_msg_ptr 3D detection message
  * @return the class ID with the highest confidence score
  */
  std::string get_detection3d_class(const vision_msgs::msg::Detection3D::SharedPtr reg_elem_msg_ptr);

  /**
  * Get the current state of a Traffic Light.
  *
  * @param traffic_light_msg_ptr the traffic light regulatory element
  * @return the traffic light state, either red, yellow or green
  */
  TrafficLightState get_traffic_light_state(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr);

  /*
  Lightweight representations of various map objects for the GetLaneObjects service.
  Includes:
    -Lane semantics
    -Traffic signs
    -Crosswalks
    -Bike lanes
  */

  // 0 is the leftmost lane, and the speed limit is in m/s
  struct LaneSemantic
  {
    lanelet::Id lane_id{lanelet::InvalId};
    int32_t lane_index{0};
    int32_t lane_count{1};
    double speed_limit{0.0};
    bool has_left_neighbor{false};
    bool has_right_neighbor{false};
    bool in_intersection{false};
  };

  // pose is for the position and orientation of the traffic sign (processed from the lanelet map)
  // type is for the type of sign, ex. stop, yield, speed limits
  struct LaneObjectTrafficSign
  {
    geometry_msgs::msg::Pose pose;
    std::string type;
  };

  struct LaneObjectCrosswalk
  {
    geometry_msgs::msg::Pose pose;
  };

  struct LaneObjectBikeLane
  {
    geometry_msgs::msg::Pose pose;
  };

  struct LaneObjects
  {
    std::vector<LaneObjectTrafficSign> traffic_signs;
    std::vector<LaneObjectCrosswalk> crosswalks;
    std::vector<LaneObjectBikeLane> bike_lanes;
  };

  /**
  * Compute semantic attributes for a lanelet.
  *
  * @param lanelet lanelet to analyze
  * @return lane semantic summary
  */
  LaneSemantic get_lane_semantic(const lanelet::ConstLanelet & lanelet) const;

  /**
  * Collect lane-related objects along a forward path along the routing graph.
  *
  * @param start_lanelet starting lanelet for the corridor
  * @param distance_ahead lookahead distance along the route [m]
  * @param lateral_radius max lateral distance from corridor centerlines [m]
  * @return lane objects found along the corridor
  */
  LaneObjects get_lane_objects_along_corridor(
    const lanelet::ConstLanelet & start_lanelet, double distance_ahead, double lateral_radius) const;

  /**
  * Build a sequence of lanelets ahead of a start lanelet.
  *
  * @param start starting lanelet
  * @param lookahead distance to traverse forward [m]
  * @return ordered lanelet sequence along the route
  */
  std::vector<lanelet::ConstLanelet> collect_lane_sequence_ahead(
    const lanelet::ConstLanelet & start, double lookahead) const;

private:
  lanelet::LaneletMapPtr lanelet_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  std::shared_ptr<lanelet::Projector> projector_;

  std::unordered_set<uint64_t> traffic_sign_list_;
  std::unordered_set<uint64_t> traffic_light_list_;
  std::unordered_set<uint64_t> pedestrian_list_;
};

#endif
