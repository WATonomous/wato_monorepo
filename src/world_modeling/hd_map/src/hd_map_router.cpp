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

#include "hd_map/hd_map_router.hpp"

#include <lanelet2_core/geometry/BoundingBox.h>

#include <map>
#include <set>
#include <string>

#include "hd_map/utils.hpp"

HDMapRouter::HDMapRouter()
{}

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
  lanelet::GPSPoint gpsPoint = lanelet::GPSPoint();
  gpsPoint.lat = gps_msg->latitude;
  gpsPoint.lon = gps_msg->longitude;
  gpsPoint.ele = gps_msg->altitude;

  return gpsPoint;
}

bool HDMapRouter::set_lanelet(const lanelet::LaneletMapPtr & lanelet_ptr)
{
  lanelet::routing::RoutingGraph::Errors errors;
  this->lanelet_ptr_ = lanelet_ptr;

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
  lanelet::routing::RoutingGraphPtr routing_graph =
    lanelet::routing::RoutingGraph::build(*this->lanelet_ptr_, *traffic_rules);

  this->routing_graph_ = routing_graph;

  errors = routing_graph_->checkValidity();
  if (errors.empty() != true) {
    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Assigning Routing Graph... Failed");
    for (auto error : errors) {
      RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Routing Graph Build Error : %s", error.c_str());
    }
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Building Traffic Rules Factory... Success");
  RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Assigning Routing Graph... Success");

  return true;
}

lanelet::LaneletMapPtr HDMapRouter::get_lanelet()
{
  return this->lanelet_ptr_;
}

bool HDMapRouter::set_projector(std::shared_ptr<lanelet::Projector> projector)
{
  this->projector_ = projector;
  return true;
}

lanelet::BasicPoint3d HDMapRouter::project_gps_to_point3d(lanelet::GPSPoint gps_point)
{
  return projector_->forward(gps_point);
}

lanelet::GPSPoint HDMapRouter::project_point3d_to_gps(lanelet::BasicPoint3d point3d)
{
  return projector_->reverse(point3d);
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point)
{
  auto search_point = project_gps_to_point3d(gps_point);
  double min_distance = __DBL_MAX__;
  lanelet::ConstLanelet nearest_lanelet;

  for (const auto & lanelet : lanelet_ptr_->laneletLayer) {
    double distance = lanelet::geometry::distanceToCenterline3d(lanelet, search_point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_lanelet = lanelet;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Found closest lanelet to GPS Point!");
  return nearest_lanelet;
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_xy(float x, float y, float width_x, float height_y)
{
  float x_center, y_center;

  x_center = (x + (0.5) * width_x);
  y_center = (y - (0.5) * height_y);

  lanelet::BasicPoint3d obstacle_center_xy(x_center, y_center, 0);
  lanelet::GPSPoint local_coordinates_xy = projector_->reverse(obstacle_center_xy);

  return get_nearest_lanelet_to_gps(local_coordinates_xy);
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_xyz(float x_center, float y_center, float z_center)
{
  lanelet::BasicPoint3d obstacle_center_xyz(x_center, y_center, z_center);
  lanelet::GPSPoint local_coordinates_xyz = projector_->reverse(obstacle_center_xyz);

  return get_nearest_lanelet_to_gps(local_coordinates_xyz);
}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(
  lanelet::GPSPoint from_point, lanelet::GPSPoint to_point)
{
  return route(get_nearest_lanelet_to_gps(from_point), get_nearest_lanelet_to_gps(to_point));
}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(
  lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet)
{
  auto route = this->routing_graph_->getRoute(from_lanelet, to_lanelet, 0);
  auto shortest_path = route->shortestPath();
  return shortest_path;
}

// TODO(wato): populate the detection attribute in the reg elem and return the state/type from the
// detection
std::string HDMapRouter::get_detection3d_class(const vision_msgs::msg::Detection3D::SharedPtr reg_elem_msg_ptr)
{
  std::string class_id = "";
  float base_score = 0;
  for (const auto & result : reg_elem_msg_ptr->results) {
    if (result.hypothesis.score > base_score) {
      class_id = result.hypothesis.class_id;
    }
  }
  return class_id;
}

// Mock function to get traffic light state (to change or remove)
TrafficLightState HDMapRouter::get_traffic_light_state(
  const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr)
{
  if (traffic_light_msg_ptr->results.empty()) {
    return TrafficLightState::UnknownLight;
  }

  TrafficLightState traffic_light_state = TrafficLightState::UnknownLight;
  float base_score = 0;
  for (const auto & result : traffic_light_msg_ptr->results) {
    if (result.hypothesis.score > base_score) {
      if (result.hypothesis.class_id == "GREEN") {
        traffic_light_state = TrafficLightState::Green;
      } else if (result.hypothesis.class_id == "YELLOW") {
        traffic_light_state = TrafficLightState::Yellow;
      } else if (result.hypothesis.class_id == "RED") {
        traffic_light_state = TrafficLightState::Red;
      }
      base_score = result.hypothesis.score;
    }
  }

  return traffic_light_state;
}
