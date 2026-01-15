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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

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

  // store in member
  this->traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::instance().create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  // build routing graph using the member
  this->routing_graph_ = lanelet::routing::RoutingGraph::build(*this->lanelet_ptr_, *this->traffic_rules_);

  errors = this->routing_graph_->checkValidity();
  if (!errors.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Assigning Routing Graph... Failed");
    for (const auto & error : errors) {
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

// TODO(wato): populate the detection attribute in the reg elem and return the state/type from the detection
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

// helper function to calculate 2D length of a polyline (for the forward path distance)
namespace
{
double polylineLength2d(const lanelet::BasicLineString2d & line)
{
  double len = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto & a = line[i];
    const auto & b = line[i + 1];
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}
}  // namespace

// converts a 3D line string into 2D. For ease of calculations and distance calculations are written in 2D.
static lanelet::BasicLineString2d toBasic2d(const lanelet::ConstLineString3d & ls)
{
  lanelet::BasicLineString2d line;
  line.reserve(ls.size());
  for (const auto & p : ls) {
    line.push_back(p.basicPoint2d());
  }
  return line;
}

std::vector<lanelet::ConstLanelet> HDMapRouter::collect_lane_sequence_ahead(
  const lanelet::ConstLanelet & start, double lookahead) const
{
  std::vector<lanelet::ConstLanelet> seq;

  // guard case against non-initialized routing graph
  if (!routing_graph_) {
    seq.push_back(start);
    return seq;
  }

  lanelet::ConstLanelet current = start;
  double accumulated = 0.0;

  // loop guard on max iteration count
  const size_t kMaxSteps = 1000;
  size_t steps = 0;

  while (steps < kMaxSteps) {
    ++steps;
    seq.push_back(current);

    const auto centerline = toBasic2d(current.centerline());
    const double len = polylineLength2d(centerline);
    accumulated += len;
    if (accumulated >= lookahead) {
      break;
    }

    auto nexts = routing_graph_->following(current);
    if (nexts.empty()) {
      break;
    }
    current = nexts.front();
  }

  return seq;
}

HDMapRouter::LaneSemantic HDMapRouter::get_lane_semantic(const lanelet::ConstLanelet & lanelet) const
{
  LaneSemantic sem;
  sem.lane_id = lanelet.id();

  const std::string location = lanelet.attributeOr("location", "");
  const std::string subtype = lanelet.attributeOr("subtype", "");
  sem.in_intersection = (location == "intersection") || (subtype == "intersection");

  // gets the speed limit (if it exists) from traffic rules (check if m/s or km/h)
  if (traffic_rules_) {
    auto lim = traffic_rules_->speedLimit(lanelet);
    sem.speed_limit = lim.speedLimit.value();
  }

  // fallback if the routing graph isn't built yet
  if (!routing_graph_) {
    sem.lane_index = 0;
    sem.lane_count = 1;
    sem.has_left_neighbor = false;
    sem.has_right_neighbor = false;
    return sem;
  }

  // counting the number of lanes to the left
  int left_count = 0;
  lanelet::ConstLanelet cur = lanelet;
  while (true) {
    auto left_neighbor = routing_graph_->left(cur);
    if (!left_neighbor) break;
    cur = *left_neighbor;
    ++left_count;
  }

  // counting the number of lanes to the right
  int right_count = 0;
  cur = lanelet;
  while (true) {
    auto right_neighbor = routing_graph_->right(cur);
    if (!right_neighbor) break;
    cur = *right_neighbor;
    ++right_count;
  }

  sem.lane_index = left_count;
  sem.lane_count = left_count + 1 + right_count;
  sem.has_left_neighbor = left_count > 0;
  sem.has_right_neighbor = right_count > 0;

  return sem;
}

// helper function to convert a point to a pose
static geometry_msgs::msg::Pose basicPointToPose(const lanelet::BasicPoint2d & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

HDMapRouter::LaneObjects HDMapRouter::get_lane_objects_along_corridor(
  const lanelet::ConstLanelet & start_lanelet, double distance_ahead, double lateral_radius) const
{
  LaneObjects lane_objects;

  // fall back for if the map isnt loaded
  if (!lanelet_ptr_) {
    return lane_objects;
  }

  auto seq = collect_lane_sequence_ahead(start_lanelet, distance_ahead);

  // block that extracts all traffic signs from the lanelets
  for (const auto & lanelet : seq) {
    for (const auto & reg : lanelet.regulatoryElements()) {
      const std::string subtype = reg->attributeOr("subtype", "");

      if (subtype != "traffic_sign") {
        continue;
      }

      LaneObjectTrafficSign sign;

      // approximates the sign position using the centerline of the lanelet, so the sign position is actually placed
      // on the center instead of the roadside
      if (!lanelet.centerline().empty()) {
        const auto & point3d = lanelet.centerline().front();
        const auto point2d = point3d.basicPoint2d();
        sign.pose = basicPointToPose(point2d);
      } else {
        // fallback pose if centerline is empty
        sign.pose = geometry_msgs::msg::Pose{};
        sign.pose.orientation.w = 1.0;
      }

      sign.type = reg->attributeOr("type", subtype);
      lane_objects.traffic_signs.push_back(sign);
    }
  }

  // block that finds all the bike lanes or crosswalks
  // loops through all lanelets objects, searches for crosswalk/bike‑lane, computes a representative point, and filters by lateral distance to the route seq
  const double lateral_radius_sq = lateral_radius * lateral_radius;

  for (const auto & lanelet : lanelet_ptr_->laneletLayer) {
    const std::string subtype = lanelet.attributeOr("subtype", "");

    // to cover alternative naming conventions in maps
    const bool is_crosswalk = (subtype == "crosswalk") || (subtype == "zebra_crossing");
    const bool is_bike_lane = (subtype == "bicycle_lane") || (subtype == "bike_lane");

    if (!is_crosswalk && !is_bike_lane) {
      continue;
    }

    const auto centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }

    // calculates a singular point as a represenative of these elements; mainly to signal if a cross walk/bike lane is nearby
    lanelet::BasicPoint2d object_point = centerline.front().basicPoint2d();
    if (centerline.size() > 1) {
      auto end_pt = centerline.back().basicPoint2d();
      object_point.x() = 0.5 * (object_point.x() + end_pt.x());
      object_point.y() = 0.5 * (object_point.y() + end_pt.y());
    }

    // calculates the min distance squared of the found object to any lanelet in the route within the radius
    double min_dist_sq = std::numeric_limits<double>::infinity();
    for (const auto & route_lanelet : seq) {
      const auto route_centerline = route_lanelet.centerline();
      for (const auto & point3d : route_centerline) {
        const auto p = point3d.basicPoint2d();
        const double dx = p.x() - object_point.x();
        const double dy = p.y() - object_point.y();
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
        }
      }
    }

    // outside of the radius
    if (min_dist_sq > lateral_radius_sq) {
      continue;
    }

    if (is_crosswalk) {
      LaneObjectCrosswalk crosswalk;
      crosswalk.pose = basicPointToPose(object_point);
      lane_objects.crosswalks.push_back(crosswalk);
    }
    if (is_bike_lane) {
      LaneObjectBikeLane bike_lane;
      bike_lane.pose = basicPointToPose(object_point);
      lane_objects.bike_lanes.push_back(bike_lane);
    }
  }

  return lane_objects;
}
