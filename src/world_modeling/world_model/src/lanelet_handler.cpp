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

#include "world_model/lanelet_handler.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace world_model
{

bool LaneletHandler::loadMap(
  const std::string & osm_path, double utm_origin_x, double utm_origin_y, const std::string & projector_type)
{
  try {
    if (projector_type == "local_cartesian") {
      // Use LocalCartesianProjector for CARLA-style maps with local coordinates
      lanelet::Origin origin({0.0, 0.0});
      projector_ = std::make_unique<lanelet::projection::LocalCartesianProjector>(origin);
      map_ = lanelet::load(osm_path, *projector_);
    } else {
      // Default: UTM projector with origin offset
      // First load OSM to get reference GPS coordinate for UTM zone
      auto temp_projector = lanelet::projection::UtmProjector(lanelet::Origin({0, 0}));
      auto temp_map = lanelet::load(osm_path, temp_projector);

      if (temp_map->pointLayer.empty()) {
        return false;
      }

      // Get first point to determine UTM zone
      auto first_point = *temp_map->pointLayer.begin();
      double ref_lat = first_point.y();
      double ref_lon = first_point.x();

      // Create temporary projector to convert UTM offset to GPS
      lanelet::Origin ref_origin({ref_lat, ref_lon});
      lanelet::projection::UtmProjector temp_utm_projector(ref_origin);

      // Reverse projection to find GPS at UTM offset
      lanelet::BasicPoint3d utm_offset(utm_origin_x, utm_origin_y, 0);
      lanelet::GPSPoint origin_gps = temp_utm_projector.reverse(utm_offset);

      // Create actual projector with computed origin
      lanelet::Origin actual_origin({origin_gps.lat, origin_gps.lon});
      projector_ = std::make_unique<lanelet::projection::UtmProjector>(actual_origin);

      // Load map with properly positioned projector
      map_ = lanelet::load(osm_path, *projector_);
    }

    if (!map_ || map_->laneletLayer.empty()) {
      return false;
    }

    // Create traffic rules for vehicles
    traffic_rules_ =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    // Build routing graph
    routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);

    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::optional<lanelet::ConstLanelet> LaneletHandler::findNearestLanelet(const geometry_msgs::msg::Point & point) const
{
  if (!map_) {
    return std::nullopt;
  }

  lanelet::BasicPoint2d search_point(point.x, point.y);
  double min_dist = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> nearest;

  for (const auto & ll : map_->laneletLayer) {
    double dist = lanelet::geometry::distance2d(ll, search_point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = ll;
    }
  }

  return nearest;
}

std::optional<int64_t> LaneletHandler::findNearestLaneletId(const geometry_msgs::msg::Point & point) const
{
  auto nearest = findNearestLanelet(point);
  if (nearest.has_value()) {
    return nearest->id();
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> LaneletHandler::getLaneletById(int64_t id) const
{
  if (!map_) {
    return std::nullopt;
  }

  try {
    return map_->laneletLayer.get(id);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::vector<lanelet::ConstLanelet> LaneletHandler::getLaneletsInRadius(
  const geometry_msgs::msg::Point & center, double radius) const
{
  std::vector<lanelet::ConstLanelet> result;

  if (!map_) {
    return result;
  }

  lanelet::BasicPoint2d center_pt(center.x, center.y);

  for (const auto & ll : map_->laneletLayer) {
    double dist = lanelet::geometry::distance2d(ll, center_pt);
    if (dist <= radius) {
      result.push_back(ll);
    }
  }

  return result;
}

// Route Caching Implementation

bool LaneletHandler::setActiveRoute(int64_t from_id, int64_t to_id)
{
  if (!map_ || !routing_graph_) {
    return false;
  }

  auto from_ll = getLaneletById(from_id);
  auto to_ll = getLaneletById(to_id);

  if (!from_ll.has_value() || !to_ll.has_value()) {
    return false;
  }

  auto route = routing_graph_->getRoute(*from_ll, *to_ll, 0);
  if (!route) {
    return false;
  }

  auto shortest_path = route->shortestPath();

  std::lock_guard<std::mutex> lock(route_mutex_);
  active_route_.clear();
  for (const auto & ll : shortest_path) {
    active_route_.push_back(ll);
  }
  goal_lanelet_id_ = to_id;

  return true;
}

bool LaneletHandler::hasActiveRoute() const
{
  std::lock_guard<std::mutex> lock(route_mutex_);
  return !active_route_.empty();
}

void LaneletHandler::clearActiveRoute()
{
  std::lock_guard<std::mutex> lock(route_mutex_);
  active_route_.clear();
  goal_lanelet_id_ = -1;
}

int64_t LaneletHandler::getGoalLaneletId() const
{
  std::lock_guard<std::mutex> lock(route_mutex_);
  return goal_lanelet_id_;
}

lanelet_msgs::srv::GetRoute::Response LaneletHandler::getRouteFromPosition(
  const geometry_msgs::msg::Point & current_pos, double distance_m) const
{
  lanelet_msgs::srv::GetRoute::Response response;
  response.success = false;
  response.remaining_length_m = 0.0;

  if (!map_ || !routing_graph_) {
    response.error_message = "map_not_loaded";
    return response;
  }

  std::lock_guard<std::mutex> lock(route_mutex_);

  if (active_route_.empty()) {
    response.error_message = "no_active_route";
    return response;
  }

  // Find the nearest lanelet to current position on the route
  lanelet::BasicPoint2d search_point(current_pos.x, current_pos.y);
  double min_dist = std::numeric_limits<double>::max();
  size_t current_idx = 0;

  for (size_t i = 0; i < active_route_.size(); ++i) {
    double dist = lanelet::geometry::distance2d(active_route_[i], search_point);
    if (dist < min_dist) {
      min_dist = dist;
      current_idx = i;
    }
  }

  // If too far from the route, return error
  if (min_dist > 50.0) {  // 50m threshold
    response.error_message = "ego_not_on_route";
    return response;
  }

  // Calculate remaining distance to goal
  double remaining_dist = 0.0;
  for (size_t i = current_idx; i < active_route_.size(); ++i) {
    remaining_dist += lanelet::geometry::length2d(active_route_[i]);
  }
  response.remaining_length_m = remaining_dist;

  // Collect lanelets within the requested distance
  double accumulated_dist = 0.0;
  for (size_t i = current_idx; i < active_route_.size() && accumulated_dist < distance_m; ++i) {
    response.lanelets.push_back(toLaneletMsg(active_route_[i]));

    // Calculate transition type to next lanelet
    if (i + 1 < active_route_.size() && accumulated_dist < distance_m) {
      // Check if next lanelet is left, right, or successor
      auto left = routing_graph_->left(active_route_[i]);
      auto right = routing_graph_->right(active_route_[i]);

      if (left && left->id() == active_route_[i + 1].id()) {
        response.transitions.push_back(lanelet_msgs::srv::GetRoute::Response::TRANSITION_LEFT);
      } else if (right && right->id() == active_route_[i + 1].id()) {
        response.transitions.push_back(lanelet_msgs::srv::GetRoute::Response::TRANSITION_RIGHT);
      } else {
        response.transitions.push_back(lanelet_msgs::srv::GetRoute::Response::TRANSITION_SUCCESSOR);
      }
    }

    accumulated_dist += lanelet::geometry::length2d(active_route_[i]);
  }

  response.success = true;
  return response;
}

lanelet_msgs::srv::GetRoute::Response LaneletHandler::getRoute(int64_t from_id, int64_t to_id) const
{
  lanelet_msgs::srv::GetRoute::Response response;
  response.success = false;

  if (!map_ || !routing_graph_) {
    response.error_message = "Map not loaded";
    return response;
  }

  auto from_ll = getLaneletById(from_id);
  auto to_ll = getLaneletById(to_id);

  if (!from_ll.has_value() || !to_ll.has_value()) {
    response.error_message = "Invalid lanelet ID";
    return response;
  }

  auto route = routing_graph_->getRoute(*from_ll, *to_ll, 0);
  if (!route) {
    response.error_message = "No route found";
    return response;
  }

  auto shortest_path = route->shortestPath();
  for (const auto & ll : shortest_path) {
    response.lanelets.push_back(toLaneletMsg(ll));
  }

  response.success = true;
  return response;
}

lanelet_msgs::srv::GetCorridor::Response LaneletHandler::getCorridor(
  int64_t from_id, int64_t to_id, double max_length_m, double sample_spacing_m) const
{
  lanelet_msgs::srv::GetCorridor::Response response;
  response.success = false;

  auto route_response = getRoute(from_id, to_id);
  if (!route_response.success) {
    response.error_message = route_response.error_message;
    return response;
  }

  // Convert route lanelet IDs to lanelet objects
  std::vector<lanelet::ConstLanelet> route_lanelets;
  for (const auto & lanelet_msg : route_response.lanelets) {
    auto ll = getLaneletById(lanelet_msg.id);
    if (ll.has_value()) {
      route_lanelets.push_back(*ll);
    }
  }

  response.corridor = toCorridorMsg(route_lanelets, sample_spacing_m, max_length_m);
  response.success = true;
  return response;
}

lanelet_msgs::srv::GetLaneletsByRegElem::Response LaneletHandler::getLaneletsByRegElem(int64_t reg_elem_id) const
{
  lanelet_msgs::srv::GetLaneletsByRegElem::Response response;
  response.success = false;

  if (!map_) {
    response.error_message = "Map not loaded";
    return response;
  }

  try {
    auto reg_elem = map_->regulatoryElementLayer.get(reg_elem_id);

    for (const auto & ll : map_->laneletLayer) {
      for (const auto & re : ll.regulatoryElements()) {
        if (re->id() == reg_elem_id) {
          response.lanelets.push_back(toLaneletMsg(ll));
          break;
        }
      }
    }

    response.success = true;
  } catch (const std::exception & e) {
    response.error_message = std::string("Regulatory element not found: ") + e.what();
  }

  return response;
}

lanelet_msgs::msg::Lanelet LaneletHandler::toLaneletMsg(const lanelet::ConstLanelet & ll) const
{
  lanelet_msgs::msg::Lanelet msg;
  msg.id = ll.id();

  // Left boundary
  for (const auto & pt : ll.leftBound()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.left_boundary.push_back(p);
  }

  // Right boundary
  for (const auto & pt : ll.rightBound()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.right_boundary.push_back(p);
  }

  // Centerline
  for (const auto & pt : ll.centerline()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.centerline.push_back(p);
  }

  // Boundary attributes (type and color)
  msg.left_boundary_type = getBoundaryTypeForVisualization(ll.leftBound());
  msg.left_boundary_color = getBoundaryColor(ll.leftBound());
  msg.right_boundary_type = getBoundaryTypeForVisualization(ll.rightBound());
  msg.right_boundary_color = getBoundaryColor(ll.rightBound());

  populateLaneletSemantics(msg, ll);
  populateLaneletConnectivity(msg, ll);
  populateLaneletRegulatoryElements(msg, ll);

  return msg;
}

void LaneletHandler::populateLaneletSemantics(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const
{
  // Get lanelet type from attributes
  if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
    msg.lanelet_type = ll.attribute(lanelet::AttributeName::Subtype).value();
  }

  // Check if intersection
  msg.is_intersection =
    ll.hasAttribute("turn_direction") || ll.hasAttribute(lanelet::AttributeName::Subtype) &&
                                           ll.attribute(lanelet::AttributeName::Subtype).value() == "intersection";

  // Speed limit
  if (traffic_rules_) {
    auto speed = traffic_rules_->speedLimit(ll);
    msg.speed_limit_mps = speed.speedLimit.value();
  }
}

void LaneletHandler::populateLaneletConnectivity(
  lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const
{
  if (!routing_graph_) {
    return;
  }

  // Successor lanelets (what comes after)
  auto following = routing_graph_->following(ll);
  for (const auto & foll : following) {
    msg.successor_ids.push_back(foll.id());
  }

  // Left neighbor
  auto left = routing_graph_->left(ll);
  if (left) {
    msg.left_lane_id = left->id();
    msg.can_change_left = true;
  } else {
    msg.left_lane_id = -1;
    msg.can_change_left = false;
  }

  // Right neighbor
  auto right = routing_graph_->right(ll);
  if (right) {
    msg.right_lane_id = right->id();
    msg.can_change_right = true;
  } else {
    msg.right_lane_id = -1;
    msg.can_change_right = false;
  }
}

void LaneletHandler::populateLaneletRegulatoryElements(
  lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const
{
  for (const auto & reg_elem : ll.regulatoryElements()) {
    // Check for specific types
    if (!reg_elem->hasAttribute(lanelet::AttributeName::Subtype)) {
      continue;
    }
    std::string subtype = reg_elem->attribute(lanelet::AttributeName::Subtype).value();

    if (subtype == "traffic_light") {
      msg.has_traffic_light = true;

      // Extract traffic light positions from the regulatory element
      // Traffic lights have a "refers" role pointing to the light linestrings
      auto refers = reg_elem->getParameters<lanelet::ConstLineString3d>("refers");
      for (const auto & light_ls : refers) {
        lanelet_msgs::msg::TrafficLightInfo tl_info;
        tl_info.id = light_ls.id();
        tl_info.stop_line_id = -1;

        // Use centroid of the linestring as position
        if (!light_ls.empty()) {
          double x = 0, y = 0, z = 0;
          for (const auto & pt : light_ls) {
            x += pt.x();
            y += pt.y();
            z += pt.z();
          }
          tl_info.position.x = x / light_ls.size();
          tl_info.position.y = y / light_ls.size();
          tl_info.position.z = z / light_ls.size();
          msg.traffic_lights.push_back(tl_info);
        }
      }

      // Check for associated stop line (ref_line role)
      auto ref_lines = reg_elem->getParameters<lanelet::ConstLineString3d>("ref_line");
      for (const auto & stop_ls : ref_lines) {
        lanelet_msgs::msg::StopLineInfo sl_info;
        sl_info.id = stop_ls.id();
        sl_info.distance_along_lanelet_m = 0.0;  // TODO(WATonomous): calculate actual distance

        for (const auto & pt : stop_ls) {
          geometry_msgs::msg::Point p;
          p.x = pt.x();
          p.y = pt.y();
          p.z = pt.z();
          sl_info.points.push_back(p);
        }

        if (!sl_info.points.empty()) {
          msg.stop_lines.push_back(sl_info);
          msg.has_stop_line = true;
        }
      }
    }

    if (subtype == "stop_sign" || subtype == "right_of_way") {
      // Extract stop lines from stop signs and right-of-way elements
      auto ref_lines = reg_elem->getParameters<lanelet::ConstLineString3d>("ref_line");
      for (const auto & stop_ls : ref_lines) {
        lanelet_msgs::msg::StopLineInfo sl_info;
        sl_info.id = stop_ls.id();
        sl_info.distance_along_lanelet_m = 0.0;

        for (const auto & pt : stop_ls) {
          geometry_msgs::msg::Point p;
          p.x = pt.x();
          p.y = pt.y();
          p.z = pt.z();
          sl_info.points.push_back(p);
        }

        if (!sl_info.points.empty()) {
          msg.stop_lines.push_back(sl_info);
          msg.has_stop_line = true;
        }
      }
    }

    if (subtype == "yield") {
      msg.has_yield_sign = true;
      msg.must_yield = true;
    }
  }
}

double LaneletHandler::getSpeedLimit(const lanelet::ConstLanelet & ll) const
{
  if (traffic_rules_) {
    auto speed = traffic_rules_->speedLimit(ll);
    return speed.speedLimit.value();
  }
  return 0.0;
}

uint8_t LaneletHandler::getBoundaryType(const lanelet::ConstLineString3d & boundary) const
{
  // Check boundary attributes for type
  if (boundary.hasAttribute("type")) {
    std::string type = boundary.attribute("type").value();
    if (type == "road_border" || type == "curbstone" || type == "guard_rail") {
      return lanelet_msgs::msg::CorridorLane::BOUNDARY_ROAD_EDGE;
    }
    if (type == "solid" || type == "solid_solid") {
      return lanelet_msgs::msg::CorridorLane::BOUNDARY_SOLID;
    }
    if (type == "dashed" || type == "dashed_dashed") {
      return lanelet_msgs::msg::CorridorLane::BOUNDARY_DASHED;
    }
  }
  // Default to solid if unknown
  return lanelet_msgs::msg::CorridorLane::BOUNDARY_SOLID;
}

uint8_t LaneletHandler::getBoundaryTypeForVisualization(const lanelet::ConstLineString3d & boundary) const
{
  // Check both "subtype" (OSM standard) and "type" attributes
  std::string subtype;
  if (boundary.hasAttribute("subtype")) {
    subtype = boundary.attribute("subtype").value();
  } else if (boundary.hasAttribute("type")) {
    subtype = boundary.attribute("type").value();
  }

  if (subtype == "solid_solid") {
    return lanelet_msgs::msg::Lanelet::BOUNDARY_SOLID_SOLID;
  }
  if (subtype == "solid") {
    return lanelet_msgs::msg::Lanelet::BOUNDARY_SOLID;
  }
  if (subtype == "dashed" || subtype == "dashed_dashed") {
    return lanelet_msgs::msg::Lanelet::BOUNDARY_DASHED;
  }
  if (subtype == "road_border" || subtype == "curbstone" || subtype == "guard_rail") {
    return lanelet_msgs::msg::Lanelet::BOUNDARY_ROAD_EDGE;
  }
  // Default to solid
  return lanelet_msgs::msg::Lanelet::BOUNDARY_SOLID;
}

uint8_t LaneletHandler::getBoundaryColor(const lanelet::ConstLineString3d & boundary) const
{
  if (boundary.hasAttribute("color")) {
    std::string color = boundary.attribute("color").value();
    if (color == "yellow") {
      return lanelet_msgs::msg::Lanelet::COLOR_YELLOW;
    }
  }
  return lanelet_msgs::msg::Lanelet::COLOR_WHITE;
}

lanelet_msgs::msg::CorridorLane LaneletHandler::buildCorridorLane(
  const std::vector<lanelet::ConstLanelet> & lanelets, double sample_spacing_m, double max_length_m) const
{
  lanelet_msgs::msg::CorridorLane lane;

  if (lanelets.empty()) {
    return lane;
  }

  double total_arc_length = 0.0;
  geometry_msgs::msg::Point prev_point;
  bool has_prev = false;

  for (const auto & ll : lanelets) {
    if (total_arc_length >= max_length_m) {
      break;
    }

    auto centerline = ll.centerline();
    auto left_bound = ll.leftBound();
    auto right_bound = ll.rightBound();

    if (centerline.empty()) {
      continue;
    }

    double speed_limit = getSpeedLimit(ll);
    uint8_t left_type = getBoundaryType(left_bound);
    uint8_t right_type = getBoundaryType(right_bound);

    // Sample along centerline
    double accumulated_dist = 0.0;

    for (size_t i = 0; i < centerline.size() && total_arc_length < max_length_m; ++i) {
      // Check if we should emit a sample
      bool emit_sample = false;
      if (i == 0 && !has_prev) {
        emit_sample = true;  // First point ever
      } else if (has_prev) {
        double dx = centerline[i].x() - prev_point.x;
        double dy = centerline[i].y() - prev_point.y;
        accumulated_dist += std::sqrt(dx * dx + dy * dy);
        if (accumulated_dist >= sample_spacing_m) {
          emit_sample = true;
          accumulated_dist = 0.0;
        }
      }

      if (emit_sample) {
        // Centerline point
        geometry_msgs::msg::Point pt;
        pt.x = centerline[i].x();
        pt.y = centerline[i].y();
        pt.z = centerline[i].z();
        lane.centerline.push_back(pt);

        // Arc length
        if (has_prev) {
          double dx = pt.x - prev_point.x;
          double dy = pt.y - prev_point.y;
          total_arc_length += std::sqrt(dx * dx + dy * dy);
        }
        lane.arc_length.push_back(total_arc_length);

        // Heading (tangent direction)
        double heading = 0.0;
        if (i + 1 < centerline.size()) {
          double dx = centerline[i + 1].x() - centerline[i].x();
          double dy = centerline[i + 1].y() - centerline[i].y();
          heading = std::atan2(dy, dx);
        } else if (i > 0) {
          double dx = centerline[i].x() - centerline[i - 1].x();
          double dy = centerline[i].y() - centerline[i - 1].y();
          heading = std::atan2(dy, dx);
        }
        lane.heading.push_back(heading);

        // Curvature (simplified - would need more points for accurate calculation)
        lane.curvature.push_back(0.0);

        // Speed limit
        lane.speed_limit.push_back(speed_limit);

        // Boundary offsets - find closest point on each boundary
        // Simplified: use same index ratio
        double t = static_cast<double>(i) / std::max(1.0, static_cast<double>(centerline.size() - 1));
        size_t left_idx = std::min(static_cast<size_t>(t * (left_bound.size() - 1)), left_bound.size() - 1);
        size_t right_idx = std::min(static_cast<size_t>(t * (right_bound.size() - 1)), right_bound.size() - 1);

        // Left offset (positive = left of centerline)
        double left_dx = left_bound[left_idx].x() - centerline[i].x();
        double left_dy = left_bound[left_idx].y() - centerline[i].y();
        double left_offset = std::sqrt(left_dx * left_dx + left_dy * left_dy);
        lane.left_offset.push_back(left_offset);
        lane.left_boundary_type.push_back(left_type);

        // Right offset (positive value, but represents right side)
        double right_dx = right_bound[right_idx].x() - centerline[i].x();
        double right_dy = right_bound[right_idx].y() - centerline[i].y();
        double right_offset = std::sqrt(right_dx * right_dx + right_dy * right_dy);
        lane.right_offset.push_back(right_offset);
        lane.right_boundary_type.push_back(right_type);

        prev_point = pt;
        has_prev = true;
      }
    }
  }

  return lane;
}

lanelet_msgs::msg::Corridor LaneletHandler::toCorridorMsg(
  const std::vector<lanelet::ConstLanelet> & route_lanelets, double sample_spacing_m, double max_length_m) const
{
  lanelet_msgs::msg::Corridor corridor;

  if (route_lanelets.empty() || !routing_graph_) {
    return corridor;
  }

  // Build reference lane from route
  corridor.reference = buildCorridorLane(route_lanelets, sample_spacing_m, max_length_m);

  // Build left adjacent lane if crossable
  std::vector<lanelet::ConstLanelet> left_lanelets;
  for (const auto & ll : route_lanelets) {
    auto left = routing_graph_->left(ll);
    if (left) {
      left_lanelets.push_back(*left);
    }
  }
  if (!left_lanelets.empty()) {
    corridor.left_lane = buildCorridorLane(left_lanelets, sample_spacing_m, max_length_m);
  }

  // Build right adjacent lane if crossable
  std::vector<lanelet::ConstLanelet> right_lanelets;
  for (const auto & ll : route_lanelets) {
    auto right = routing_graph_->right(ll);
    if (right) {
      right_lanelets.push_back(*right);
    }
  }
  if (!right_lanelets.empty()) {
    corridor.right_lane = buildCorridorLane(right_lanelets, sample_spacing_m, max_length_m);
  }

  return corridor;
}

}  // namespace world_model
