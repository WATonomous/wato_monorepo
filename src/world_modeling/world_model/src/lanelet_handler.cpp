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
#include <set>
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

std::optional<int64_t> LaneletHandler::findCurrentLaneletId(
  const geometry_msgs::msg::Point & point,
  double heading_rad,
  double route_priority_threshold_m,
  double heading_search_radius_m) const
{
  if (!map_) {
    return std::nullopt;
  }

  lanelet::BasicPoint2d search_point(point.x, point.y);

  // 1. If route exists, check if ego is on a route lanelet
  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    if (!active_route_.empty()) {
      for (const auto & ll : active_route_) {
        double dist = lanelet::geometry::distance2d(ll, search_point);
        if (dist <= route_priority_threshold_m) {
          return ll.id();  // On route - use this lanelet
        }
      }
    }
  }

  // 2. Fallback: find lanelet with best heading alignment
  auto nearby = getLaneletsInRadius(point, heading_search_radius_m);
  if (nearby.empty()) {
    return findNearestLaneletId(point);  // Ultimate fallback
  }

  // Compute ego heading vector
  double ego_dx = std::cos(heading_rad);
  double ego_dy = std::sin(heading_rad);

  int64_t best_id = -1;
  double best_score = -1.0;

  for (const auto & ll : nearby) {
    // Get centerline tangent at closest point to ego
    auto centerline = ll.centerline2d();
    if (centerline.size() < 2) {
      continue;
    }

    // Find closest segment
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < centerline.size(); ++i) {
      double dx = centerline[i].x() - point.x;
      double dy = centerline[i].y() - point.y;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    // Compute tangent at closest point
    size_t next_idx = std::min(closest_idx + 1, centerline.size() - 1);
    size_t prev_idx = closest_idx > 0 ? closest_idx - 1 : 0;
    double tangent_dx = centerline[next_idx].x() - centerline[prev_idx].x();
    double tangent_dy = centerline[next_idx].y() - centerline[prev_idx].y();
    double tangent_len = std::sqrt(tangent_dx * tangent_dx + tangent_dy * tangent_dy);
    if (tangent_len < 1e-6) {
      continue;
    }
    tangent_dx /= tangent_len;
    tangent_dy /= tangent_len;

    // Dot product = cos(angle) - higher means better alignment
    double alignment = std::abs(ego_dx * tangent_dx + ego_dy * tangent_dy);

    // Weight by distance (prefer closer lanelets when alignment is similar)
    double dist_weight = 1.0 / (1.0 + std::sqrt(min_dist));
    double score = alignment * dist_weight;

    if (score > best_score) {
      best_score = score;
      best_id = ll.id();
    }
  }

  if (best_id >= 0) {
    return best_id;
  }
  return findNearestLaneletId(point);  // Ultimate fallback
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

lanelet_msgs::srv::GetShortestRoute::Response LaneletHandler::getShortestRoute(
  const geometry_msgs::msg::Point & current_pos) const
{
  lanelet_msgs::srv::GetShortestRoute::Response response;
  response.success = false;
  response.total_length_m = 0.0;

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

  // Calculate total distance to goal and collect ALL lanelets from current position
  double total_dist = 0.0;
  for (size_t i = current_idx; i < active_route_.size(); ++i) {
    response.lanelets.push_back(toLaneletMsg(active_route_[i]));
    total_dist += lanelet::geometry::length2d(active_route_[i]);

    // Calculate transition type to next lanelet
    if (i + 1 < active_route_.size()) {
      // Check if next lanelet is left, right, or successor
      auto left = routing_graph_->left(active_route_[i]);
      auto right = routing_graph_->right(active_route_[i]);

      if (left && left->id() == active_route_[i + 1].id()) {
        response.transitions.push_back(lanelet_msgs::srv::GetShortestRoute::Response::TRANSITION_LEFT);
      } else if (right && right->id() == active_route_[i + 1].id()) {
        response.transitions.push_back(lanelet_msgs::srv::GetShortestRoute::Response::TRANSITION_RIGHT);
      } else {
        response.transitions.push_back(lanelet_msgs::srv::GetShortestRoute::Response::TRANSITION_SUCCESSOR);
      }
    }
  }

  response.total_length_m = total_dist;
  response.success = true;
  return response;
}

lanelet_msgs::msg::RouteAhead LaneletHandler::getRouteAhead(
  const geometry_msgs::msg::Point & current_pos, double lookahead_distance_m) const
{
  lanelet_msgs::msg::RouteAhead msg;
  msg.has_active_route = false;
  msg.distance_to_first_m = 0.0;
  msg.total_distance_m = 0.0;

  if (!map_ || !routing_graph_) {
    return msg;
  }

  std::lock_guard<std::mutex> lock(route_mutex_);

  if (active_route_.empty()) {
    return msg;
  }

  msg.has_active_route = true;

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

  // If too far from the route, return empty result
  if (min_dist > 50.0) {  // 50m threshold
    msg.has_active_route = false;
    return msg;
  }

  // Set distance to first lanelet start (approximate)
  msg.distance_to_first_m = min_dist;

  // Collect lanelets within the lookahead distance
  double accumulated_dist = 0.0;
  for (size_t i = current_idx; i < active_route_.size() && accumulated_dist < lookahead_distance_m; ++i) {
    msg.ids.push_back(active_route_[i].id());
    msg.lanelets.push_back(toLaneletMsg(active_route_[i]));
    accumulated_dist += lanelet::geometry::length2d(active_route_[i]);
  }

  msg.total_distance_m = accumulated_dist;
  return msg;
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
  // Track processed regulatory element IDs to avoid duplicates
  std::set<int64_t> processed_ids;

  for (const auto & reg_elem : ll.regulatoryElements()) {
    // Skip if already processed
    if (processed_ids.count(reg_elem->id()) > 0) {
      continue;
    }
    processed_ids.insert(reg_elem->id());

    lanelet_msgs::msg::RegulatoryElement re_msg;
    re_msg.id = reg_elem->id();

    // Get subtype
    if (reg_elem->hasAttribute(lanelet::AttributeName::Subtype)) {
      re_msg.subtype = reg_elem->attribute(lanelet::AttributeName::Subtype).value();
    } else {
      re_msg.subtype = "unknown";
    }

    // Extract referred geometry positions (traffic lights, signs)
    auto refers = reg_elem->getParameters<lanelet::ConstLineString3d>("refers");
    for (const auto & refer_ls : refers) {
      // Use centroid of the linestring as position
      if (!refer_ls.empty()) {
        double x = 0, y = 0, z = 0;
        for (const auto & pt : refer_ls) {
          x += pt.x();
          y += pt.y();
          z += pt.z();
        }
        geometry_msgs::msg::Point p;
        p.x = x / refer_ls.size();
        p.y = y / refer_ls.size();
        p.z = z / refer_ls.size();
        re_msg.refers_positions.push_back(p);
      }
    }

    // Also check for point-based refers
    auto refers_points = reg_elem->getParameters<lanelet::ConstPoint3d>("refers");
    for (const auto & pt : refers_points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      re_msg.refers_positions.push_back(p);
    }

    // Extract reference lines (stop lines)
    auto ref_lines = reg_elem->getParameters<lanelet::ConstLineString3d>("ref_line");
    for (const auto & ref_ls : ref_lines) {
      lanelet_msgs::msg::RefLine ref_line_msg;

      // Calculate distance along lanelet centerline to the ref_line (stop line)
      auto centerline = ll.centerline2d();
      if (centerline.size() >= 2 && !ref_ls.empty()) {
        // Compute centroid of the ref_line
        double ref_cx = 0.0, ref_cy = 0.0;
        for (const auto & pt : ref_ls) {
          ref_cx += pt.x();
          ref_cy += pt.y();
        }
        ref_cx /= static_cast<double>(ref_ls.size());
        ref_cy /= static_cast<double>(ref_ls.size());

        // Find the closest point on the centerline and compute arc length to it
        double min_dist_sq = std::numeric_limits<double>::max();
        double best_arc_length = 0.0;
        double accumulated_length = 0.0;

        for (size_t i = 0; i + 1 < centerline.size(); ++i) {
          double ax = centerline[i].x(), ay = centerline[i].y();
          double bx = centerline[i + 1].x(), by = centerline[i + 1].y();
          double dx = bx - ax, dy = by - ay;
          double seg_len_sq = dx * dx + dy * dy;

          // Project ref_line centroid onto this centerline segment
          double t = 0.0;
          if (seg_len_sq > 1e-12) {
            t = ((ref_cx - ax) * dx + (ref_cy - ay) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
          }

          double proj_x = ax + t * dx;
          double proj_y = ay + t * dy;
          double dist_sq =
            (ref_cx - proj_x) * (ref_cx - proj_x) + (ref_cy - proj_y) * (ref_cy - proj_y);

          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_arc_length = accumulated_length + t * std::sqrt(seg_len_sq);
          }

          accumulated_length += std::sqrt(seg_len_sq);
        }

        ref_line_msg.distance_along_lanelet_m = best_arc_length;
      } else {
        ref_line_msg.distance_along_lanelet_m = 0.0;
      }

      for (const auto & pt : ref_ls) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        ref_line_msg.points.push_back(p);
      }

      if (!ref_line_msg.points.empty()) {
        re_msg.ref_lines.push_back(ref_line_msg);
      }
    }

    // Extract yield relationships for right_of_way elements
    if (re_msg.subtype == "right_of_way") {
      // Get yield lanelets
      auto yield_lanelets = reg_elem->getParameters<lanelet::ConstLanelet>("yield");
      for (const auto & yield_ll : yield_lanelets) {
        re_msg.yield_lanelet_ids.push_back(yield_ll.id());
      }

      // Get right-of-way lanelets
      auto row_lanelets = reg_elem->getParameters<lanelet::ConstLanelet>("right_of_way");
      for (const auto & row_ll : row_lanelets) {
        re_msg.right_of_way_lanelet_ids.push_back(row_ll.id());
      }
    }

    // Extract generic attributes
    for (const auto & attr : reg_elem->attributes()) {
      re_msg.attribute_keys.push_back(attr.first);
      re_msg.attribute_values.push_back(attr.second.value());
    }

    msg.regulatory_elements.push_back(re_msg);
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

}  // namespace world_model
