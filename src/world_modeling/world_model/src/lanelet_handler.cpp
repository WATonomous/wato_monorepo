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
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_routing/Route.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace world_model
{

LaneletHandler::LaneletHandler()
: map_(nullptr), routing_graph_(nullptr), traffic_rules_(nullptr), projector_(nullptr)
{
}

bool LaneletHandler::loadMap(const std::string & osm_path, double lat_origin, double lon_origin)
{
  try {
    // Create UTM projector with origin
    lanelet::Origin origin({lat_origin, lon_origin});
    projector_ = std::make_unique<lanelet::projection::UtmProjector>(origin);

    // Load the map
    map_ = lanelet::load(osm_path, *projector_);
    if (!map_) {
      return false;
    }

    // Create traffic rules for vehicles
    traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    // Build routing graph
    routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);

    return true;
  } catch (const std::exception & e) {
    map_ = nullptr;
    routing_graph_ = nullptr;
    return false;
  }
}

bool LaneletHandler::isMapLoaded() const
{
  return map_ != nullptr && routing_graph_ != nullptr;
}

lanelet_msgs::srv::GetRoute::Response LaneletHandler::getRoute(int64_t from_id, int64_t to_id)
{
  lanelet_msgs::srv::GetRoute::Response response;

  if (!isMapLoaded()) {
    response.success = false;
    response.error_message = "map_not_loaded";
    return response;
  }

  // Get lanelets by ID
  auto from_ll_opt = getLaneletById(from_id);
  auto to_ll_opt = getLaneletById(to_id);

  if (!from_ll_opt.has_value() || !to_ll_opt.has_value()) {
    response.success = false;
    response.error_message = "lanelet_not_found";
    return response;
  }

  // Compute route
  auto route = routing_graph_->getRoute(from_ll_opt.value(), to_ll_opt.value());
  if (!route) {
    response.success = false;
    response.error_message = "no_route_exists";
    return response;
  }

  // Get shortest path
  auto shortest_path = route->shortestPath();

  // Build response
  response.success = true;
  response.total_length_m = 0.0;

  for (size_t i = 0; i < shortest_path.size(); ++i) {
    const auto & ll = shortest_path[i];
    response.lanelets.push_back(toLaneletMsg(ll));
    response.total_length_m += lanelet::geometry::length2d(ll);

    // Add transition type
    if (i < shortest_path.size() - 1) {
      response.transitions.push_back(getTransitionType(ll, shortest_path[i + 1]));
    }
  }

  return response;
}

lanelet_msgs::srv::GetCorridor::Response LaneletHandler::getCorridor(
  int64_t from_id, int64_t to_id, double max_length_m,
  double sample_spacing_m, int32_t num_lanes_each_side)
{
  lanelet_msgs::srv::GetCorridor::Response response;

  if (!isMapLoaded()) {
    response.success = false;
    response.error_message = "map_not_loaded";
    return response;
  }

  auto from_ll_opt = getLaneletById(from_id);
  auto to_ll_opt = getLaneletById(to_id);

  if (!from_ll_opt.has_value() || !to_ll_opt.has_value()) {
    response.success = false;
    response.error_message = "lanelet_not_found";
    return response;
  }

  auto route = routing_graph_->getRoute(from_ll_opt.value(), to_ll_opt.value());
  if (!route) {
    response.success = false;
    response.error_message = "no_route_exists";
    return response;
  }

  auto shortest_path = route->shortestPath();
  std::vector<lanelet::ConstLanelet> route_lanelets(shortest_path.begin(), shortest_path.end());

  response.success = true;
  response.corridor = toCorridorMsg(route_lanelets, num_lanes_each_side, sample_spacing_m, max_length_m);

  return response;
}

lanelet_msgs::srv::GetLaneletsByRegElem::Response LaneletHandler::getLaneletsByRegElem(
  int64_t reg_elem_id)
{
  lanelet_msgs::srv::GetLaneletsByRegElem::Response response;

  if (!isMapLoaded()) {
    response.success = false;
    response.error_message = "map_not_loaded";
    return response;
  }

  // Find regulatory element
  auto reg_elem = map_->regulatoryElementLayer.find(reg_elem_id);
  if (reg_elem == map_->regulatoryElementLayer.end()) {
    response.success = false;
    response.error_message = "reg_elem_not_found";
    return response;
  }

  response.success = true;

  // Find all lanelets that reference this regulatory element
  for (const auto & ll : map_->laneletLayer) {
    for (const auto & re : ll.regulatoryElements()) {
      if (re->id() == reg_elem_id) {
        response.lanelets.push_back(toLaneletMsg(ll));
        break;
      }
    }
  }

  return response;
}

std::optional<lanelet::ConstLanelet> LaneletHandler::findNearestLanelet(
  const geometry_msgs::msg::Point & point)
{
  if (!isMapLoaded()) {
    return std::nullopt;
  }

  lanelet::BasicPoint2d search_point(point.x, point.y);
  double min_dist = std::numeric_limits<double>::max();
  lanelet::ConstLanelet nearest;
  bool found = false;

  for (const auto & ll : map_->laneletLayer) {
    double dist = lanelet::geometry::distance2d(ll, search_point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = ll;
      found = true;
    }
  }

  if (found) {
    return nearest;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> LaneletHandler::getLaneletById(int64_t id)
{
  if (!isMapLoaded()) {
    return std::nullopt;
  }

  auto it = map_->laneletLayer.find(id);
  if (it != map_->laneletLayer.end()) {
    return *it;
  }
  return std::nullopt;
}

std::vector<lanelet::ConstLanelet> LaneletHandler::getLaneletsInRadius(
  const geometry_msgs::msg::Point & center, double radius)
{
  std::vector<lanelet::ConstLanelet> result;

  if (!isMapLoaded()) {
    return result;
  }

  lanelet::BasicPoint2d center_pt(center.x, center.y);

  for (const auto & ll : map_->laneletLayer) {
    if (lanelet::geometry::distance2d(ll, center_pt) <= radius) {
      result.push_back(ll);
    }
  }

  return result;
}

lanelet_msgs::msg::Lanelet LaneletHandler::toLaneletMsg(const lanelet::ConstLanelet & ll)
{
  lanelet_msgs::msg::Lanelet msg;
  msg.id = ll.id();

  // Geometry - left boundary
  for (const auto & pt : ll.leftBound()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.left_boundary.push_back(p);
  }

  // Geometry - right boundary
  for (const auto & pt : ll.rightBound()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.right_boundary.push_back(p);
  }

  // Geometry - centerline
  for (const auto & pt : ll.centerline()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    msg.centerline.push_back(p);
  }

  // Populate other fields
  populateLaneletSemantics(msg, ll);
  populateLaneletConnectivity(msg, ll);
  populateLaneletRegulatoryElements(msg, ll);

  return msg;
}

lanelet_msgs::msg::Corridor LaneletHandler::toCorridorMsg(
  const std::vector<lanelet::ConstLanelet> & route_lanelets,
  int32_t num_lanes_each_side, double sample_spacing_m, double max_length_m)
{
  lanelet_msgs::msg::Corridor corridor;

  if (route_lanelets.empty()) {
    return corridor;
  }

  // Collect centerline points along route with sampling
  std::vector<geometry_msgs::msg::Point> sampled_centerline;
  double accumulated_length = 0.0;
  double last_sample_length = -sample_spacing_m;  // Ensure first point is sampled

  for (const auto & ll : route_lanelets) {
    if (max_length_m > 0 && accumulated_length >= max_length_m) {
      break;
    }

    for (const auto & pt : ll.centerline()) {
      if (!sampled_centerline.empty()) {
        const auto & last = sampled_centerline.back();
        double dx = pt.x() - last.x;
        double dy = pt.y() - last.y;
        double dz = pt.z() - last.z;
        accumulated_length += std::sqrt(dx * dx + dy * dy + dz * dz);
      }

      if (accumulated_length - last_sample_length >= sample_spacing_m) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        sampled_centerline.push_back(p);
        last_sample_length = accumulated_length;
      }

      if (max_length_m > 0 && accumulated_length >= max_length_m) {
        break;
      }
    }
  }

  // Build main lane path
  lanelet_msgs::msg::LanePath main_lane;
  for (const auto & ll : route_lanelets) {
    main_lane.lanelet_ids.push_back(ll.id());
  }
  main_lane.centerline = sampled_centerline;

  // Estimate widths (simplified - uses first lanelet width)
  double avg_width = 3.5;  // Default
  if (!route_lanelets.empty()) {
    const auto & first_ll = route_lanelets[0];
    if (!first_ll.leftBound().empty() && !first_ll.rightBound().empty()) {
      auto left_pt = first_ll.leftBound().front();
      auto right_pt = first_ll.rightBound().front();
      avg_width = std::sqrt(
        std::pow(left_pt.x() - right_pt.x(), 2) +
        std::pow(left_pt.y() - right_pt.y(), 2));
    }
  }
  main_lane.widths.resize(sampled_centerline.size(), avg_width);

  // Set corridor fields
  corridor.num_lanes = 1;  // Start with reference lane only
  corridor.reference_lane_index = 0;
  corridor.lanes.push_back(main_lane);

  // Add adjacent lanes if requested
  // Note: Full implementation would traverse left/right in routing graph
  // For now, we just set up the structure

  // Build arc length, heading, curvature arrays
  corridor.arc_length.push_back(0.0);
  for (size_t i = 1; i < sampled_centerline.size(); ++i) {
    double dx = sampled_centerline[i].x - sampled_centerline[i - 1].x;
    double dy = sampled_centerline[i].y - sampled_centerline[i - 1].y;
    corridor.arc_length.push_back(corridor.arc_length.back() + std::sqrt(dx * dx + dy * dy));
  }

  // Compute headings
  for (size_t i = 0; i < sampled_centerline.size(); ++i) {
    double heading = 0.0;
    if (i < sampled_centerline.size() - 1) {
      double dx = sampled_centerline[i + 1].x - sampled_centerline[i].x;
      double dy = sampled_centerline[i + 1].y - sampled_centerline[i].y;
      heading = std::atan2(dy, dx);
    } else if (i > 0) {
      heading = corridor.heading.back();
    }
    corridor.heading.push_back(heading);
  }

  // Compute curvatures (simplified finite difference)
  for (size_t i = 0; i < corridor.heading.size(); ++i) {
    double curvature = 0.0;
    if (i > 0 && i < corridor.heading.size() - 1) {
      double dtheta = corridor.heading[i + 1] - corridor.heading[i - 1];
      double ds = corridor.arc_length[i + 1] - corridor.arc_length[i - 1];
      if (ds > 0.001) {
        curvature = dtheta / ds;
      }
    }
    corridor.curvature.push_back(curvature);
  }

  // Set outer boundaries (simplified - just use first and last lanelet bounds)
  if (!route_lanelets.empty()) {
    for (const auto & ll : route_lanelets) {
      for (const auto & pt : ll.leftBound()) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        corridor.leftmost_boundary.push_back(p);
      }
      for (const auto & pt : ll.rightBound()) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        corridor.rightmost_boundary.push_back(p);
      }
    }
  }

  return corridor;
}

lanelet::routing::RoutingGraphConstPtr LaneletHandler::getRoutingGraph() const
{
  return routing_graph_;
}

lanelet::traffic_rules::TrafficRulesPtr LaneletHandler::getTrafficRules() const
{
  return traffic_rules_;
}

void LaneletHandler::populateLaneletSemantics(
  lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll)
{
  // Lanelet type
  if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
    msg.lanelet_type = ll.attribute(lanelet::AttributeName::Subtype).value();
  } else {
    msg.lanelet_type = "road";
  }

  // Speed limit
  if (traffic_rules_) {
    auto speed_limit = traffic_rules_->speedLimit(ll);
    msg.speed_limit_mps = speed_limit.speedLimit.value();
    msg.speed_limit_is_mandatory = speed_limit.isMandatory;
  } else {
    msg.speed_limit_mps = 0.0;
    msg.speed_limit_is_mandatory = false;
  }

  // Intersection flags
  msg.is_intersection = ll.hasAttribute("turn_direction") ||
    msg.lanelet_type == "intersection";
  msg.has_traffic_light = false;
  msg.has_stop_line = false;
  msg.has_yield_sign = false;
  msg.must_yield = false;

  // Check regulatory elements for flags
  for (const auto & re : ll.regulatoryElements()) {
    auto type = re->attribute(lanelet::AttributeName::Subtype);
    if (type.value() == "traffic_light") {
      msg.has_traffic_light = true;
    } else if (type.value() == "stop_line" || type.value() == "stop_sign") {
      msg.has_stop_line = true;
    } else if (type.value() == "yield") {
      msg.has_yield_sign = true;
      msg.must_yield = true;
    }
  }
}

void LaneletHandler::populateLaneletConnectivity(
  lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll)
{
  msg.left_lane_id = -1;
  msg.right_lane_id = -1;
  msg.can_change_left = false;
  msg.can_change_right = false;

  if (!routing_graph_) {
    return;
  }

  // Get adjacent lanelets
  auto left = routing_graph_->left(ll);
  auto right = routing_graph_->right(ll);

  if (left) {
    msg.left_lane_id = left->id();
    msg.can_change_left = true;
  }
  if (right) {
    msg.right_lane_id = right->id();
    msg.can_change_right = true;
  }

  // Get successors
  auto following = routing_graph_->following(ll);
  for (const auto & succ : following) {
    msg.successor_ids.push_back(succ.id());
  }
}

void LaneletHandler::populateLaneletRegulatoryElements(
  lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll)
{
  for (const auto & re : ll.regulatoryElements()) {
    auto type = re->attribute(lanelet::AttributeName::Subtype);

    if (type.value() == "traffic_light") {
      lanelet_msgs::msg::TrafficLightInfo tl_info;
      tl_info.id = re->id();

      // Get position from ref line if available
      auto ref_lines = re->getParameters<lanelet::ConstLineString3d>("ref_line");
      if (!ref_lines.empty() && !ref_lines[0].empty()) {
        auto pt = ref_lines[0].front();
        tl_info.position.x = pt.x();
        tl_info.position.y = pt.y();
        tl_info.position.z = pt.z();
      }

      // Get associated stop line
      auto stop_lines = re->getParameters<lanelet::ConstLineString3d>("stop_line");
      if (!stop_lines.empty()) {
        tl_info.stop_line_id = stop_lines[0].id();
      } else {
        tl_info.stop_line_id = -1;
      }

      msg.traffic_lights.push_back(tl_info);
    }

    // Handle stop lines
    auto stop_lines = re->getParameters<lanelet::ConstLineString3d>("stop_line");
    for (const auto & sl : stop_lines) {
      lanelet_msgs::msg::StopLineInfo sl_info;
      sl_info.id = sl.id();

      for (const auto & pt : sl) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        sl_info.points.push_back(p);
      }

      // Calculate distance along lanelet (simplified)
      sl_info.distance_along_lanelet_m = 0.0;
      if (!ll.centerline().empty() && !sl.empty()) {
        auto sl_center = sl.front();
        for (size_t i = 0; i < ll.centerline().size() - 1; ++i) {
          auto pt = ll.centerline()[i];
          auto next_pt = ll.centerline()[i + 1];
          double dx = next_pt.x() - pt.x();
          double dy = next_pt.y() - pt.y();
          sl_info.distance_along_lanelet_m += std::sqrt(dx * dx + dy * dy);

          // Check if stop line is near this segment
          double dist_to_sl = std::sqrt(
            std::pow(pt.x() - sl_center.x(), 2) +
            std::pow(pt.y() - sl_center.y(), 2));
          if (dist_to_sl < 5.0) {
            break;
          }
        }
      }

      msg.stop_lines.push_back(sl_info);
    }
  }
}

std::vector<geometry_msgs::msg::Point> LaneletHandler::sampleCenterline(
  const lanelet::ConstLanelet & ll, double spacing_m)
{
  std::vector<geometry_msgs::msg::Point> result;

  double accumulated = 0.0;
  double last_sample = -spacing_m;

  for (size_t i = 0; i < ll.centerline().size(); ++i) {
    const auto & pt = ll.centerline()[i];

    if (i > 0) {
      const auto & prev = ll.centerline()[i - 1];
      double dx = pt.x() - prev.x();
      double dy = pt.y() - prev.y();
      accumulated += std::sqrt(dx * dx + dy * dy);
    }

    if (accumulated - last_sample >= spacing_m || i == 0) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      result.push_back(p);
      last_sample = accumulated;
    }
  }

  return result;
}

uint8_t LaneletHandler::getTransitionType(
  const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to)
{
  if (!routing_graph_) {
    return lanelet_msgs::srv::GetRoute::Response::TRANSITION_SUCCESSOR;
  }

  // Check if it's a lane change
  auto left = routing_graph_->left(from);
  if (left && left->id() == to.id()) {
    return lanelet_msgs::srv::GetRoute::Response::TRANSITION_LEFT;
  }

  auto right = routing_graph_->right(from);
  if (right && right->id() == to.id()) {
    return lanelet_msgs::srv::GetRoute::Response::TRANSITION_RIGHT;
  }

  // Otherwise it's a successor (following)
  return lanelet_msgs::srv::GetRoute::Response::TRANSITION_SUCCESSOR;
}

}  // namespace world_model
