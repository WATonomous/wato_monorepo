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

// Handling (creating, updating, deleting) Reg elements

void HDMapRouter::process_traffic_light_msg(
  const vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg_ptr)
{
  std::map<uint64_t, bool> found_id;
  for (const auto & traffic_light_msg : traffic_light_array_msg_ptr->detections) {
    uint64_t traffic_light_id = stoull(traffic_light_msg.id);

    found_id[traffic_light_id] = true;
    if (traffic_light_list_.find(traffic_light_id) == nullptr) {
      add_traffic_light(std::make_shared<vision_msgs::msg::Detection3D>(traffic_light_msg));
      traffic_light_list_.insert(traffic_light_id);
    } else {
      update_traffic_light(std::make_shared<vision_msgs::msg::Detection3D>(traffic_light_msg));
    }
  }

  for (auto it = traffic_light_list_.begin(); it != traffic_light_list_.end(); it++) {
    if (found_id[*it]) {
      remove_traffic_light(*it);
      it = traffic_light_list_.erase(it);
    } else {
      it++;
    }
  }
}

void HDMapRouter::process_traffic_sign_msg(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr)
{
  uint64_t traffic_sign_id = stoi(traffic_sign_msg_ptr->id);
  if (traffic_sign_list_.find(traffic_sign_id) == nullptr) {
    add_traffic_sign(traffic_sign_msg_ptr);
    traffic_sign_list_.insert(traffic_sign_id);
  } else {
    update_traffic_sign(traffic_sign_msg_ptr);
  }
}

void HDMapRouter::process_pedestrian_msg(const vision_msgs::msg::Detection3DArray::SharedPtr pedestrian_msg_ptr)
{
  std::set<uint64_t> current_pedestrian_ids;
  for (const auto & pedestrian_msg : pedestrian_msg_ptr->detections) {
    uint64_t pedestrian_id = stoi(pedestrian_msg.id);
    current_pedestrian_ids.insert(pedestrian_id);
    if (pedestrian_list_.find(pedestrian_id) == nullptr) {
      add_pedestrian(std::make_shared<vision_msgs::msg::Detection3D>(pedestrian_msg));
      pedestrian_list_.insert(pedestrian_id);
    } else {
      update_pedestrian(std::make_shared<vision_msgs::msg::Detection3D>(pedestrian_msg));
    }
  }

  // Handle removal of pedestrians that are no longer detected
  for (auto it = pedestrian_list_.begin(); it != pedestrian_list_.end();) {
    if (current_pedestrian_ids.find(*it) == current_pedestrian_ids.end()) {
      remove_pedestrian(*it);
      it = pedestrian_list_.erase(it);  // Erase and get next iterator
    } else {
      ++it;
    }
  }
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

// Updating Reg Elements in HD Map

void HDMapRouter::update_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr)
{
  TrafficLightState traffic_light_state = HDMapRouter::get_traffic_light_state(traffic_light_msg_ptr);
  if (traffic_light_state == TrafficLightState::UnknownLight) {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Light Type Does Not Exist in Vocabulary!");
  }

  uint64_t traffic_light_id = std::stoull(traffic_light_msg_ptr->id);

  auto bbox = traffic_light_msg_ptr->bbox;

  lanelet::BoundingBox3d traffic_light_bbox = utils::detection3dToLaneletBBox(bbox);

  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);

  // Find the existing traffic light regulatory element
  for (const auto & reg_elem : lanelet_ptr_->regulatoryElementLayer) {
    auto traffic_light_elem = std::dynamic_pointer_cast<TrafficLightRegElem>(reg_elem);

    if (traffic_light_elem && traffic_light_elem->getId() == traffic_light_id) {
      traffic_light_elem->updateTrafficLight(traffic_light_bbox, traffic_light_state);

      // Re-associate the updated regulatory element with the appropriate lanelet if necessary
      lanelet::Lanelet current_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
      current_lanelet.addRegulatoryElement(traffic_light_elem);  // if duplicate, no addition

      RCLCPP_INFO(
        rclcpp::get_logger("hd_map_router"),
        "Updated traffic light in lanelet map: ID = %lu, New Position = (%f, %f, %f)",
        traffic_light_id,
        bbox.center.position.x,
        bbox.center.position.y,
        bbox.center.position.z);
    }
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("hd_map_router"), "Traffic light with ID %lu not found for update.", traffic_light_id);
}

// TODO(wato): implement updating traffic sign in the HD Map

void HDMapRouter::update_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr)
{
  std::string traffic_sign_class_id = get_detection3d_class(traffic_sign_msg_ptr);
  TrafficSignSubtype traffic_sign_subtype = TrafficSignRegElem::getSubtypeFromClassId(traffic_sign_class_id);
  // TrafficSignSubtype traffic_sign_subtype = TrafficSignSubtype::UnknownSign;

  if (traffic_sign_subtype == TrafficSignSubtype::UnknownSign) {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Sign Type Does Not Exist in Vocabulary!");
  }

  uint64_t traffic_sign_id = std::stoull(traffic_sign_msg_ptr->id);

  // auto pose = traffic_sign_msg_ptr->results.front().pose.pose;

  // transform pose to Point3d
  // lanelet::Point3d traffic_sign_position = lanelet::Point3d(pose.position.x, pose.position.y,
  // pose.position.z);

  auto bbox = traffic_sign_msg_ptr->bbox;

  lanelet::BoundingBox3d traffic_sign_bbox = utils::detection3dToLaneletBBox(bbox);

  // use bbox center for the pose? or pose, since pose is pose with converiance

  // bbox implementation
  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);

  // pose implementation
  // lanelet::ConstLanelet nearest_lanelet = get_nearest_lanelet_to_xyz(
  //     pose.position.x, pose.position.y, pose.position.z);

  // Find the existing traffic light regulatory element
  for (const auto & reg_elem : lanelet_ptr_->regulatoryElementLayer) {
    auto traffic_sign_elem = std::dynamic_pointer_cast<TrafficSignRegElem>(reg_elem);

    if (traffic_sign_elem && traffic_sign_elem->getId() == traffic_sign_id) {
      traffic_sign_elem->updateTrafficSign(traffic_sign_bbox, traffic_sign_subtype);

      // Re-associate the updated regulatory element with the appropriate lanelet if necessary
      lanelet::Lanelet current_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
      current_lanelet.addRegulatoryElement(traffic_sign_elem);  // if duplicate, no addition

      RCLCPP_INFO(
        rclcpp::get_logger("hd_map_router"),
        "Updated Traffic Sign in lanelet map: ID = %lu, New Position = (%f, %f, %f)",
        traffic_sign_id,
        bbox.center.position.x,
        bbox.center.position.y,
        bbox.center.position.z);
    }
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("hd_map_router"), "Traffic Sign with ID %lu not found for update.", traffic_sign_class_id);
}

void HDMapRouter::update_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr pedestrian_msg_ptr)
{
  std::string pedestrian_class = get_detection3d_class(pedestrian_msg_ptr);
  if (pedestrian_class != "PEDESTRIAN") {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Received non-pedestrian message in update_pedestrian function!");
    return;
  }

  uint64_t pedestrian_id = std::stoull(pedestrian_msg_ptr->id);

  auto bbox = pedestrian_msg_ptr->bbox;

  lanelet::BoundingBox3d new_pedestrian_bbox = lanelet::BoundingBox3d(
    lanelet::BasicPoint3d(
      bbox.center.position.x - bbox.size.x / 2,
      bbox.center.position.y - bbox.size.y / 2,
      bbox.center.position.z - bbox.size.z / 2),
    lanelet::BasicPoint3d(
      bbox.center.position.x + bbox.size.x / 2,
      bbox.center.position.y + bbox.size.y / 2,
      bbox.center.position.z + bbox.size.z / 2));

  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);

  // Find the existing pedestrian regulatory element
  for (const auto & reg_elem : lanelet_ptr_->regulatoryElementLayer) {
    auto pedestrian_elem = std::dynamic_pointer_cast<PedestrianRegElem>(reg_elem);
    if (pedestrian_elem && pedestrian_elem->getId() == pedestrian_id) {
      pedestrian_elem->updatePedestrian(new_pedestrian_bbox);

      // Re-associate the updated regulatory element with the appropriate lanelet if necessary
      lanelet::Lanelet mutable_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
      mutable_lanelet.addRegulatoryElement(pedestrian_elem);

      RCLCPP_INFO(
        rclcpp::get_logger("hd_map_router"),
        "Updated pedestrian in the lanelet map: ID = %lu, New Position = (%f, %f, %f)",
        pedestrian_id,
        bbox.center.position.x,
        bbox.center.position.y,
        bbox.center.position.z);
      return;
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Pedestrian with ID %lu not found for update.", pedestrian_id);
}

// Adding Reg Elements to HD Map

void HDMapRouter::add_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr)
{
  TrafficLightState traffic_light_state = HDMapRouter::get_traffic_light_state(traffic_light_msg_ptr);
  if (traffic_light_state == TrafficLightState::UnknownLight) {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Light Type Does Not Exist in Vocabulary!");
  }

  uint64_t traffic_light_id = std::stoull(traffic_light_msg_ptr->id);

  auto bbox = traffic_light_msg_ptr->bbox;

  lanelet::BoundingBox3d traffic_light_bbox = utils::detection3dToLaneletBBox(bbox);

  auto traffic_light_elem = TrafficLightRegElem::make(traffic_light_bbox, traffic_light_state, traffic_light_id);

  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);
  lanelet::Lanelet current_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
  current_lanelet.addRegulatoryElement(traffic_light_elem);

  lanelet_ptr_->add(traffic_light_elem);

  RCLCPP_INFO(
    rclcpp::get_logger("hd_map_router"),
    "Added traffic light to the lanelet map: ID = %lu, Position = (%f, %f, %f)",
    traffic_light_id,
    bbox.center.position.x,
    bbox.center.position.y,
    bbox.center.position.z);
}

// TODD: Implement adding traffic sign to the HD Map
void HDMapRouter::add_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr)
{
  std::string traffic_sign_class_id = get_detection3d_class(traffic_sign_msg_ptr);
  TrafficSignSubtype traffic_sign_subtype = TrafficSignRegElem::getSubtypeFromClassId(traffic_sign_class_id);

  if (traffic_sign_subtype == TrafficSignSubtype::UnknownSign) {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Sign Type Does Not Exist in Vocabulary!");
  }

  uint64_t traffic_sign_id = std::stoull(traffic_sign_msg_ptr->id);

  auto bbox = traffic_sign_msg_ptr->bbox;

  lanelet::BoundingBox3d traffic_sign_bbox = utils::detection3dToLaneletBBox(bbox);

  auto traffic_sign_elem = TrafficSignRegElem::make(traffic_sign_bbox, traffic_sign_subtype, traffic_sign_id);

  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);

  lanelet::Lanelet current_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
  current_lanelet.addRegulatoryElement(traffic_sign_elem);

  lanelet_ptr_->add(traffic_sign_elem);

  RCLCPP_INFO(
    rclcpp::get_logger("hd_map_router"),
    "Added traffic sign to the lanelet map: ID = %lu, Position = (%f, %f, %f)",
    traffic_sign_id,
    bbox.center.position.x,
    bbox.center.position.y,
    bbox.center.position.z);
}

void HDMapRouter::add_pedestrian(const vision_msgs::msg::Detection3D::SharedPtr pedestrian_msg_ptr)
{
  std::string pedestrian_class = get_detection3d_class(pedestrian_msg_ptr);

  if (pedestrian_class != "PEDESTRIAN") {
    RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Received non-pedestrian message in add_pedestrian function!");
    return;
  }

  uint64_t pedestrian_id = std::stoull(pedestrian_msg_ptr->id);

  auto bbox = pedestrian_msg_ptr->bbox;
  lanelet::BoundingBox3d new_pedestrian_bbox = lanelet::BoundingBox3d(
    lanelet::BasicPoint3d(
      bbox.center.position.x - bbox.size.x / 2,
      bbox.center.position.y - bbox.size.y / 2,
      bbox.center.position.z - bbox.size.z / 2),
    lanelet::BasicPoint3d(
      bbox.center.position.x + bbox.size.x / 2,
      bbox.center.position.y + bbox.size.y / 2,
      bbox.center.position.z + bbox.size.z / 2));

  lanelet::ConstLanelet nearest_lanelet =
    get_nearest_lanelet_to_xyz(bbox.center.position.x, bbox.center.position.y, bbox.center.position.z);

  auto pedestrian_reg_elem = PedestrianRegElem::make(new_pedestrian_bbox, pedestrian_id);

  // Add the regulatory element to the lanelet
  lanelet::Lanelet mutable_lanelet = lanelet_ptr_->laneletLayer.get(nearest_lanelet.id());
  mutable_lanelet.addRegulatoryElement(pedestrian_reg_elem);

  // Add the pedestrian to the map
  lanelet_ptr_->add(pedestrian_reg_elem);

  RCLCPP_INFO(
    rclcpp::get_logger("hd_map_router"),
    "Added pedestrian to the lanelet map: ID = %lu, Position = (%f, %f, %f)",
    pedestrian_id,
    bbox.center.position.x,
    bbox.center.position.y,
    bbox.center.position.z);
}

// Removing Reg Elements from HD Map

void HDMapRouter::remove_traffic_light(uint64_t traffic_light_id)
{
  for (const auto & reg_elem : lanelet_ptr_->regulatoryElementLayer) {
    auto traffic_light_elem = std::dynamic_pointer_cast<TrafficLightRegElem>(reg_elem);
    if (traffic_light_elem && traffic_light_elem->getId() == traffic_light_id) {
      for (auto & lanelet : lanelet_ptr_->laneletLayer) {
        lanelet.removeRegulatoryElement(traffic_light_elem);
      }

      RCLCPP_INFO(
        rclcpp::get_logger("hd_map_router"), "Removed traffic light from the lanelet map: ID = %lu", traffic_light_id);
      return;
    }
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("hd_map_router"), "Traffic light with ID %lu not found for removal.", traffic_light_id);
}

void HDMapRouter::remove_pedestrian(uint64_t pedestrian_id)
{
  for (const auto & reg_elem : lanelet_ptr_->regulatoryElementLayer) {
    auto pedestrian_elem = std::dynamic_pointer_cast<PedestrianRegElem>(reg_elem);
    if (pedestrian_elem && pedestrian_elem->getId() == pedestrian_id) {
      for (auto & lanelet : lanelet_ptr_->laneletLayer) {
        lanelet.removeRegulatoryElement(pedestrian_elem);
      }

      RCLCPP_INFO(
        rclcpp::get_logger("hd_map_router"), "Removed pedestrian from the lanelet map: ID = %lu", pedestrian_id);
      return;
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Pedestrian with ID %lu not found for removal.", pedestrian_id);
}
