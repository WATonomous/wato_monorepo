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

#include "hd_map/hd_map_service.hpp"

#include <memory>
#include <string>

HDMapService::HDMapService()
: Node("hd_map_service")
{
  // Declare parameters
  this->declare_parameter<std::string>("osm_map_filename", std::string("Town10HD.osm"));

  this->declare_parameter<std::string>("visualization_output_topic", std::string("hd_map_viz"));
  this->declare_parameter<std::string>("route_output_topic", std::string("hd_map_route"));
  this->declare_parameter<std::string>("start_output_topic", std::string("hd_map_start_lanelet"));
  this->declare_parameter<std::string>("end_output_topic", std::string("hd_map_end_lanelet"));
  this->declare_parameter<std::string>("desired_lane_output_topic", std::string("hd_map_desired_lane"));
  this->declare_parameter<std::string>("current_lane_output_topic", std::string("hd_map_current_lane"));

  this->declare_parameter<std::string>("traffic_light_input_topic", std::string("traffic_light"));
  this->declare_parameter<std::string>("traffic_sign_input_topic", std::string("traffic_sign"));
  this->declare_parameter<std::string>("pedestrian_input_topic", std::string("pedestrian"));
  this->declare_parameter<std::string>("point_input_topic", std::string("clicked_point"));
  this->declare_parameter<std::string>("query_point_input_topic", std::string("query_point"));

  // Get parameters
  std::string osm_map_filename = this->get_parameter("osm_map_filename").as_string();
  std::string visualization_output_topic = this->get_parameter("visualization_output_topic").as_string();
  std::string route_output_topic = this->get_parameter("route_output_topic").as_string();
  std::string start_output_topic = this->get_parameter("start_output_topic").as_string();
  std::string end_output_topic = this->get_parameter("end_output_topic").as_string();
  std::string desired_lane_output_topic = this->get_parameter("desired_lane_output_topic").as_string();
  std::string current_lane_output_topic = this->get_parameter("current_lane_output_topic").as_string();

  std::string traffic_light_input_topic = this->get_parameter("traffic_light_input_topic").as_string();
  std::string traffic_sign_input_topic = this->get_parameter("traffic_sign_input_topic").as_string();
  std::string pedestrian_input_topic = this->get_parameter("pedestrian_input_topic").as_string();
  std::string point_input_topic = this->get_parameter("point_input_topic").as_string();
  std::string query_point_input_topic = this->get_parameter("query_point_input_topic").as_string();

  router_ = std::make_shared<HDMapRouter>();
  manager_ = std::make_shared<HDMapManager>(this->router_);

  behaviour_tree_info_service = this->create_service<world_modeling_msgs::srv::BehaviourTreeInfo>(
    "behaviour_tree_info",
    std::bind(&HDMapService::behaviour_tree_info_callback, this, std::placeholders::_1, std::placeholders::_2));

  lanelet_info_gps_service_ = this->create_service<world_modeling_msgs::srv::LaneletInfoGPS>(
    "lanelet_info_gps",
    std::bind(&HDMapService::laneletInfoGPSCallback, this, std::placeholders::_1, std::placeholders::_2));

  lanelet_info_xyz_service_ = this->create_service<world_modeling_msgs::srv::LaneletInfoXYZ>(
    "lanelet_info_xyz",
    std::bind(&HDMapService::laneletInfoXYZCallback, this, std::placeholders::_1, std::placeholders::_2));

  lanelet_info_xy_service_ = this->create_service<world_modeling_msgs::srv::LaneletInfoXY>(
    "lanelet_info_xy",
    std::bind(&HDMapService::laneletInfoXYCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Map selection hardcoded for now

  std::string osm_map = manager_->get_maps_directory() + osm_map_filename;
  RCLCPP_INFO(this->get_logger(), "Selecting Lanelet Map %s...\n", osm_map.c_str());
  if (manager_->select_osm_map(osm_map)) {
    RCLCPP_INFO(this->get_logger(), "Map Selection Successful!\n");
  } else {
    RCLCPP_INFO(this->get_logger(), "Map Selection Failed.\n");
  }

  hd_map_visualization_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_output_topic, 20);
  hd_map_route_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(route_output_topic, 20);
  hd_map_start_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(start_output_topic, 20);
  hd_map_end_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(end_output_topic, 20);
  hd_map_desired_lane_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(desired_lane_output_topic, 20);
  hd_map_current_lane_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(current_lane_output_topic, 20);

  point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    point_input_topic, 20, std::bind(&HDMapService::point_callback, this, std::placeholders::_1));
  query_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    query_point_input_topic, 20, std::bind(&HDMapService::get_desired_lane, this, std::placeholders::_1));

  hd_map_visualization_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&HDMapService::publish_hd_map_marker, this));
}

void HDMapService::publish_hd_map_marker()
{
  auto marker_array = world_modeling::hd_map::laneletMapAsMarkerArray(router_->get_lanelet());

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  hd_map_visualization_publisher_->publish(marker_array);
}

void HDMapService::point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_point_ = msg->point;

  auto pt1 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{0, 0, 0});
  auto pt2 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{msg->point.x, msg->point.y, msg->point.z});

  RCLCPP_INFO(this->get_logger(), "GPS Point 1: %f %f %f", pt1.lat, pt1.lon, pt1.ele);
  RCLCPP_INFO(this->get_logger(), "GPS Point 2: %f %f %f", pt2.lat, pt2.lon, pt2.ele);

  auto lanelet1 = router_->get_nearest_lanelet_to_gps(pt1);
  auto lanelet2 = router_->get_nearest_lanelet_to_gps(pt2);

  std_msgs::msg::ColorRGBA color;
  color.g = 1;
  color.a = 1;

  int id1 = 0;

  auto marker1 = world_modeling::hd_map::laneletAsMarkerArray(lanelet1, &id1, false, true, color, color);
  hd_map_start_publisher_->publish(marker1);
  int id2 = 0;
  auto marker2 = world_modeling::hd_map::laneletAsMarkerArray(lanelet2, &id2, false, true, color, color);
  hd_map_end_publisher_->publish(marker2);

  lanelet_path = router_->route(pt1, pt2);
  if (lanelet_path) {
    RCLCPP_INFO(this->get_logger(), "Route Found. Publishing Route Markers...\n");
    for (auto lanelet = lanelet_path->begin(); lanelet != lanelet_path->end(); ++lanelet) {
      RCLCPP_INFO(this->get_logger(), "Lanelet: \n");
    }
    auto path_marker_array = world_modeling::hd_map::laneletPathAsMarkerArray(*lanelet_path);
    hd_map_route_publisher_->publish(path_marker_array);
  }
}

void HDMapService::get_desired_lane(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  current_point_ = msg->point;

  auto pt = router_->project_point3d_to_gps(lanelet::BasicPoint3d{msg->point.x, msg->point.y, msg->point.z});
  auto lanelet = router_->get_nearest_lanelet_to_gps(pt);

  current_lanelet_ = lanelet;

  if (lanelet_path) {
    auto it = std::find(lanelet_path->begin(), lanelet_path->end(), lanelet);
    if (it != lanelet_path->end()) {
      int idx = it - lanelet_path->begin();
      RCLCPP_INFO(this->get_logger(), "Found lanelet: %i\n", idx);

      visualization_msgs::msg::MarkerArray current_marker;
      visualization_msgs::msg::MarkerArray desired_marker;
      std_msgs::msg::ColorRGBA color;
      color.b = 1;
      color.a = 1;
      int id = 0;
      if (lanelet_path->getRemainingLane(it).size() > 1) {
        RCLCPP_INFO(this->get_logger(), "Using Current It");
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
      } else if (idx + 1 < lanelet_path->size()) {
        RCLCPP_INFO(this->get_logger(), "Using Next It");
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray(
          (*lanelet_path)[idx + 1], &id, false, true, color, color, .3, .4);
      } else {
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
      }

      hd_map_desired_lane_publisher_->publish(desired_marker);
      hd_map_current_lane_publisher_->publish(current_marker);
    }
  }
}

// TODO(wato) convert lanelet to lanelet msg

world_modeling_msgs::msg::Lanelet HDMapService::convert_lanelet_to_msg(const lanelet::ConstLanelet & lanelet)
{
  world_modeling_msgs::msg::Lanelet lanelet_msg;

  // convert left boundary
  for (const auto & point : lanelet.leftBound()) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    lanelet_msg.left_boundary.push_back(p);
  }

  // convert right boundary
  for (const auto & point : lanelet.rightBound()) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    lanelet_msg.left_boundary.push_back(p);
  }

  // convert centerline
  for (const auto & point : lanelet.centerline()) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    lanelet_msg.centerline.push_back(p);
  }

  lanelet_msg.id = lanelet.id();

  return lanelet_msg;
}

// TODO(wato) convert path to lanelet path msg

world_modeling_msgs::msg::LaneletPath HDMapService::convert_laneletPath_to_msg(
  const lanelet::Optional<lanelet::routing::LaneletPath> & path)
{
  world_modeling_msgs::msg::LaneletPath path_msg;

  if (path) {
    for (const auto & lanelet : *path) {
      path_msg.lanelets.push_back(this->convert_lanelet_to_msg(lanelet));
    }
  }

  return path_msg;
}

void HDMapService::behaviour_tree_info_callback(
  const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Request> request,
  const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Response> response)
{
  response->current_point = current_point_;
  response->goal_point = goal_point_;

  // response->current_lanelet = convert_lanelet_to_msg(current_lanelet_);
  // response->goal_lanelet = convert_lanelet_to_msg(goal_lanelet_);
  // response->route_list = convert_laneletPath_to_msg(lanelet_path);
}

void HDMapService::laneletInfoGPSCallback(
  const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoGPS::Request> request,
  std::shared_ptr<world_modeling_msgs::srv::LaneletInfoGPS::Response> response)
{
  lanelet::GPSPoint gps_point;
  gps_point.lat = request->latitude;
  gps_point.lon = request->longitude;
  gps_point.ele = request->altitude;

  lanelet::ConstLanelet closest_lanelet = router_->get_nearest_lanelet_to_gps(gps_point);
  response->lanelet = convert_lanelet_to_msg(closest_lanelet);
}

void HDMapService::laneletInfoXYZCallback(
  const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXYZ::Request> request,
  std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXYZ::Response> response)
{
  lanelet::ConstLanelet closest_lanelet = router_->get_nearest_lanelet_to_xyz(request->x, request->y, request->z);
  response->lanelet = convert_lanelet_to_msg(closest_lanelet);
}

void HDMapService::laneletInfoXYCallback(
  const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXY::Request> request,
  std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXY::Response> response)
{
  lanelet::ConstLanelet closest_lanelet =
    router_->get_nearest_lanelet_to_xy(request->x, request->y, request->width_x, request->height_y);
  response->lanelet = convert_lanelet_to_msg(closest_lanelet);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HDMapService>());
  rclcpp::shutdown();
  return 0;
}
