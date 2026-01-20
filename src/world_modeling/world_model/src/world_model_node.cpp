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

#include "world_model/world_model_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>

namespace world_model
{

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions & options)
: Node("world_model", options),
  ego_pose_received_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("osm_map_path", "");
  this->declare_parameter<double>("lat_origin", 0.0);
  this->declare_parameter<double>("lon_origin", 0.0);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->declare_parameter<double>("agent_history_duration_sec", 5.0);
  this->declare_parameter<double>("agent_prune_timeout_sec", 2.0);
  this->declare_parameter<double>("traffic_light_timeout_sec", 1.0);
  this->declare_parameter<double>("lane_context_publish_rate_hz", 10.0);
  this->declare_parameter<double>("map_viz_publish_rate_hz", 1.0);
  this->declare_parameter<double>("map_viz_radius_m", 100.0);
  this->declare_parameter<std::string>("detections_topic", "/perception/detections3d");
  this->declare_parameter<std::string>("tl_detections_topic", "/perception/traffic_light/detections");
  this->declare_parameter<std::string>("predictions_topic", "/prediction/predictions");
  this->declare_parameter<std::string>("ego_pose_topic", "/localization/ego_pose");
  this->declare_parameter<std::string>("lane_context_topic", "~/lane_context");
  this->declare_parameter<std::string>("map_viz_topic", "~/map_visualization");

  // Get parameters
  std::string osm_map_path = this->get_parameter("osm_map_path").as_string();
  double lat_origin = this->get_parameter("lat_origin").as_double();
  double lon_origin = this->get_parameter("lon_origin").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  double agent_history_sec = this->get_parameter("agent_history_duration_sec").as_double();
  double agent_prune_sec = this->get_parameter("agent_prune_timeout_sec").as_double();
  double tl_timeout_sec = this->get_parameter("traffic_light_timeout_sec").as_double();
  double lane_context_rate = this->get_parameter("lane_context_publish_rate_hz").as_double();
  double map_viz_rate = this->get_parameter("map_viz_publish_rate_hz").as_double();
  map_viz_radius_m_ = this->get_parameter("map_viz_radius_m").as_double();

  std::string detections_topic = this->get_parameter("detections_topic").as_string();
  std::string tl_detections_topic = this->get_parameter("tl_detections_topic").as_string();
  std::string predictions_topic = this->get_parameter("predictions_topic").as_string();
  std::string ego_pose_topic = this->get_parameter("ego_pose_topic").as_string();
  std::string lane_context_topic = this->get_parameter("lane_context_topic").as_string();
  std::string map_viz_topic = this->get_parameter("map_viz_topic").as_string();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize lanelet handler
  lanelet_handler_ = std::make_unique<LaneletHandler>();

  // Load map if path provided
  if (!osm_map_path.empty()) {
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", osm_map_path.c_str());
    if (lanelet_handler_->loadMap(osm_map_path, lat_origin, lon_origin)) {
      RCLCPP_INFO(this->get_logger(), "Map loaded successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No map path provided, services will return errors");
  }

  // Initialize trackers
  agent_tracker_ = std::make_unique<AgentTracker>(
    lanelet_handler_.get(), agent_history_sec, agent_prune_sec);
  tl_tracker_ = std::make_unique<TrafficLightTracker>(
    lanelet_handler_.get(), tl_timeout_sec);

  // Create services
  route_srv_ = this->create_service<lanelet_msgs::srv::GetRoute>(
    "~/get_route",
    std::bind(&WorldModelNode::handleGetRoute, this,
    std::placeholders::_1, std::placeholders::_2));

  corridor_srv_ = this->create_service<lanelet_msgs::srv::GetCorridor>(
    "~/get_corridor",
    std::bind(&WorldModelNode::handleGetCorridor, this,
    std::placeholders::_1, std::placeholders::_2));

  reg_elem_srv_ = this->create_service<lanelet_msgs::srv::GetLaneletsByRegElem>(
    "~/get_lanelets_by_reg_elem",
    std::bind(&WorldModelNode::handleGetLaneletsByRegElem, this,
    std::placeholders::_1, std::placeholders::_2));

  // Create publishers
  lane_context_pub_ = this->create_publisher<lanelet_msgs::msg::CurrentLaneContext>(
    lane_context_topic, 10);
  map_viz_pub_ = this->create_publisher<lanelet_msgs::msg::MapVisualization>(
    map_viz_topic, 10);

  // Create subscriptions
  detections_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    detections_topic, 10,
    std::bind(&WorldModelNode::detectionsCallback, this, std::placeholders::_1));

  tl_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    tl_detections_topic, 10,
    std::bind(&WorldModelNode::tlDetectionsCallback, this, std::placeholders::_1));

  predictions_sub_ = this->create_subscription<prediction_msgs::msg::PredictionHypothesesArray>(
    predictions_topic, 10,
    std::bind(&WorldModelNode::predictionsCallback, this, std::placeholders::_1));

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    ego_pose_topic, 10,
    std::bind(&WorldModelNode::egoPoseCallback, this, std::placeholders::_1));

  // Create timers
  if (lane_context_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / lane_context_rate);
    lane_context_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WorldModelNode::publishLaneContext, this));
  }

  if (map_viz_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / map_viz_rate);
    map_viz_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WorldModelNode::publishMapVisualization, this));
  }

  RCLCPP_INFO(this->get_logger(), "WorldModelNode initialized");
}

void WorldModelNode::handleGetRoute(
  const std::shared_ptr<lanelet_msgs::srv::GetRoute::Request> request,
  std::shared_ptr<lanelet_msgs::srv::GetRoute::Response> response)
{
  auto result = lanelet_handler_->getRoute(request->from_lanelet_id, request->to_lanelet_id);
  *response = result;
}

void WorldModelNode::handleGetCorridor(
  const std::shared_ptr<lanelet_msgs::srv::GetCorridor::Request> request,
  std::shared_ptr<lanelet_msgs::srv::GetCorridor::Response> response)
{
  auto result = lanelet_handler_->getCorridor(
    request->from_lanelet_id,
    request->to_lanelet_id,
    request->max_length_m,
    request->sample_spacing_m,
    request->num_lanes_each_side);
  *response = result;
}

void WorldModelNode::handleGetLaneletsByRegElem(
  const std::shared_ptr<lanelet_msgs::srv::GetLaneletsByRegElem::Request> request,
  std::shared_ptr<lanelet_msgs::srv::GetLaneletsByRegElem::Response> response)
{
  auto result = lanelet_handler_->getLaneletsByRegElem(request->reg_elem_id);
  *response = result;
}

void WorldModelNode::detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  agent_tracker_->updateDetections(*msg, this->now());
}

void WorldModelNode::tlDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  tl_tracker_->update(*msg, this->now());
}

void WorldModelNode::predictionsCallback(
  const prediction_msgs::msg::PredictionHypothesesArray::SharedPtr msg)
{
  agent_tracker_->updatePredictions(*msg);
}

void WorldModelNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_ego_pose_ = *msg;
  ego_pose_received_ = true;
}

void WorldModelNode::publishLaneContext()
{
  if (!lanelet_handler_->isMapLoaded()) {
    return;
  }

  auto context = buildLaneContext();
  lane_context_pub_->publish(context);
}

void WorldModelNode::publishMapVisualization()
{
  if (!lanelet_handler_->isMapLoaded()) {
    return;
  }

  geometry_msgs::msg::PoseStamped ego_pose;
  if (!getEgoPoseFromTF(ego_pose) && !ego_pose_received_) {
    return;
  }

  if (ego_pose_received_) {
    ego_pose = current_ego_pose_;
  }

  lanelet_msgs::msg::MapVisualization msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;

  geometry_msgs::msg::Point center;
  center.x = ego_pose.pose.position.x;
  center.y = ego_pose.pose.position.y;
  center.z = ego_pose.pose.position.z;

  auto nearby_lanelets = lanelet_handler_->getLaneletsInRadius(center, map_viz_radius_m_);
  for (const auto & ll : nearby_lanelets) {
    msg.lanelets.push_back(lanelet_handler_->toLaneletMsg(ll));
  }

  map_viz_pub_->publish(msg);
}

lanelet_msgs::msg::CurrentLaneContext WorldModelNode::buildLaneContext()
{
  lanelet_msgs::msg::CurrentLaneContext context;
  context.header.stamp = this->now();
  context.header.frame_id = map_frame_;

  // Get ego pose
  geometry_msgs::msg::PoseStamped ego_pose;
  if (!getEgoPoseFromTF(ego_pose)) {
    if (ego_pose_received_) {
      ego_pose = current_ego_pose_;
    } else {
      return context;  // No pose available
    }
  }

  geometry_msgs::msg::Point ego_point;
  ego_point.x = ego_pose.pose.position.x;
  ego_point.y = ego_pose.pose.position.y;
  ego_point.z = ego_pose.pose.position.z;

  // Find nearest lanelet
  auto nearest_ll = lanelet_handler_->findNearestLanelet(ego_point);
  if (!nearest_ll.has_value()) {
    return context;
  }

  context.current_lanelet = lanelet_handler_->toLaneletMsg(nearest_ll.value());

  // Calculate arc length, lateral offset, and heading error
  // These would require more sophisticated projection onto the centerline
  // For now, provide placeholder values
  context.arc_length = 0.0;
  context.lateral_offset = 0.0;
  context.heading_error = 0.0;

  // Calculate distance to end of lanelet (simplified)
  if (!nearest_ll->centerline().empty()) {
    auto last_pt = nearest_ll->centerline().back();
    double dx = last_pt.x() - ego_point.x;
    double dy = last_pt.y() - ego_point.y;
    context.distance_to_lanelet_end_m = std::sqrt(dx * dx + dy * dy);
  }

  // Set distance to events (placeholder - would need to traverse routing graph)
  context.distance_to_intersection_m = -1.0;
  context.distance_to_traffic_light_m = -1.0;
  context.distance_to_stop_line_m = -1.0;
  context.distance_to_yield_m = -1.0;

  // Check current lanelet for regulatory elements
  if (context.current_lanelet.has_traffic_light) {
    context.distance_to_traffic_light_m = 0.0;  // In current lanelet
  }
  if (context.current_lanelet.has_stop_line) {
    context.distance_to_stop_line_m = 0.0;
  }
  if (context.current_lanelet.is_intersection) {
    context.distance_to_intersection_m = 0.0;
  }

  return context;
}

bool WorldModelNode::getEgoPoseFromTF(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, base_link_frame_, tf2::TimePointZero);

    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
}

}  // namespace world_model
