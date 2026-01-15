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

#ifndef WORLD_MODELING_HD_MAP_SERVICE_
#define WORLD_MODELING_HD_MAP_SERVICE_

#include <algorithm>
#include <memory>

#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "hd_map/hd_map_manager.hpp"
#include "hd_map/hd_map_router.hpp"
#include "hd_map/lanelet_visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_modeling_msgs/msg/lanelet.hpp"
#include "world_modeling_msgs/msg/lanelet_path.hpp"
#include "world_modeling_msgs/srv/behaviour_tree_info.hpp"
#include "world_modeling_msgs/srv/get_lane_objects.hpp"
#include "world_modeling_msgs/srv/get_lane_semantic.hpp"
#include "world_modeling_msgs/srv/lanelet_info.hpp"
#include "world_modeling_msgs/srv/lanelet_info_gps.hpp"
#include "world_modeling_msgs/srv/lanelet_info_xy.hpp"
#include "world_modeling_msgs/srv/lanelet_info_xyz.hpp"

class HDMapService : public rclcpp::Node
{
public:
  HDMapService();

private:
  /**
  * Handle incoming point messages for route visualization.
  *
  * @param msg the selected position
  */
  void point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
  * Get the desired lane based on current vehicle position.
  *
  * @param msg current vehicle position
  */
  void get_desired_lane(geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
  * Publish HD map visualization markers.
  *
  */
  void publish_hd_map_marker();

  /**
  * Handle behaviour tree info service requests.
  *
  * @param request service request containing query info
  * @param response response with current or goal points
  */
  void behaviour_tree_info_callback(
    const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Request> request,
    const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Response> response);

  /**
  * Handle lanelet lookup for a GPS point query.
  *
  * @param request service request containing query info.
  * @param response response closest lanelet to the GPS point.
  */
  void laneletInfoGPSCallback(
    const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoGPS::Request> request,
    std::shared_ptr<world_modeling_msgs::srv::LaneletInfoGPS::Response> response);

  /**
  * Handle lanelet lookup for a local XYZ point query.
  *
  * @param request service request containing query info.
  * @param response response closest lanelet to the XYZ point.
  */
  void laneletInfoXYZCallback(
    const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXYZ::Request> request,
    std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXYZ::Response> response);

  /**
  * Handle lanelet lookup for a 2D bounding box query.
  *
  * @param request service request containing query info.
  * @param response response closest lanelet to the bbox center.
  */
  void laneletInfoXYCallback(
    const std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXY::Request> request,
    std::shared_ptr<world_modeling_msgs::srv::LaneletInfoXY::Response> response);

  /**
  * Handle lane semantic lookup for a pose query.
  *
  * @param request service request containing pose and related query info.
  * @param response response lane semantic info for the nearest lanelet.
  */
  void get_lane_semantic_callback(
    const std::shared_ptr<world_modeling_msgs::srv::GetLaneSemantic::Request> request,
    std::shared_ptr<world_modeling_msgs::srv::GetLaneSemantic::Response> response);

  /**
  * Handle lane object lookup along a forward path on the routing graph
  *
  * @param request request service request containing pose, distance_ahead, lateral_radius.
  * @param response response lane objects found along the path.
  */
  void get_lane_objects_callback(
    const std::shared_ptr<world_modeling_msgs::srv::GetLaneObjects::Request> request,
    std::shared_ptr<world_modeling_msgs::srv::GetLaneObjects::Response> response);

  /**
  * Convert a lanelet to a ROS message
  *
  * @param lanelet the lanelet to convert
  * @return lanelet as a ROS message
  */
  world_modeling_msgs::msg::Lanelet convert_lanelet_to_msg(const lanelet::ConstLanelet & lanelet);

  /**
  * Convert a lanelet path to a ROS message
  *
  * @param path the lanelet path to convert
  * @return lanelet path as a ROS message
  */
  world_modeling_msgs::msg::LaneletPath convert_laneletPath_to_msg(
    const lanelet::Optional<lanelet::routing::LaneletPath> & path);

  rclcpp::Service<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedPtr behaviour_tree_info_service;
  rclcpp::Service<world_modeling_msgs::srv::LaneletInfoGPS>::SharedPtr lanelet_info_gps_service_;
  rclcpp::Service<world_modeling_msgs::srv::LaneletInfoXYZ>::SharedPtr lanelet_info_xyz_service_;
  rclcpp::Service<world_modeling_msgs::srv::LaneletInfoXY>::SharedPtr lanelet_info_xy_service_;
  rclcpp::Service<world_modeling_msgs::srv::GetLaneSemantic>::SharedPtr lane_semantic_service_;
  rclcpp::Service<world_modeling_msgs::srv::GetLaneObjects>::SharedPtr lane_objects_service_;

  std::shared_ptr<HDMapRouter> router_;
  std::shared_ptr<HDMapManager> manager_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_visualization_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_route_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_start_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_end_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_desired_lane_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_current_lane_publisher_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr hd_map_pedestrian_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_point_subscriber_;

  rclcpp::TimerBase::SharedPtr hd_map_visualization_timer_;

  lanelet::Optional<lanelet::routing::LaneletPath> lanelet_path;

  geometry_msgs::msg::Point goal_point_;
  geometry_msgs::msg::Point current_point_;
  lanelet::Lanelet goal_lanelet_;
  lanelet::ConstLanelet current_lanelet_;
};

#endif
