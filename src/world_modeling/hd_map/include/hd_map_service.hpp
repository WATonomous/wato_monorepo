#ifndef WORLD_MODELING_HD_MAP_SERVICE_
#define WORLD_MODELING_HD_MAP_SERVICE_

#include <algorithm>
#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "hd_map_manager.hpp"
#include "hd_map_router.hpp"
#include "lanelet_visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_modeling_msgs/msg/lanelet.hpp"
#include "world_modeling_msgs/msg/lanelet_path.hpp"
#include "world_modeling_msgs/srv/behaviour_tree_info.hpp"
#include "world_modeling_msgs/srv/lanelet_info.hpp"

class HDMapService : public rclcpp::Node {
 public:
  HDMapService();

 private:
  void hd_map_traffic_light_callback(
      vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg);
  void hd_map_traffic_sign_callback(vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg);
  void hd_map_pedestrian_callback(vision_msgs::msg::Detection3DArray::SharedPtr pedestrian_msg);
  void point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
  void get_desired_lane(geometry_msgs::msg::PointStamped::SharedPtr msg);
  void publish_hd_map_marker();
  void behaviour_tree_info_callback(
      const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Request> request,
      const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Response> response);
  world_modeling_msgs::msg::Lanelet convert_lanelet_to_msg(const lanelet::ConstLanelet& lanelet);
  world_modeling_msgs::msg::LaneletPath convert_laneletPath_to_msg(
      const lanelet::Optional<lanelet::routing::LaneletPath>& path);

  rclcpp::Service<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedPtr
      behaviour_tree_info_service;

  std::shared_ptr<HDMapRouter> router_;
  std::shared_ptr<HDMapManager> manager_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      hd_map_visualization_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_route_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_start_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_end_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_desired_lane_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_current_lane_publisher_;

  rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr hd_map_traffic_sign_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr
      hd_map_traffic_light_subscriber_;
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
