#ifndef WORLD_MODELING_HD_MAP_SERVICE_
#define WORLD_MODELING_HD_MAP_SERVICE_

#include "rclcpp/rclcpp.hpp"
#include "lanelet_visualization.hpp"
#include "hd_map_router.hpp"
#include "hd_map_manager.hpp"
#include "common_msgs/msg/obstacle.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_modeling_msgs/srv/lanelet_info.hpp"
#include <algorithm>

class HDMapService : public rclcpp::Node
{
  public:
    HDMapService();

  private:
    void hd_map_traffic_light_callback(vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg);
    void hd_map_traffic_sign_callback(vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg);
    void hd_map_pedestrian_callback(vision_msgs::msg::Detection3DArray::SharedPtr pedestrian_msg);
    void point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void get_desired_lane(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void publish_hd_map_marker();
    
    std::shared_ptr<HDMapRouter> router_;
    std::shared_ptr<HDMapManager> manager_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_visualization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_route_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_start_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_end_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_desired_lane_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_current_lane_publisher_;

    rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr hd_map_traffic_sign_subscriber_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr hd_map_traffic_light_subscriber_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr hd_map_pedestrian_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_point_subscriber_;

    rclcpp::TimerBase::SharedPtr hd_map_visualization_timer_;

    lanelet::Optional<lanelet::routing::LaneletPath> lanelet_path;

};

#endif