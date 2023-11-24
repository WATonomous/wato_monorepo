#ifndef WORLD_MODELING_HD_MAP_SERVICE_
#define WORLD_MODELING_HD_MAP_SERVICE_

#include "rclcpp/rclcpp.hpp"
#include "lanelet_visualization.hpp"
#include "hd_map_router.hpp"
#include "hd_map_manager.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <algorithm>

class HDMapService : public rclcpp::Node
{
  public:
    HDMapService();

  private:
    void obstacle_msg_subscriber(common_msgs::msg::Obstacle::SharedPtr obstacle_msg);
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

    rclcpp::Subscription<common_msgs::msg::Obstacle>::SharedPtr hd_map_obstacle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_point_subscriber_;

    rclcpp::TimerBase::SharedPtr hd_map_visualization_timer_;

    lanelet::Optional<lanelet::routing::LaneletPath> lanelet_path;

};

#endif
