#ifndef WORLD_MODELING_HD_MAP_SERVICE_
#define WORLD_MODELING_HD_MAP_SERVICE_

#include "rclcpp/rclcpp.hpp"
#include "lanelet_visualization.hpp"
#include "hd_map_router.hpp"
#include "hd_map_manager.hpp"

class HDMapService : public rclcpp::Node
{
  public:
    HDMapService();

  private:
    void publish_hd_map_marker();
    
    std::shared_ptr<HDMapRouter> router_;
    std::shared_ptr<HDMapManager> manager_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_visualization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_route_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_start_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hd_map_end_publisher_;

    rclcpp::TimerBase::SharedPtr hd_map_visualization_timer_;

};

#endif
