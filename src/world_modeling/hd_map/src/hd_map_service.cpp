#include <memory>

#include "hd_map_service.hpp"

HDMapService::HDMapService()
: Node("hd_map_service") {
  router_ = std::make_shared<HDMapRouter>();
  manager_ = std::make_shared<HDMapManager>(this->router_);

  // Map selection hardcoded for now
  RCLCPP_INFO(this->get_logger(), "Selecting Lanelet Map Town05.osm...\n");
  if(manager_->select_osm_map("/home/docker/ament_ws/src/maps/osm/Town05.osm")){
    RCLCPP_INFO(this->get_logger(), "Map Selection Successful!\n");
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Map Selection Failed.\n");
  }

  hd_map_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hd_map", 20);

  hd_map_visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(10000), std::bind(&HDMapService::publish_hd_map_marker, this));

}

void HDMapService::publish_hd_map_marker(){
  auto marker_array = world_modeling::hd_map::laneletMapAsMarkerArray(router_->get_lanelet());

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  hd_map_visualization_publisher_->publish(marker_array);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HDMapService>());
  rclcpp::shutdown();
  return 0;
}