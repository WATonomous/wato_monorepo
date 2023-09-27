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
  hd_map_route_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hd_map_route", 20);
  hd_map_start_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hd_map_start_lanelet", 20);
  hd_map_end_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hd_map_end_lanelet", 20);

  hd_map_visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&HDMapService::publish_hd_map_marker, this));

}

void HDMapService::publish_hd_map_marker(){
  auto marker_array = world_modeling::hd_map::laneletMapAsMarkerArray(router_->get_lanelet());

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  hd_map_visualization_publisher_->publish(marker_array);

  auto pt1 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{0, 0, 0});
  auto pt2 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{-200, -200, 0});
  
  
  RCLCPP_INFO(this->get_logger(), "GPS Point 1: %f %f %f", pt1.lat, pt1.lon, pt1.ele);
  RCLCPP_INFO(this->get_logger(), "GPS Point 2: %f %f %f", pt2.lat, pt2.lon, pt2.ele);

  auto lanelet1 = router_->get_nearest_lanelet_to_gps(pt1);
  auto lanelet2 = router_->get_nearest_lanelet_to_gps(pt2);
  
  int id1 = 0;
  auto marker1 = world_modeling::hd_map::laneletAsMarkerArray(lanelet1, &id1);
  hd_map_start_publisher_->publish(marker1);
  int id2 = 0;
  auto marker2 = world_modeling::hd_map::laneletAsMarkerArray(lanelet2, &id2);
  hd_map_end_publisher_->publish(marker2);

  auto lanelet_path = router_->route(pt1, pt2);
  if(lanelet_path){
    RCLCPP_INFO(this->get_logger(), "Route Found. Publishing Route Markers...\n");
    auto path_marker_array = world_modeling::hd_map::laneletPathAsMarkerArray(*lanelet_path);
    hd_map_route_publisher_->publish(path_marker_array);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HDMapService>());
  rclcpp::shutdown();
  return 0;
}