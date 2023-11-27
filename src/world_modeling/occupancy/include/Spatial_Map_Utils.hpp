#ifndef WORLD_MODELING_SPATIAL_MAP_UTILS_
#define WORLD_MODELING_SPATIAL_MAP_UTILS_

#include <vector>
#include "rclcpp/rclcpp.hpp"

/**
  * Adds a vehicle to the spatial temporal map 
  * 
  * @param vehicle The obstacle message including the bounding box for the vehicle to add
  * @param map A shared pointer to the spatial temportal map 
  * @return void
  */
void add_vehicle_to_map(common_msgs::msg::Obstacle vehicle, nav_msgs::msg::OccupancyGrid::SharedPtr map);

#endif // WORLD_MODELING_SPATIAL_MAP_UTILS_