#ifndef WORLD_MODELING_OCCUPANCY_GRID_HPP_
#define WORLD_MODELING_OCCUPANCY_GRID_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
// #include <ros/ros.h>
// #include "sample_msgs/msg/point_cloud2.hpp"
// #include "sample_msgs/msg/occupancy_grid.hpp"

namespace world_modeling::occupancy
{
  /**
   * Implementation for the internal logic for the Grid ROS2
   * node performing data processing and validation.
   */
  class Grid
  {

  public:
    /**
     * Grid constructor.
     */
    Grid();

    bool arbitrary_occupancy(
      // the message needs to be replaced with the lidar message which does not exist yet from what I can see
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        nav_msgs::msg::OccupancyGrid & occupancy_grid);
  };
}  // namespace world_modeling::occupancy

#endif // WORLD_MODELING_OCCUPANCY_GRID_HPP_
