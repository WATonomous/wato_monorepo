#ifndef WORLD_MODELING_OCCUPANCY_GRID_HPP_
#define WORLD_MODELING_OCCUPANCY_GRID_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"

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

    bool Grid::arbitrary_occupancy(
      // the message needs to be replaced with the lidar message which does not exist yet from what I can see
        const sample_msgs::msg::THIS NEEDS TO BE CHANGED::SharedPtr msg,
        sample_msgs::msg::OccupancyGrid & occupancy_grid);
  };
}  // namespace world_modeling::occupancy

#endif // WORLD_MODELING_OCCUPANCY_GRID_HPP_
