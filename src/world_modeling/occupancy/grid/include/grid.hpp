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
  };
}  // namespace world_modeling::occupancy

#endif // WORLD_MODELING_OCCUPANCY_GRID_HPP_
