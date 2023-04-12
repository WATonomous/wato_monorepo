#include <string>
#include <vector>

#include "grid.hpp"

namespace world_modeling::occupancy
{
    Grid::Grid()
    {}

    bool Grid::arbitrary_occupancy(
        // the message needs to be replaced with the lidar message which does not exist yet from what I can see
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        nav_msgs::msg::OccupancyGrid & occupancy_grid)
    {
        // Some arbitrary transformation of lidar data to occupancy grid
        return true;
    }

}  // namespace world_modeling::hd_map
