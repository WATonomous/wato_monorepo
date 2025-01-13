#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include <memory>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OccupancyCore {
 public:
  // Resolution of the costmap (in cells/m)
  int CELLS_PER_METER;

  /**
   * OccupancyCore constructor.
   */
  OccupancyCore();
  OccupancyCore(int resolution);

  /**
   * Removes the z-axis dimension from the given PointCloud2 message.
   *
   * @param msg The input PointCloud2 message
   * @returns the processed point cloud
   */
  nav_msgs::msg::OccupancyGrid remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif  // OCCUPANCY_HPP
