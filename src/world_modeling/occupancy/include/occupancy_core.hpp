#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include <vector>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


class OccupancyCore {
 public:
  /**
   * OccupancyCore constructor.
   */
  OccupancyCore();

  /**
   * Removes the z-axis dimension from the given PointCloud2 message.
   *
   * @param msg The input PointCloud2 message
   * @returns the processed point cloud
   */
  nav_msgs::msg::OccupancyGrid remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif // OCCUPANCY_HPP
