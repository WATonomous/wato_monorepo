#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include <vector>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>


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
  sensor_msgs::msg::PointCloud2 remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg);
 
 private:
  // Buffer storing processed messages until BUFFER_CAPACITY. Clear after
  // messages are published.
  std::vector<sensor_msgs::msg::PointCloud2> buffer_;
};

#endif // OCCUPANCY_HPP
