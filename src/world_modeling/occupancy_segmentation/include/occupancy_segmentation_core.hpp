#ifndef OCCUPANCY_SEGMENTATION_CORE_HPP_
#define OCCUPANCY_SEGMENTATION_CORE_HPP_

#include "sample_msgs/msg/unfiltered.hpp"

namespace world_modeling
{

/**
 * Implementation of the internal logic used by the OccupancySegmentation Node to
 * serialize and update coordinates.
 */
class OccupancySegmentationCore
{
public:
  /**
   * OccupancySegmentation constructor.
   *
   * @param x optionally set the initial value of the x-coordinate
   * @param y same as above but for the y-coordinate
   * @param z same as above but for the z-coordinate
   */
  // explicit OccupancySegmentationCore(float x = 0, float y = 0, float z = 0);

  /**
   * Modify the value of the velocity. Used by the OccupancySegmentation Node whenever
   * the velocity parameter is dynamically updated.
   *
   * @param velocity the new value of the velocity
   */
  // void update_velocity(int velocity);

  /**
   * Modify the value of the starting position. This is not dynamically
   * updated.
   *
   * @param pos_x the starting value of the x-coordinate
   * @param pos_y the starting value of the x-coordinate
   * @param pos_z the starting value of the x-coordinate
   */
  // void update_position(double pos_x, double pos_y, double pos_z);

  /**
   * Use the velocity parameter to increment the coordinate values.
   */
  // void update_coordinates();

  /**
   * Converts coordinate data into a string representation of the form
   * "x:num1;y:num2;z:num3;".
   *
   * @param[out] msg an unfiltered message with empty data field
   */
  // void serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const;

private:
  // Coordinate values
  // double pos_x_;
  // double pos_y_;
  // double pos_z_;

  // Used to increment the value of coordinates on fixed time intervals.
  // double velocity_;
};

}

#endif  // OCCUPANCY_SEGMENTATION_CORE_HPP_
