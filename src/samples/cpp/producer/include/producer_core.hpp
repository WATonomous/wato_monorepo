#ifndef PRODUCER_CORE_HPP_
#define PRODUCER_CORE_HPP_

#include "sample_msgs/msg/unfiltered.hpp"

namespace samples
{

/**
 * Implementation of the internal logic used by the Producer Node to
 * serialize and update coordinates.
 */
class ProducerCore
{
public:
  /**
   * Producer constructor.
   *
   * @param x optionally set the initial value of the x-coordinate
   * @param y same as above but for the y-coordinate
   * @param z same as above but for the z-coordinate
   */
  explicit ProducerCore(float x = 0, float y = 0, float z = 0);

  /**
   * Modify the value of the velocity. Used by the Producer Node whenever
   * the velocity parameter is dynamically updated.
   *
   * @param velocity the new value of the velocity
   */
  void update_velocity(int velocity);

  /**
   * Modify the value of the starting position. This is not dynamically
   * updated.
   *
   * @param pos_x the starting value of the x-coordinate
   * @param pos_y the starting value of the x-coordinate
   * @param pos_z the starting value of the x-coordinate
   */
  void update_position(double pos_x, double pos_y, double pos_z);

  /**
   * Use the velocity parameter to increment the coordinate values.
   */
  void update_coordinates();

  /**
   * Converts coordinate data into a string representation of the form
   * "x:num1;y:num2;z:num3;".
   *
   * @param[out] msg an unfiltered message with empty data field
   */
  void serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const;

private:
  // Coordinate values
  double pos_x_;
  double pos_y_;
  double pos_z_;

  // Used to increment the value of coordinates on fixed time intervals.
  double velocity_;
};

}  // namespace samples

#endif  // PRODUCER_CORE_HPP_
