#ifndef PRODUCER_HPP_
#define PRODUCER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"

namespace samples
{

/**
 * Implementation of the internal logic used by the Producer Node to
 * serialize and update coordinates.
 */
class Producer
{
public:
  /**
   * Producer constructor.
   *
   * @param x optionally set the initial value of the x-coordinate
   * @param y same as above but for the y-coordinate
   * @param z same as above but for the z-coordinate
   */
  explicit Producer(int x = 0, int y = 0, int z = 0);

  /**
   * Modify the value of the velocity. Used by the Producer Node whenever
   * the velocity parameter is dynamically updated.
   *
   * @param velocity the new value of the velocity
   */
  void update_velocity(int velocity);

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
  int pos_x_;
  int pos_y_;
  int pos_z_;

  // Used to increment the value of coordinates on fixed time intervals.
  int velocity_;
};

}  // namespace samples

#endif  // PRODUCER_HPP_
