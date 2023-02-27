#ifndef AVERAGER_HPP_
#define AVERAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/filtered_array_average.hpp"

namespace world_modeling
{

/**
 * Implementation of the internal logic used by the Averager Node to
 * calculate the operating frequency of topics.
 */
class Averager
{
public:
  /**
   * Averager constructor.
   *
   */
  explicit Averager();

  /**
   * Averages the x, y, and z floats for all Filtered messages in the FilteredArray message
   *
   * @param msg
   */
  void average_msg(
    const sample_msgs::msg::FilteredArray::SharedPtr msg,
    sample_msgs::msg::FilteredArrayAverage & avg_msg);

};

}  // namespace world_modeling

#endif  // AVERAGER_HPP_
