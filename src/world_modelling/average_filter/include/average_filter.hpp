#ifndef AVERAGE_FILTER_HPP_
#define AVERAGE_FILTER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered_array.hpp"
#include "world_modelling_msgs/msg/filter_array_average.hpp"

namespace world_modelling
{

/**
 * Implementation for the internal logic for the AverageFilter ROS2
 * node performing data processing and validation.
 */
class AverageFilter
{
public:

public:
  /**
   * AverageFilter constructor.
   */
  AverageFilter();

  /**
   * Removes all messages in buffer. Called by the average filter
   * node after messages have been published.
   */
  void clear_buffer();

private:

};

}  // namespace world_modelling

#endif  // AVERAGE_FILTER_HPP_
