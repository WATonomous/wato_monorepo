#ifndef AVERAGE_FILTER_HPP_
#define AVERAGE_FILTER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/filter_array_average.hpp"

namespace samples
{

/**
 * Implementation for the internal logic for the AverageFilter ROS2
 * node performing data processing.
 */
class AverageFilter
{
public:
  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

  /**
   * AverageFilter constructor.
   */
  AverageFilter();
};

}  // namespace samples

#endif  // AVERAGE_FILTER_HPP_
