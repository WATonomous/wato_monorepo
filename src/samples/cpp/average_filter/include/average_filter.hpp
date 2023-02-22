#ifndef AVERAGE_FILTER_HPP_
#define AVERAGE_FILTER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/filtered_array_average.hpp"

namespace samples
{

/**
 * Implementation for the internal logic for the Transformer ROS2
 * node performing data processing and validation.
 */
class AverageFilter
{
public:
  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

public:
  AverageFilter();

  // std::vector<sample_msgs::msg::FilteredArray> buffer_messages() const;
  // void clear_buffer();
  // bool enqueue_message(const sample_msgs::msg::FilteredArrayAverage & msg);


  bool average_coordinate(
    const sample_msgs::msg::FilteredArray::SharedPtr filteredArray,
    sample_msgs::msg::FilteredArrayAverage & filteredArrayAverage);

private:
  std::vector<sample_msgs::msg::FilteredArray> buffer_;
};

}  // namespace samples

#endif
