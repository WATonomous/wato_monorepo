#ifndef WORLD_MODELING_HD_MAP_AVERAGE_FILTER_HPP_
#define WORLD_MODELING_HD_MAP_AVERAGE_FILTER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/filtered_array_average.hpp"

namespace world_modeling::hd_map
{
  /**
   * Implementation for the internal logic for the Sample ROS2
   * node performing data processing and validation.
   */
  class AverageFilter
  {

  public:
    /**
     * Sample constructor.
     */
    AverageFilter();

    sample_msgs::msg::FilteredArrayAverage getAverage(const sample_msgs::msg::FilteredArray::SharedPtr msg);
  };
}  // namespace world_modeling::hd_map

#endif // WORLD_MODELING_HD_MAP_SAMPLE_HPP_