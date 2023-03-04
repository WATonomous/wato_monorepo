#ifndef WORLD_MODELING_LOCALIZATION_SAMPLE_HPP_
#define WORLD_MODELING_LOCALIZATION_SAMPLE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"

namespace world_modeling::localization
{
  /**
   * Implementation for the internal logic for the Sample ROS2
   * node performing data processing and validation.
   */
  class Sample
  {

  public:
    /**
     * Sample constructor.
     */
    Sample();
  };
}  // namespace world_modeling::localization

#endif // WORLD_MODELING_LOCALIZATION_SAMPLE_HPP_