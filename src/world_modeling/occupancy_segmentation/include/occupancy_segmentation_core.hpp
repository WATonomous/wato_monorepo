#ifndef OCCUPANCY_SEGMENTATION_CORE_HPP_
#define OCCUPANCY_SEGMENTATION_CORE_HPP_

#include <vector>


/**
 * Implementation for the internal logic for the Transformer ROS2
 * node performing data processing and validation.
 */
class OccupancySegmentationCore {
 public:
  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

 public:
  /**
   * Transformer constructor.
   */
  OccupancySegmentationCore();
};


#endif  // TRANSFORMER_CORE_HPP_
