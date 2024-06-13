#ifndef OCCUPANCY_SEGMENTATION_NODE_HPP_
#define OCCUPANCY_SEGMENTATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "occupancy_segmentation_core.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class OccupancySegmentationNode : public rclcpp::Node {
 public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Transformer node constructor.
   */
  OccupancySegmentationNode();

 private:

  // Object that handles data processing and validation.

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};

#endif  // TRANSFORMER_NODE_HPP_
