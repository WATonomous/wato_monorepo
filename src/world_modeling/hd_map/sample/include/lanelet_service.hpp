#ifndef WORLD_MODELING_HD_MAP_LANELET_SERVICE_HPP_
#define WORLD_MODELING_HD_MAP_LANELET_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "world_modeling_msgs/srv/lanelet_service.hpp"

#include "sample.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class LaneletService : public rclcpp::Node
{
  public:
    LaneletService();

};

#endif  // LANELET_SERVICE_HPP_
