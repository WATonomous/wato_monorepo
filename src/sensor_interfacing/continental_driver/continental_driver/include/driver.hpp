#ifndef CONTINENTAL_DRIVER_DRIVER_HPP_
#define CONTINENTAL_DRIVER_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "continental_msgs/msg/ars_detection.hpp"
#include "continental_msgs/msg/ars_packet.hpp"

namespace continental_driver
{

class Driver
{
public:
  Driver();
  ~Driver() = default;
};

}  // namespace continental_driver

#endif  // CONTINENTAL_DRIVER_DRIVER_HPP_
