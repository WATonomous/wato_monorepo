#ifndef RADAR_CENTERFUSION_HPP_
#define RADAR_CENTERFUSION_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

namespace inference
{

class RadarCenterFusion
{
public:
  RadarCenterFusion();

private:
};

}  // namespace inference

#endif  // RADAR_CENTERFUSION_HPP_