#ifndef ARS_POINTCLOUD_FILTER_NODE_HPP_
#define ARS_POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "ars_pointcloud_filter.hpp"


/**
* Implementation of a ROS2 Point Cloud Filter node that listens to "unfiltered" radar 
* topics from ARS ROSbags and publishes to "filtered" radar topic.
*/
class ARSPointCloudFilterNode : public rclcpp::Node
{
public:

    /**
    * PointCloudFilter Node Constructor.
    */
    ARSPointCloudFilterNode();

    struct filter_parameters
    {
      std::string scan_mode;
      double vrel_rad_param;
      double el_ang_param;
      double rcs0_param;
      double snr_param;
      double range_param;
      double az_ang0_param;
    };

  filter_parameters parameters;

private:
  radar_msgs::msg::RadarPacket buffer_packet;
  
  /**
  * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
  * topic 
  *
  * @param msg a raw message from the "unfiltered" topic
  */
  void unfiltered_ars_radar_right_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  /**
  * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
  * topic 
  *
  * @param msg a raw message from the "unfiltered" topic
  */
  void unfiltered_ars_radar_left_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  // ROS2 Subscriber listening to the unfiltered radar packet topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_left_sub_;

  // ROS2 Subscriber listening to the unfiltered radar packet topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_right_sub_;

  // ROS2 publisher that sends filtered messages from left and right radar to the processed topic.
  rclcpp::Publisher<radar_msgs::msg::RadarPacket>::SharedPtr left_right_pub_;

  // Add an object below from radar_pointcloud_filter.hpp that contains the methods
  filtering::ARSPointCloudFilter pointcloudfilter_;

};


#endif  // ARS_POINTCLOUD_FILTER_NODE_HPP_
