#ifndef ARS_POINTCLOUD_FILTER_NODE_HPP_
#define ARS_POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/UnfilteredRadarLeftPacket.hpp"
#include "radar_msgs/msg/UnfilteredRadarRightPacket.hpp"
#include "radar_msgs/msg/UnfilteredCarlaLeftPacket.hpp"
#include "radar_msgs/msg/UnfilteredCarlaRightPacket.hpp"
#include "radar_msgs/msg/RadarDetection.hpp

#include "ars_pointcloud_filter.hpp"

    /**
    * Implementation of a ROS2 Point Cloud Filter node that listens to "unfiltered" radar 
    * topics from ARS ROSbags and publishes to "filtered" radar topic.
    */

class ARSPointCloudFilterNode : public rclcpp:: Node
{
public:
    // Configure pubsub nodes to keep last 20 messages.
    // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
    static constexpr int ADVERTISING_FREQ = 20;

    /**
    * PointCloudFilter Node Constructor.
    */
    ARSPointCloudFilterNode();

private:

   /**
   * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
   * topic 
   *
   * @param msg a raw message from the "unfiltered" topic
   */
   void unfiltered_ars_radar_right_callback(
    const radar_msgs::msg::UnfilteredRadarRightPacket::SharedPtr msg);


   /**
   * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
   * topic 
   *
   * @param msg a raw message from the "unfiltered" topic
   */
   void unfiltered_ars_radar_left_callback(
    const radar_msgs::msg::UnfilteredRadarLeftPacket::SharedPtr msg);


  // ROS2 Subscriber listening to the unfiltered radar left topic.
  rclcpp::Subscription<radar_msgs::msg::UnfilteredRadarLeftPacket>::SharedPtr raw_left_sub_

  // ROS2 Subscriber listening to the unfiltered radar right topic.
  rclcpp::Subscription<radar_msgs::msg::UnfilteredRadarRightPacket>::SharedPtr raw_right_sub_

  // Add an object below from radar_pointcloud_filter.hpp that contains the methods (ADD LATER ONCE LOGIC CREATED)

};



#endif  // ARS_POINTCLOUD_FILTER_NODE_HPP_
