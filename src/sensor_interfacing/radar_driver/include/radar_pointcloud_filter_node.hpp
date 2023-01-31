#ifndef RADAR_POINTCLOUD_FILTER_NODE_HPP_
#define RADAR_POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/UnfilteredRadarLeft.hpp"
#include "radar_msgs/msg/UnfilteredRadarRight.hpp"
#include "radar_msgs/msg/UnfilteredCarlaLeft.hpp"
#include "radar_msgs/msg/UnfilteredCarlaRight.hpp"

#include "radar_pointcloud_filter.hpp"

    /**
    * Implementation of a ROS2 Point Cloud Filter node that listens to "unfiltered" radar 
    * topics from CARLA ROS Bridge and ROSBags and publishes to "filtered" radar topic.
    */

class PointCloudFilterNode : public rclcpp:: Node
{
public:
    // Configure pubsub nodes to keep last 20 messages.
    // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
    static constexpr int ADVERTISING_FREQ = 20;

    /**
    * PointCloudFilter Node Constructor.
    */
    PointCloudFilterNode();

private:
   /**
   * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
   * topic 
   *
   * @param msg a raw message from the "unfiltered" topic
   */
   void unfiltered_radar_right_callback(
    const radar_msgs::msg::UnfilteredRadarRight::SharedPtr msg);


   /**
   * A ROS2 subscription node callback used to unpack raw radar data from the "unfiltered"
   * topic 
   *
   * @param msg a raw message from the "unfiltered" topic
   */
   void unfiltered_radar_left_callback(
    const radar_msgs::msg::UnfilteredRadarLeft::SharedPtr msg);


   /**
   * A ROS2 subscription node callback used to unpack raw CARLA radar data from 
   * "unfiltered" topics.
   *
   * @param msg a processed message from the "unfiltered" topic
   */
  void unfiltered_carla_radar_left_callback(
    const radar_msgs::msg::UnfilteredCarlaLeft::SharedPtr msg);


   /**
   * A ROS2 subscription node callback used to unpack raw CARLA radar data from 
   * "unfiltered" topics.
   *
   * @param msg a processed message from the "unfiltered" topic
   */
  void unfiltered_carla_radar_right_callback(
    const radar_msgs::msg::UnfilteredCarlaRight::SharedPtr msg);


  // ROS2 Subscriber listening to the unfiltered radar left topic.
  rclcpp::Subscription<radar_msgs::msg::UnfilteredRadarLeft>::SharedPtr raw_left_sub_

  // ROS2 Subscriber listening to the unfiltered radar right topic.
  rclcpp::Subscription<radar_msgs::msg::UnfilteredRadarRight>::SharedPtr raw_right_sub_

  // ROS2 Subscriber listening to the unfiltered Carla left topic (radar1).
  rclcpp::Subscription<radar_msgs::msg::UnfilteredCarlaLeft>::SharedPtr raw_carla_left_sub_

  // ROS2 Subscriber listening to the unfiltered Carla right topic (radar2).
  rclcpp::Subscription<radar_msgs::msg::UnfilteredCarlaRight>::SharedPtr raw_carla_right_sub_

  // Add an object below from radar_pointcloud_filter.hpp that contains the methods

};

#endif  // RADAR_POINTCLOUD_FILTER_NODE_HPP_
