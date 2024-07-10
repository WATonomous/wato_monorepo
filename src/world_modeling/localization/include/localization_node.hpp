#ifndef WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_
#define WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/unfiltered.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtsam/geometry/Point3.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "sample.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class LocalizationNode : public rclcpp::Node {
 public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Sample node constructor.
   */
  LocalizationNode();

 private:
  /**
   * A ROS2 subscription node callback used to process raw data from the
   * "unfiltered" topic and publish to the "filtered" topic.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void sample_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void sample_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void sample_publish(const sample_msgs::msg::Unfiltered::SharedPtr msg);

  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // ROS2 publishers
  rclcpp::Publisher<sample_msgs::msg::Unfiltered>::SharedPtr sample_pub_;

  // Object that handles data processing and validation.
  world_modeling::hd_map::Sample sample_;
};

#endif  // LOCALIZATION_NODE_HPP_