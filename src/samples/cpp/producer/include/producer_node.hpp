#ifndef PRODUCER_NODE_HPP_
#define PRODUCER_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sample_msgs/msg/unfiltered.hpp"

#include "producer_core.hpp"

/**
 * Implementation of a ROS2 node that generates unfiltered ROS2 messages on a
 * time interval.
 */
class ProducerNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Producer node constructor.
   *
   * @param delay_ms the frequency with which the node produces data.
   */
  explicit ProducerNode(int delay_ms);

private:
  /**
   * ROS timer callback used to trigger data generation and publish result
   * to the "unfiltered" topic.
   */
  void timer_callback();

  /**
   * Callback used to dynamically update velocity data at runtime.
   *
   * @param parameters list of parameters (only velocity in this case) that were modified
   * @returns status message indicating whether update was successful
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  // ROS2 publisher sending raw messages to the unfiltered topic.
  rclcpp::Publisher<sample_msgs::msg::Unfiltered>::SharedPtr data_pub_;

  // ROS2 timer used to call data generation callback at fixed intervals.
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback to dynamically modify node parameters.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Producer implementation containing logic for coordinate serialization and management.
  samples::ProducerCore producer_;
};

#endif  // PRODUCER_NODE_HPP_
