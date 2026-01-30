#ifndef BEHAVIOUR__EXECUTE_BEHAVIOUR_NODE_HPP_
#define BEHAVIOUR__EXECUTE_BEHAVIOUR_NODE_HPP_

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_topic_pub_node.hpp>

#include "behaviour/utils/ports.hpp"
#include "behaviour_msgs/msg/execute_behaviour.hpp" // Assuming msg equivalent exists

namespace behaviour
{
/**
 * @class ExecuteBehaviourPublisher
 * @brief Publishes a behaviour command (lane follow, lane change, etc.) to the planner.
 */
class ExecuteBehaviourPublisher : public BT::RosTopicPubNode<behaviour_msgs::msg::ExecuteBehaviour>
{
public:
  ExecuteBehaviourPublisher(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicPubNode<behaviour_msgs::msg::ExecuteBehaviour>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("behaviour", "The behaviour to execute"), BT::InputPort<std::vector<int64_t>>("preferred_lanelet_ids")});
  }

  /**
   * @brief Callback to fill the message. 
   * Returning true sends the message; returning false skips publishing.
   */
  bool setMessage(behaviour_msgs::msg::ExecuteBehaviour & msg) override
  {
    auto behaviour = ports::tryGet<std::string>(*this, "behaviour");
    auto preferred_lanelet_ids = ports::tryGet<std::vector<int64_t>>(*this, "preferred_lanelet_ids"); 
    if (!behaviour) {
      RCLCPP_WARN(node_->get_logger(), "[%s] No behaviour provided on port", name().c_str());
      return false;
    }

    if (!preferred_lanelet_ids) {
      RCLCPP_WARN(node_->get_logger(), "[%s] No preferred lanelet IDs provided on port", name().c_str());;
    }

    msg.behaviour = behaviour.value();
    msg.preferred_lanelet_ids = preferred_lanelet_ids.has_value() ? preferred_lanelet_ids.value() : std::vector<int64_t>{};
    
    RCLCPP_INFO(node_->get_logger(), "Publishing new behaviour: %s", msg.behaviour.c_str());
    RCLCPP_INFO(node_->get_logger(), "Preferred lanelet IDs count: %zu", msg.preferred_lanelet_ids.size());
    return true;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__EXECUTE_BEHAVIOUR_NODE_HPP_