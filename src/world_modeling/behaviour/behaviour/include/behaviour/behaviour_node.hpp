#ifndef BEHAVIOUR__BEHAVIOUR_NODE_HPP_
#define BEHAVIOUR__BEHAVIOUR_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "behaviour/behaviour_tree.hpp"
#include "behaviour/dynamic_object_store.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
 * @class BehaviourNode
 * @brief A standard node that manages Eve's behavior tree and handles world state updates.
 */
class BehaviourNode : public rclcpp::Node
{
public:
  /**
   * @brief Initializes parameters, TF listeners, subscribers, and the behavior tree.
   * @param options Node options for configuration.
   */
  explicit BehaviourNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Virtual destructor.
   */
  virtual ~BehaviourNode() = default;

private:
  /**
   * @brief Time callback that ticks the behavior tree
   */
  void tickTreeTimerCallback();

  /**
   * @brief Timer callback to look up the latest ego state from TF and updates the blackboard.
   */
  void egoStateTimerCallback();

  // Core component
  std::shared_ptr<BehaviourTree> tree_;

  // TF for odom and pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Communications
  rclcpp::TimerBase::SharedPtr tick_tree_timer_;
  rclcpp::TimerBase::SharedPtr ego_state_timer_;
  rclcpp::Subscription<world_model_msgs::msg::DynamicObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr current_lane_context_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_point_sub_;

  // Configuration
  std::string map_frame_;
  std::string base_frame_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_NODE_HPP_