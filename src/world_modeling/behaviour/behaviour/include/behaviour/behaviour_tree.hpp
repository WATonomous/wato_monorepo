#ifndef BEHAVIOUR__BEHAVIOUR_TREE_HPP_
#define BEHAVIOUR__BEHAVIOUR_TREE_HPP_


#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace behaviour
{
/**
 * @class BehaviourTree
 * @brief Manages the BT factory, blackboard, and execution logic.
 */
class BehaviourTree
{
public:
  /**
   * @brief Initializes the tree, registers nodes, and loads the XML file.
   */
  BehaviourTree(
    rclcpp::Node::SharedPtr node,
    const std::string & tree_file_path,
    bool logging = false);

  /** @brief Performs a single tick of the behavior tree. */
  void tick();

  /** @brief Sets a value on the blackboard. */
  template <typename T>
  void updateBlackboard(const std::string & key, const T & value)
  {
    blackboard_->set<T>(key, value);
  }

  /** @brief Gets a value from the blackboard. */
  template <typename T>
  T getBlackboard(const std::string & key) const
  {
    return blackboard_->get<T>(key);
  }

private:
  /** @brief Registers custom BT nodes with the factory. */
  void registerNodes();

  /** @brief Loads the XML file and creates the internal tree. */
  void buildTree(const std::string & tree_file_path);

  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  
  rclcpp::Node::SharedPtr node_;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_TREE_HPP_