#ifndef ROS2_BT_TUT__BT_TUT_01_HPP_
#define ROS2_BT_TUT__BT_TUT_01_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  // You must override the virtual function tick()
  NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return NodeStatus::SUCCESS;
  }
};

NodeStatus CheckBattery()
{
  std::cout << "Battery OK" << std::endl;
  return NodeStatus::SUCCESS;
}

class GripperInterface
{
public:
  GripperInterface() : _open(true) {}

  NodeStatus open()
  {
    _open = true;
    std::cout << "Gripper open" << std::endl;
    return NodeStatus::SUCCESS;
  }

  NodeStatus close()
  {
    _open = false;
    std::cout << "Gripper close" << std::endl;
    return NodeStatus::SUCCESS;
  }

private:
  bool _open;
};

#endif // ROS2_BT_TUT__BT_TUT_01_HPP_