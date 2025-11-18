#include "behaviour/bt_tut_01.hpp"

#define DEFAULT_BT_XML "/home/bolty/ament_ws/src/world_modeling/behaviour/behaviour_trees/gripper.xml"

int main()
{
  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  // The recommended way to create a Node is through inheritance.
  factory.registerNodeType<ApproachObject>("ApproachObject");

  // Registering a SimpleActionNode using a function pointer.
  // Here we prefer to use a lambda,but you can use std::bind too
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

  // You can also create SimpleActionNodes using methods of a class.
  GripperInterface gripper;

  factory.registerSimpleAction(
      "OpenGripper",
      std::bind(&GripperInterface::open, &gripper));

  factory.registerSimpleAction(
      "CloseGripper",
      std::bind(&GripperInterface::close, &gripper));

  // Trees are created at deployment-time (i.e. at run-time, but only
  // once at the beginning).

  // IMPORTANT: when the object "tree" goes out of scope, all the
  // TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(DEFAULT_BT_XML);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  tree.tickRoot();

  return 0;
}