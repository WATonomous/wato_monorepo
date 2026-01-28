# Behavior Tree Naming & Structure Standards

To ensure consistency and maintainability across Eve's behavior engine, all new Behavior Tree (BT) nodes must follow these standards.

## 1. Directory Structure
Nodes are categorized by their underlying BehaviorTree.CPP class type and their ROS interaction.

include/behaviour/
├── actions/      # BT::RosActionNode (Async ROS 2 Actions)
├── services/     # BT::RosServiceNode (Sync ROS 2 Services)
├── conditions/   # BT::ConditionNode (State checks)
├── nodes/        # BT::SyncActionNode (Internal C++ logic)
└── decorators/   # BT::DecoratorNode (Child modifiers)
