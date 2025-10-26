#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>
#include "bt_base.h"

class Fallback : public BT::ControlNode {

    public:
        Fallback(const std::string& name, const BT::NodeConfig& config) : BT::ControlNode(name, config) {}
        
        BT::NodeStatus tick() override {
            for(int i = 0; i < children_nodes_.size(); i++) {
                if(children_nodes_[i]->executeTick() == BT::NodeStatus::SUCCESS) {
                    return BT::NodeStatus::FAILURE;
                } else if(children_nodes_[i]->executeTick() == BT::NodeStatus::RUNNING) {
                    return BT::NodeStatus::RUNNING;
                }
            }
            return BT::NodeStatus::FAILURE;
        }
};