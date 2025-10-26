#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>
#include "bt_base.h"

class Sequence : public BT::ControlNode {

    public:
        Sequence(const std::string& name, const BT::NodeConfig& config) : BT::ControlNode(name, config) {}
        
        BT::NodeStatus tick() override {
            for(int i = 0; i < children_nodes_.size(); i++) {
                if(children_nodes_[i]->executeTick() == BT::NodeStatus::RUNNING) {
                    return BT::NodeStatus::RUNNING;
                } else if(children_nodes_[i]->executeTick() == BT::NodeStatus::FAILURE) {
                    children_nodes_[i + 1]->halt();
                    return BT::NodeStatus::FAILURE;
                }
            }

            return BT::NodeStatus::SUCCESS;
        }
};