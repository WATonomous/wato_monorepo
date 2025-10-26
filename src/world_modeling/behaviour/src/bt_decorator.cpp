#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorator_node.h>
#include "bt_base.h"

class Decorator : public BT::DecoratorNode {

    private:
        int attempts = 0;

    public:

        Decorator(const std::string& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config) {}

        static PortsList providedPorts() {
            return {BT::InputPort<int>("maxattempt")};
        }

        BT::NodeStatus tick() override {
            int maxattempt = getInput<int>("maxattempt").value();
            while(attempts < maxattempt) {
                if(child_node_->executeTick() == BT::NodeStatus::SUCCESS) {
                    return BT::NodeStatus::SUCCESS;
                }
                attempts++;
            }
            attempts = 0;
            return BT::NodeStatus::FAILURE;
        }
};