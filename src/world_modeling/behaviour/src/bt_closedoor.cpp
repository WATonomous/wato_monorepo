#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "bt_base.h"

class CloseDoor : public BT::ActionNodeBase {

    public:
        CloseDoor(const std::string& name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config) {}

        BT::NodeStatus tick() override {
            std::cout << "CloseDoor: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
};