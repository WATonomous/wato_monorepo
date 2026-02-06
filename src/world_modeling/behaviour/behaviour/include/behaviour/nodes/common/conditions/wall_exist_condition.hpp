#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_ID_EXIST_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_ID_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
    class WallIdExistCondition : public BT::ConditionNode
    {
    public:
        WallIdExistCondition(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int32_t>("wall_id"),
            };
        }

        BT::NodeStatus tick() override
        {
            auto wall_id = ports::tryGet<int32_t>(*this, "wall_id");
            if (!wall_id)
            {
                std::cout << "[WallIdValid] wall_id missing -> treat as no-op" << std::endl;
                return BT::NodeStatus::FAILURE; // used for gating in a Fallback
            }

            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_ID_EXIST_CONDITION_HPP_
