#ifndef BEHAVIOUR__GLOBAL_ROUTE_EXIST_CONDITION_HPP_
#define BEHAVIOUR__GLOBAL_ROUTE_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "lanelet_msgs/srv/get_shortest_route.hpp"

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
  /**
   * @class GlobalRouteExistCondition
   * @brief Checks if a global route exists in the provided route response.
   */
  class GlobalRouteExistCondition : public BT::ConditionNode
  {
  public:
    GlobalRouteExistCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::shared_ptr<lanelet_msgs::srv::GetShortestRoute::Response>>("route")};
    }

    BT::NodeStatus tick() override
    {
      auto route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "route");

      if (!route || route->lanelets.empty())
      {
        std::cout << "[GlobalRouteExist]: Route does not exist or is empty" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__GLOBAL_ROUTE_EXIST_CONDITION_HPP_