#ifndef BEHAVIOUR__CAR_ON_ROUTE_CONDITION_HPP_
#define BEHAVIOUR__CAR_ON_ROUTE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "lanelet_msgs/srv/get_shortest_route.hpp" 
#include "lanelet_msgs/msg/current_lane_context.hpp"

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
 * @class CarOnRouteCondition
 * @brief Validates if the ego vehicle's current lanelet is present in the global route.
 */
class CarOnRouteCondition : public BT::ConditionNode
{
public:
  CarOnRouteCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { 
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, size_t>>>("route_index_map"),
      BT::InputPort<std::shared_ptr<lanelet_msgs::msg::CurrentLaneContext>>("lane_ctx")
    };
  }

  BT::NodeStatus tick() override {
    auto route_index_map = ports::tryGetPtr<std::unordered_map<int64_t, size_t>>(*this, "route_index_map");
    auto ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    
    // If either pointer is missing, we cannot verify position, so return failure
    if (!route_index_map) {
      std::cout << "[CarOnRoute]: Missing route index map" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!ctx) {
      std::cout << "[CarOnRoute]: Missing lane context" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // Check if current lanelet ID exists in the route index map
    auto it = route_index_map->find(ctx->current_lanelet.id);

    return (it != route_index_map->end()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__CAR_ON_ROUTE_CONDITION_HPP_