#ifndef BEHAVIOUR__NODES__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour/utils/utils.hpp"

#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class GetRouteContextAction
 * @brief Locates ego on the route and extracts the next transition and target lanelet.
 */
class GetRouteContextAction : public BT::SyncActionNode
{
public:
  GetRouteContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<lanelet_msgs::srv::GetShortestRoute::Response>>("route"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("route_index_map"),
      BT::InputPort<std::shared_ptr<lanelet_msgs::msg::CurrentLaneContext>>("lane_ctx"),
      BT::OutputPort<types::LaneTransition>("out_transition"),
      BT::OutputPort<std::shared_ptr<lanelet_msgs::msg::Lanelet>>("out_next_lanelet")};
  }

  BT::NodeStatus tick() override
  {
    auto route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "route");
    auto map = ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "route_index_map");
    auto current_lane_context = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");

    if (!route || !map || !current_lane_context) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t current_id = current_lane_context->current_lanelet.id;
    auto current_it = map->find(current_id);
    if (current_it == map->end()) {
      // Current lanelet not found on route
      return BT::NodeStatus::FAILURE;
    }

    const std::size_t current_idx = current_it->second;
    if (current_idx >= route->lanelets.size() - 1) {
      // At last lanelet, no further transitions
      setOutput("out_transition", types::LaneTransition::SUCCESSOR);
      setOutput("out_next_lanelet", nullptr);
      return BT::NodeStatus::SUCCESS;
    }

    // Extract transition and next lanelet on route from the current lanelet (target_lanelet)
    const uint8_t transition = route->transitions[current_idx];
    const std::shared_ptr<lanelet_msgs::msg::Lanelet> next_lanelet = std::make_shared<lanelet_msgs::msg::Lanelet>(route->lanelets[current_idx + 1]);

    setOutput("out_transition", static_cast<types::LaneTransition>(transition));
    setOutput("out_next_lanelet", next_lanelet);

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_
