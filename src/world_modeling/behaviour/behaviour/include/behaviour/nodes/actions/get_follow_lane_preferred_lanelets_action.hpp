#ifndef BEHAVIOUR__NODES__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{

/**
 * @class GetFollowLanePreferredLaneletsAction
 * @brief BT sync action to select preferred lanelet IDs for following the current lane NOT the route.
 */
class GetFollowLanePreferredLaneletsAction : public BT::SyncActionNode
{
public:
  GetFollowLanePreferredLaneletsAction(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx", "Current lane context"),
      BT::OutputPort<std::vector<int64_t>>("out_preferred_lanelet_ids", "Preferred lanelet IDs (straight only)"),
      BT::OutputPort<std::string>("error_message", "Error if failed"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!lane_ctx) {
      setOutput("error_message", "missing_lane_context");
      return BT::NodeStatus::FAILURE;
    }

    // Allows for right turn and any lanelets successor ids
    // This only disallows lane changes

    auto preferred = std::make_shared<std::vector<int64_t>>();

    preferred->push_back(lane_ctx->current_lanelet.id);
    for (const auto& id : lane_ctx->current_lanelet.successor_ids) {
      preferred->push_back(id);
    }

    setOutput("out_preferred_lanelet_ids", preferred);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif // BEHAVIOUR__NODES__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_
