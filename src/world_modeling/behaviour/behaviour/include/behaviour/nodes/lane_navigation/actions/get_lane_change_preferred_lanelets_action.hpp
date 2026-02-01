#ifndef BEHAVIOUR__NODES__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour
{
/**
 * @class GetLaneChangePreferredLaneletsAction
 * @brief BT sync action to select preferred lanelet IDs for a lane change.
 */
class GetLaneChangePreferredLaneletsAction : public BT::SyncActionNode
{
public:
  GetLaneChangePreferredLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>(
        "lane_ctx", "Current lane context"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>(
        "target_lanelet", "Target lanelet for lane change"),
      BT::OutputPort<std::vector<int64_t>>(
        "out_preferred_lanelet_ids", "Preferred lanelet IDs for behaviour execution"),
      BT::OutputPort<std::string>("error_message", "Error description if the node fails"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!lane_ctx) {
        setOutput("error_message", "missing_lane_context");
        std::cout << "[GetLaneChangePreferredLanelets]: Missing lane_ctx input" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    
    const auto target_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "target_lanelet");
    if (!target_lanelet) {
      setOutput("error_message", "missing_target_lanelet");
      std::cout << "[GetLaneChangePreferredLanelets]: Missing target_lanelet input" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::vector<int64_t> preferred;
    preferred.push_back(target_lanelet->id);
    preferred.push_back(lane_ctx->current_lanelet.id);
    for (const auto & id : target_lanelet->successor_ids) {
      preferred.push_back(id);
    }

    setOutput("out_preferred_lanelet_ids", preferred);
    std::cout << "[GetLaneChangePreferredLanelets]: Result=SUCCESS (preferred_ids_count="
              << preferred.size() << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_
