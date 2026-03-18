// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_SEARCH_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_SEARCH_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"

namespace behaviour
{
/**
 * @class GetIntersectionSearchLaneletsAction
 * @brief Produces an ordered list of lanelets to scan for traffic-control context.
 *
 * Output ordering contract (downstream consumers depend on this):
 *   search_lanelets[0]    = ego's current lanelet
 *   search_lanelets[1..N] = subsequent lanelets, closest-to-furthest
 *
 * Source priority:
 *   1. Route-ahead (if active): lanelets from the planned route, distance-gated.
 *   2. Lanelets-ahead (fallback): BFS forward via successor links, sorted by distance.
 */
class GetIntersectionSearchLaneletsAction : public BT::SyncActionNode
{
public:
  GetIntersectionSearchLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<lanelet_msgs::msg::RouteAhead::SharedPtr>("route_ahead"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("route_ahead_index_map"),
      BT::InputPort<lanelet_msgs::msg::LaneletAhead::SharedPtr>("lanelets_ahead"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("lanelets_ahead_index_map"),
      BT::InputPort<double>("lookahead_threshold_m"),
      BT::OutputPort<std::vector<lanelet_msgs::msg::Lanelet>>("search_lanelets"),
      BT::OutputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("search_lanelet_index_map"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing = [](const char * port) {
      std::cout << "[GetIntersectionSearchLanelets] Missing " << port << std::endl;
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing)) return BT::NodeStatus::FAILURE;

    auto lookahead_opt = ports::tryGet<double>(*this, "lookahead_threshold_m");
    if (!ports::require(lookahead_opt, "lookahead_threshold_m", missing)) return BT::NodeStatus::FAILURE;
    const double lookahead_m = *lookahead_opt;

    std::vector<lanelet_msgs::msg::Lanelet> search_lanelets;
    auto index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();

    // Current lanelet is always first — downstream assumes search_lanelets[0] is ego's lanelet.
    append_unique(lane_ctx->current_lanelet, search_lanelets, *index_map);

    auto route_ahead = ports::tryGetPtr<lanelet_msgs::msg::RouteAhead>(*this, "route_ahead");
    bool used_route = false;

    // primary: route-ahead (already ordered by route progression)
    if (route_ahead && route_ahead->has_active_route && !route_ahead->lanelets.empty()) {
      used_route = true;

      auto ra_idx =
        ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "route_ahead_index_map");
      if (!ra_idx) ra_idx = build_index_map(route_ahead->lanelets);

      // upcoming_lanelet_ids is distance-sorted; collect those within lookahead
      const std::size_t n = std::min(
        lane_ctx->upcoming_lanelet_ids.size(), lane_ctx->upcoming_lanelet_distances_m.size());
      for (std::size_t i = 0; i < n; ++i) {
        if (lane_ctx->upcoming_lanelet_distances_m[i] < 0.0) continue;
        if (lane_ctx->upcoming_lanelet_distances_m[i] > lookahead_m) break;

        const auto it = ra_idx->find(lane_ctx->upcoming_lanelet_ids[i]);
        if (it != ra_idx->end() && it->second < route_ahead->lanelets.size()) {
          append_unique(route_ahead->lanelets[it->second], search_lanelets, *index_map);
        }
      }

      // Startup fallback: upcoming_* may be empty during initialization
      if (search_lanelets.size() <= 1) {
        double acc_m = 0.0;
        for (const auto & ll : route_ahead->lanelets) {
          if (acc_m > lookahead_m) break;
          append_unique(ll, search_lanelets, *index_map);
          acc_m += geometry::polylineLengthXY(ll.centerline);
        }
      }
    }

    // fallback: bfs through lanelets-ahead via successor links
    if (!used_route) {
      auto la = ports::tryGetPtr<lanelet_msgs::msg::LaneletAhead>(*this, "lanelets_ahead");
      if (la && !la->lanelets.empty()) {
        auto la_idx =
          ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "lanelets_ahead_index_map");
        if (!la_idx) la_idx = build_index_map(la->lanelets);

        // resolve bfs start: prefer ego's lanelet, fall back to message-reported id
        int64_t start_id = lane_ctx->current_lanelet.id;
        if (!la_idx->count(start_id)) {
          start_id = (la->current_lanelet_id >= 0 && la_idx->count(la->current_lanelet_id))
                       ? la->current_lanelet_id
                       : la->lanelets.front().id;
        }

        // collect (distance, source-index) pairs via bfs
        std::vector<std::pair<double, std::size_t>> candidates;
        std::queue<std::pair<int64_t, double>> frontier;
        std::unordered_set<int64_t> visited;
        frontier.emplace(start_id, 0.0);
        visited.insert(start_id);

        while (!frontier.empty()) {
          const auto [lid, dist] = frontier.front();
          frontier.pop();
          if (dist > lookahead_m) continue;

          const auto it = la_idx->find(lid);
          if (it == la_idx->end() || it->second >= la->lanelets.size()) continue;

          candidates.emplace_back(dist, it->second);

          const auto & ll = la->lanelets[it->second];
          const double next_dist = dist + geometry::polylineLengthXY(ll.centerline);
          for (const auto sid : ll.successor_ids) {
            if (la_idx->count(sid) && visited.insert(sid).second) {
              frontier.emplace(sid, next_dist);
            }
          }
        }

        // Sort by distance to enforce closest-to-furthest ordering
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto & a, const auto & b) { return a.first < b.first; });

        for (const auto & [d, idx] : candidates) {
          append_unique(la->lanelets[idx], search_lanelets, *index_map);
        }
      }
    }

    setOutput("search_lanelets", search_lanelets);
    setOutput("search_lanelet_index_map", index_map);

    std::cout << "[GetIntersectionSearchLanelets] source="
              << (used_route ? "route" : "lanelets_ahead")
              << " count=" << search_lanelets.size() << std::endl;

    return BT::NodeStatus::SUCCESS;
  }

private:
  static std::shared_ptr<std::unordered_map<int64_t, std::size_t>> build_index_map(
    const std::vector<lanelet_msgs::msg::Lanelet> & lanelets)
  {
    auto index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();
    index_map->reserve(lanelets.size());
    for (std::size_t i = 0; i < lanelets.size(); ++i) {
      (*index_map)[lanelets[i].id] = i;
    }
    return index_map;
  }

  static void append_unique(
    const lanelet_msgs::msg::Lanelet & lanelet,
    std::vector<lanelet_msgs::msg::Lanelet> & search_lanelets,
    std::unordered_map<int64_t, std::size_t> & index_map)
  {
    if (index_map.find(lanelet.id) != index_map.end()) {
      return;
    }

    index_map[lanelet.id] = search_lanelets.size();
    search_lanelets.push_back(lanelet);
  }

};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_SEARCH_LANELETS_ACTION_HPP_
