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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/ports.hpp"

#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
    /**
     * @class GetStopSignCars
     * @brief Gets cars "queued" near stop lines for the yield lanelets of a stop-sign/right_of_way reg elem.
     *
     * Assumptions:
     * - stop_sign->yield_lanelet_ids is populated (right_of_way subtype).
     * - dynamic objects provide lanelet_ahead.current_lanelet_id and detection.bbox.center.position.
     * - lanelets input includes the yield lanelets controlled by this reg elem (from GetLaneletsByRegElem).
     *
     * Distance heuristic:
     * - Uses centroid of the stop line polyline and Euclidean distance in XY.
     */
    class GetStopSignCarsAction : public BT::SyncActionNode
    {
    public:
        GetStopSignCarsAction(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("stop_sign"),
                BT::InputPort<std::vector<lanelet_msgs::msg::Lanelet>>("lanelets"),
                BT::InputPort<double>("threshold_m"),
                BT::InputPort<std::shared_ptr<const DynamicObjectStore::Snapshot>>("dynamic_objects_snapshot"),
                BT::OutputPort<std::vector<std::string>>("out_stop_sign_car_ids"),
            };
        }

        BT::NodeStatus tick() override
        {
            auto stop_sign = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "stop_sign");
            auto snap = ports::tryGetPtr<const DynamicObjectStore::Snapshot>(*this, "dynamic_objects_snapshot");
            auto lanelets = ports::tryGet<std::vector<lanelet_msgs::msg::Lanelet>>(*this, "lanelets");
            auto threshold_m = ports::tryGet<double>(*this, "threshold_m");

            if (!stop_sign)
            {
                std::cout << "[GetStopSignCars] Missing stop_sign" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            if (!snap || !snap->objects_snapshot_)
            {
                std::cout << "[GetStopSignCars] Missing dynamic_objects_snapshot" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            if (!lanelets)
            {
                std::cout << "[GetStopSignCars] Missing lanelets" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            if (!threshold_m)
            {
                std::cout << "[GetStopSignCars] Missing threshold_m" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            if (stop_sign->yield_lanelet_ids.empty())
            {
                std::cout << "[GetStopSignCars] yield_lanelet_ids empty" << std::endl;
                setOutput("out_stop_sign_car_ids", std::vector<std::string>{});
                return BT::NodeStatus::SUCCESS;
            }

            std::unordered_set<int64_t> yield_ids;
            yield_ids.reserve(stop_sign->yield_lanelet_ids.size());
            for (const auto id : stop_sign->yield_lanelet_ids)
            {
                yield_ids.insert(id);
            }

            std::vector<std::string> out_ids;
            out_ids.reserve(32);

            for (const auto &ll : *lanelets)
            {
                if (yield_ids.find(ll.id) == yield_ids.end())
                {
                    continue;
                }

                // Find this reg elem within this lanelet so we can use its ref_lines (stop lines).
                const lanelet_msgs::msg::RegulatoryElement *re_in_ll = nullptr;
                for (const auto &re : ll.regulatory_elements)
                {
                    if (re.id == stop_sign->id)
                    {
                        re_in_ll = &re;
                        break;
                    }
                }
                if (!re_in_ll)
                {
                    continue;
                }

                // Pick the first ref_line with points as the stop line.
                const lanelet_msgs::msg::RefLine *stop_line = nullptr;
                for (const auto &rl : re_in_ll->ref_lines)
                {
                    if (!rl.points.empty())
                    {
                        stop_line = &rl;
                        break;
                    }
                }
                if (!stop_line)
                {
                    continue;
                }

                // Compute centroid of the stop line polyline in XY.
                double cx = 0.0, cy = 0.0;
                for (const auto &p : stop_line->points)
                {
                    cx += p.x;
                    cy += p.y;
                }
                cx /= static_cast<double>(stop_line->points.size());
                cy /= static_cast<double>(stop_line->points.size());

                // Get cars currently in this lanelet and filter by distance to stop line centroid.
                const auto cars = snap->getCarsInLanelet(ll.id);
                for (const auto *car : cars)
                {
                    if (!car)
                        continue;

                    const auto &pos = car->detection.bbox.center.position;

                    const double dx = pos.x - cx;
                    const double dy = pos.y - cy;
                    const double d = std::sqrt(dx * dx + dy * dy);

                    if (d <= *threshold_m)
                    {
                        out_ids.push_back(car->detection.id);
                    }
                }
            }

            // Deduplicate IDs (defensive; avoids duplicates if any mapping overlap occurs).
            std::sort(out_ids.begin(), out_ids.end());
            out_ids.erase(std::unique(out_ids.begin(), out_ids.end()), out_ids.end());

            setOutput("out_stop_sign_car_ids", out_ids);
            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_
