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

#ifndef BEHAVIOUR__IS_EGO_STATE_CONDITION_HPP_
#define BEHAVIOUR__IS_EGO_STATE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

namespace behaviour
{
    // when we need to know what current state of the vehicle is
    enum class LatticePlannerBehaviour
    {
        FOLLOW_EGO_LANE = "follow_ego_lane",
        LANE_CHANGE_LEFT = "lane_change_left",
        LANE_CHANGE_RIGHT = "lane_change_right",
        STANDBY = "standby"
    };

    // This is the "magic" function BT.CPP looks for
    inline LatticePlannerBehaviour convertFromString(BT::StringView str)
    {
        if (str == "FOLLOW_EGO_LANE")
            return LatticePlannerBehaviour::FOLLOW_EGO_LANE;
        if (str == "LANE_CHANGE_LEFT")
            return LatticePlannerBehaviour::LANE_CHANGE_LEFT;
        if (str == "LANE_CHANGE_RIGHT")
            return LatticePlannerBehaviour::LANE_CHANGE_RIGHT;
        if (str == "STANDBY")
            return LatticePlannerBehaviour::STANDBY;
        throw BT::RuntimeError("Invalid Behaviour: ", str.data());
    }
} // namespace behaviour

#endif
