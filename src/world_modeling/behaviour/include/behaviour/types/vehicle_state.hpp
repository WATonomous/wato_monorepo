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
  enum class VehicleState
  {
    STOPPED = 0,
    DRIVING = 1,
    EMERGENCY = 2
  };

  // This is the "magic" function BT.CPP looks for
  inline VehicleState convertFromString(BT::StringView str)
  {
    if (str == "STOPPED")
      return VehicleState::STOPPED;
    if (str == "DRIVING")
      return VehicleState::DRIVING;
    if (str == "EMERGENCY")
      return VehicleState::EMERGENCY;
    throw BT::RuntimeError("Invalid VehicleState: ", str.data());
  }
} // namespace behaviour

#endif
