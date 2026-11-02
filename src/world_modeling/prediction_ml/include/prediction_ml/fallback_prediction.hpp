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

// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PREDICTION_ML__FALLBACK_PREDICTION_HPP_
#define PREDICTION_ML__FALLBACK_PREDICTION_HPP_

#include <array>

#include <vision_msgs/msg/detection3_d.hpp>

namespace prediction_ml
{

// (vx, vy) of the tracker's linear_velocity result (m/s, in the detection's frame), or (0, 0) when it
// is missing, unscored or non-finite, so an unmeasured track gets a zero-length fallback path.
std::array<double, 2> trackerVelocity(const vision_msgs::msg::Detection3D & detection);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__FALLBACK_PREDICTION_HPP_
