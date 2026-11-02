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

#include "prediction_ml/fallback_prediction.hpp"

#include <cmath>

namespace prediction_ml
{

std::array<double, 2> trackerVelocity(const vision_msgs::msg::Detection3D & detection)
{
  for (const auto & result : detection.results) {
    const auto & v = result.pose.pose.position;
    if (
      result.hypothesis.class_id == "linear_velocity" && result.hypothesis.score > 0.0 && std::isfinite(v.x) &&
      std::isfinite(v.y))
    {
      return {v.x, v.y};
    }
  }
  return {0.0, 0.0};
}

}  // namespace prediction_ml
