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

#pragma once

#include <gtsam/base/Vector.h>

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos
{

/**
 * @brief Holonomic constant-velocity state transition model.
 *
 * Maintains a velocity estimate (6-DOF: vx, vy, vz, wx, wy, wz in body frame)
 * updated from consecutive measurement-corrected poses. Prediction propagates
 * the current pose forward at the last known velocity.
 *
 * Between measurement corrections (~20Hz from LISO), the model coasts at
 * constant velocity for smooth 500Hz TF output.
 */
class HolonomicMotionModel : public MotionModelPlugin
{
public:
  HolonomicMotionModel() = default;
  ~HolonomicMotionModel() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  gtsam::Pose3 predict(const gtsam::Pose3 & current, double dt) override;
  void onMeasurementUpdate(const gtsam::Pose3 & corrected, double timestamp) override;

private:
  gtsam::Vector6 velocity_ = gtsam::Vector6::Zero();
  gtsam::Pose3 last_corrected_;
  double last_corrected_time_ = 0.0;
  bool has_last_corrected_ = false;
};

}  // namespace eidos
