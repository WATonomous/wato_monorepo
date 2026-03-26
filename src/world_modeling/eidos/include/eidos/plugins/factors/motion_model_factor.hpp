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

#include <string>
#include <vector>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos_msgs/srv/predict_relative_transform.hpp"

namespace eidos
{

/**
 * @brief Motion model factor plugin.
 *
 * Bridges temporally consecutive states created by different factor plugins.
 * Calls eidos_transform's PredictRelativeTransform service to get the predicted
 * relative pose between timestamps, then produces a BetweenFactor<Pose3>.
 *
 * Architecturally similar to loop closure — latches onto existing states with
 * a between-factor — but uses temporal prediction instead of spatial GICP matching.
 */
class MotionModelFactor : public FactorPlugin
{
public:
  MotionModelFactor() = default;
  ~MotionModelFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  rclcpp::Client<eidos_msgs::srv::PredictRelativeTransform>::SharedPtr predict_client_;

  gtsam::Key last_key_{0};
  double last_timestamp_{0.0};
  std::string last_owner_;
  bool has_last_{false};
  bool active_{false};

  std::vector<double> noise_cov_;
};

}  // namespace eidos
