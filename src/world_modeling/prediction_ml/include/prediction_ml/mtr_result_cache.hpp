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

#ifndef PREDICTION_ML__MTR_RESULT_CACHE_HPP_
#define PREDICTION_ML__MTR_RESULT_CACHE_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "deep_msgs/msg/mtr_prediction_array.hpp"
#include "deep_msgs/msg/mtr_scene.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace prediction_ml
{
class MtrResultCache
{
public:
  explicit MtrResultCache(double ttl_s);
  void rememberRequest(const deep_msgs::msg::MtrScene & scene);
  bool accept(const deep_msgs::msg::MtrPredictionArray & result, double now_s);
  std::vector<world_model_msgs::msg::WorldObject> select(
    const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s) const;

private:
  struct Pending
  {
    double source_s;
    std::string frame_id;
    std::unordered_set<std::string> track_ids;
  };

  struct Cached
  {
    double source_s;
    std::vector<world_model_msgs::msg::Prediction> predictions;
  };

  double ttl_s_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, Pending> pending_;
  std::unordered_map<std::string, Cached> cached_;
};
}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_RESULT_CACHE_HPP_
