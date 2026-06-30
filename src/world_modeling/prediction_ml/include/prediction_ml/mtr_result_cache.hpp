// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.

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
