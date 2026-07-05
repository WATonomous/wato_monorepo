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

#ifndef PREDICTION_ML__SCENE_BUILDER_HPP_
#define PREDICTION_ML__SCENE_BUILDER_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "prediction_ml/mtr_types.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

namespace prediction_ml
{

// Maintains per-object history and packs MTR input tensors. (Person A)
class SceneBuilder
{
public:
  explicit SceneBuilder(MtrConfig config);

  // Append the latest tracked detections into per-object history.
  void addFrame(const vision_msgs::msg::Detection3DArray & detections);

  // Pack the current scene into MTR input tensors + sidecar.
  MtrInputTensors build(const MtrFrameContext & frame);

private:
  enum class MtrObjectClass
  {
    Vehicle,
    Pedestrian,
    Cyclist
  };

  struct HistorySample
  {
    std::string detection_id;
    double timestamp{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double dx{0.0};
    double dy{0.0};
    double dz{0.0};
    double heading{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    MtrObjectClass object_class{MtrObjectClass::Vehicle};
    std::string frame_id;
  };

  static std::optional<double> timestampFromHeader(const vision_msgs::msg::Detection3DArray & detections);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static std::optional<MtrObjectClass> parseObjectClass(const vision_msgs::msg::Detection3D & detection);
  static std::optional<HistorySample> sampleFromDetection(
    const vision_msgs::msg::Detection3D & detection, double timestamp, const std::string & frame_id);

  void addFrameAtTimestamp(const vision_msgs::msg::Detection3DArray & detections, double timestamp);
  void addSample(const HistorySample & sample);
  void pruneHistory(double current_time);
  std::vector<std::string> orderedTrackIds() const;
  bool hasRetainedHistory() const;

  MtrConfig config_;
  bool config_valid_{false};
  std::unordered_map<std::string, std::vector<HistorySample>> history_;
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__SCENE_BUILDER_HPP_
