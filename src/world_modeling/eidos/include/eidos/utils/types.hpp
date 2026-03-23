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

#include <pcl/kdtree/kdtree_flann.h>

#include <optional>
#include <string>
#include <vector>
#define PCL_NO_PRECOMPILE
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace eidos
{

/// Primary point type used for keyframe 3D positions and KD-tree queries.
using PointType = pcl::PointXYZI;

/// 6-DOF pose point type for keyframe storage (position + roll/pitch/yaw + time).
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PoseType = PointXYZIRPYT;

/**
 * @brief Eidos node lifecycle states.
 *
 * INITIALIZING: node created but not yet configured.
 * WARMING_UP:   waiting for all plugins to report ready (e.g. IMU stationarity).
 * RELOCALIZING: prior map loaded, searching for initial pose via relocalization plugins.
 * TRACKING:     actively running SLAM or localization.
 */
enum class SlamState
{
  INITIALIZING,
  WARMING_UP,
  RELOCALIZING,
  TRACKING
};

/**
 * @brief Return type for FactorPlugin::produceFactor().
 *
 * If timestamp has a value, EidosNode creates a new state at that timestamp.
 * If nullopt, the plugin contributes factors to an existing state (latching).
 */
struct StampedFactorResult
{
  /// Sensor timestamp. If set, EidosNode creates a new ISAM2 state at this time.
  /// If nullopt, factors are latched onto an existing state.
  std::optional<double> timestamp;

  /// GTSAM factors to add to the graph.
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  /// Initial values for any new variables introduced.
  gtsam::Values values;
};

}  // namespace eidos

// PCL registration (must be in global scope)
POINT_CLOUD_REGISTER_POINT_STRUCT(
  eidos::PointXYZIRPYT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(
    float, yaw, yaw)(double, time, time))
