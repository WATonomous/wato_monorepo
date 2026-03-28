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
 * Transitions (managed by InitSequencer, see init_sequencer.cpp):
 *   INITIALIZING -> WARMING_UP:  on_activate() calls InitSequencer::reset()
 *   WARMING_UP -> RELOCALIZING:  all plugins isReady() AND prior map loaded
 *   WARMING_UP -> TRACKING:      all plugins isReady() AND no prior map
 *   RELOCALIZING -> TRACKING:    relocalization succeeds OR timeout expires
 *
 * Gating logic: InitSequencer::handleWarmingUp() polls FactorPlugin::isReady()
 * on every factor plugin. LisoFactor gates on IMU stationary detection +
 * gravity alignment. ImuFactor gates on its own warmup if loaded.
 *
 * Values match eidos_msgs/msg/SlamStatus enum constants.
 */
enum class SlamState
{
  INITIALIZING = 0,  ///< Node created, not yet configured/activated
  WARMING_UP = 1,  ///< Waiting for all factor plugins to report isReady()
  RELOCALIZING = 2,  ///< Prior map loaded, polling relocalization plugins
  TRACKING = 3  ///< SLAM/localization active, graph optimizer running
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

  /// Whether this result contains a loop closure factor. When true, the
  /// optimizer runs extra iterations for convergence.
  bool loop_closure = false;

  /// Whether this result contains a correction factor (e.g. GPS unary).
  /// When true (and loop_closure is false), extra correction iterations run.
  bool correction = false;
};

}  // namespace eidos

// PCL registration (must be in global scope)
POINT_CLOUD_REGISTER_POINT_STRUCT(
  eidos::PointXYZIRPYT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(
    float, yaw, yaw)(double, time, time))
