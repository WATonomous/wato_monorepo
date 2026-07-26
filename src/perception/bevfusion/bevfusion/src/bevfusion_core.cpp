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

#include "bevfusion/bevfusion_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace wato::perception::bevfusion
{

BEVFusionCore::BEVFusionCore(const BEVFusionInputConfig & config)
: config_(config)
{}

bool BEVFusionCore::initialize()
{
  /**
  * TODO(bevfusion_team)
  * Overview: Translate config struct into ::bevfusion::CoreParameter and calls
  *           ::bevfusion::create_core(param) to build and deserialize the entire CUDA-BEVFusion pipeline
  *           (allocating GPU memory, compiling/optimizing custom kernels, and loading the 5 engine plans
  *           into memory)
  *
  * Key steps:
  *   1. Load custom Layer Normalization shared object required for the detection head.
  *   2. Build parameter structs for normalization, voxelization, SCN, camera-to-BEV projection geometry, and 2D detection head.
  *   3. Create a CUDA stream for asynchronous execution.
  *   4. Deserialize all TensorRT engine plans (camera backbone, view transformer, fusion, head) into executable engines.
  *   5. Allocate GPU memory for intermediate buffers (camera images, lidar points, feature maps, BEV tensors).
  */
}

std::vector<BoundingBox> BEVFusionCore::infer(
  const std::vector<const unsigned char *> & camera_images, const std::vector<float> & lidar_points, int num_points)
{
  /**
 * TODO(bevfusion_team)
 * Overview: Convert data to required format and feed it to GPU pipeline (needs LiDAR data to be FP16)
 *
 * Key steps:
 *   1. Convert the input `lidar_points` (vector of `float`) into a vector of `nvtype::half`.
 *   2. Call `pipeline_->forward(images.data(), lidar_half.data(), num_points, stream_)`.
 *   3. Return the `std::vector<BoundingBox>` result.
 */
}

void BEVFusionCore::updateCalibration(
  const std::vector<float> & camera_to_lidar,
  const std::vector<float> & camera_intrinsics,
  const std::vector<float> & lidar_to_camera,
  const std::vector<float> & img_aug_matrix)
{
  /**
 * TODO(bevfusion_team)
 * Overview: Feed the current physical positions of the cameras (extrinsics) and lens properties (instrinsics) to
 *           the BEVFusion Core (pipeline_). This is done by calling pipeline_->update() with the appropriate
 *           parameters. Note: Extrinsic and instrinsics are static, so only run this once at startup or when calibration
 *           changes. MUST be called before the first forward() call.
 *
 * Call core_->update(camera2lidar.data(), camera_intrinsics.data(), lidar2image.data(), img_aug_matrix.data(), stream_)
 * and that I think that should be it.
 */
}

}  // namespace wato::perception::bevfusion
