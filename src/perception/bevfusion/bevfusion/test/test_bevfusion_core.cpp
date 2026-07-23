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

#include <catch2/catch_all.hpp>

#include "bevfusion/bevfusion_core.hpp"

using wato::perception::bevfusion::BEVFusionCore;
using wato::perception::bevfusion::BEVFusionInputConfig;

static BEVFusionInputConfig make_test_config()
{
  BEVFusionInputConfig config;
  config.camera_backbone_plan = "/dev/null/camera.backbone.plan";
  config.camera_vtransform_plan = "/dev/null/camera.vtransform.plan";
  config.fuser_plan = "/dev/null/fuser.plan";
  config.head_bbox_plan = "/dev/null/head.bbox.plan";
  config.lidar_backbone_onnx = "/dev/null/lidar.backbone.onnx";
  return config;
}

// =============================================================================
// TEST 1: Guard flag before initialize()
// WHY: BEVFusionCore must start in the uninitialized state. Calling infer() or
//      updateCalibration() before a successful initialize() is undefined behavior
//      (pipeline_ is null, stream_ is null). This verifies the guard is correctly
//      set at construction so callers can check isInitialized() before use.
// =============================================================================
TEST_CASE("BEVFusionCore: isInitialized is false after construction", "[core][fast]")
{
  BEVFusionCore core(make_test_config());
  REQUIRE_FALSE(core.isInitialized());
}

// =============================================================================
// TEST 2: infer() guard before initialize()
// WHY: infer() requires pipeline_ and stream_ to be live. Calling it before
//      initialize() would dereference a null pipeline_ — undefined behavior.
//      This verifies the guard returns an empty vector safely, no GPU needed.
// =============================================================================
TEST_CASE("BEVFusionCore: infer returns empty vector when not initialized", "[core][fast]")
{
  BEVFusionCore core(make_test_config());
  REQUIRE(core.infer({}, {}, 0).empty());
}

// =============================================================================
// TEST 3: updateCalibration() guard before initialize()
// WHY: updateCalibration() calls pipeline_->update(), which requires pipeline_
//      to be live. Calling it before initialize() would dereference a null
//      pipeline_. This verifies the guard silently no-ops instead of crashing.
// =============================================================================
TEST_CASE("BEVFusionCore: updateCalibration does not crash when not initialized", "[core][fast]")
{
  BEVFusionCore core(make_test_config());
  REQUIRE_NOTHROW(core.updateCalibration({}, {}, {}, {}));
}
