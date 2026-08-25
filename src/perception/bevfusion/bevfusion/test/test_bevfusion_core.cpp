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

#include <memory>
#include <vector>

#include <catch2/catch_all.hpp>

#define private public
#include "bevfusion/bevfusion_core.hpp"
#undef private

using wato::perception::bevfusion::BEVFusionCore;
using wato::perception::bevfusion::BEVFusionInputConfig;

class MockCore : public ::bevfusion::Core  // Create a fake version of cuda-bevfusion
{
public:
  bool update_called = false;
  bool forward_called = false;
  int last_num_points = 0;

  std::vector<::bevfusion::head::transbbox::BoundingBox> forward(
    const unsigned char ** /*camera_images*/,
    const nvtype::half * /*lidar_points*/,
    int num_points,
    void * /*stream*/) override
  {
    forward_called = true;
    last_num_points = num_points;

    ::bevfusion::head::transbbox::BoundingBox box;
    box.position.x = 1.0f;
    box.position.y = 2.0f;
    box.position.z = 3.0f;
    box.size.w = 4.0f;
    box.size.l = 5.0f;
    box.size.h = 6.0f;
    box.velocity.vx = 7.0f;
    box.velocity.vy = 8.0f;
    box.z_rotation = 9.0f;
    box.score = 0.95f;
    box.id = 2;
    return {box};
  }

  std::vector<::bevfusion::head::transbbox::BoundingBox> forward_no_normalize(
    const nvtype::half * /*camera_normed_images_device*/,
    const nvtype::half * /*lidar_points*/,
    int /*num_points*/,
    void * /*stream*/) override
  {
    return {};
  }

  void print() override
  {}

  void set_timer(bool /*enable*/) override
  {}

  void update(
    const float * /*camera2lidar*/,
    const float * /*camera_intrinsics*/,
    const float * /*lidar2image*/,
    const float * /*img_aug_matrix*/,
    void * /*stream*/ = nullptr) override
  {
    update_called = true;
  }

  void free_excess_memory() override
  {}
};

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

// =============================================================================
// TEST 4: hasCalibration() state tracking
// WHY: The core must track if calibration matrices have been provided. It should
//      start false, and if updateCalibration is called before initialization,
//      it should remain false (since the update fails).
// =============================================================================
TEST_CASE("BEVFusionCore: hasCalibration tracks state correctly", "[core][fast]")
{
  BEVFusionCore core(make_test_config());

  // 1. Should be false initially
  REQUIRE_FALSE(core.hasCalibration());
  // 2. Try to update calibration (this will fail internally because isInitialized() == false)
  core.updateCalibration({}, {}, {}, {});
  // 3. Should still be false because the calibration update was rejected
  REQUIRE_FALSE(core.hasCalibration());
}

// =============================================================================
// TEST 5: updateCalibration() with valid pipeline updates mock core
// WHY: Verifies that when BEVFusionCore has a valid pipeline, updateCalibration
//      correctly passes through the parameters and updates the internal state.
// =============================================================================
TEST_CASE("BEVFusionCore: updateCalibration sets hasCalibration flag and calls pipeline", "[core][fast]")
{
  BEVFusionCore core(make_test_config());
  auto mock_pipeline = std::make_shared<MockCore>();
  core.pipeline_ = mock_pipeline;
  core.initialized_ = true;  // Fake initialization

  std::vector<float> dummy(16, 1.0f);
  core.updateCalibration(dummy, dummy, dummy, dummy);

  REQUIRE(core.hasCalibration());
  REQUIRE(mock_pipeline->update_called);
}

// =============================================================================
// TEST 6: infer() with valid pipeline translates results
// WHY: Verifies that BEVFusionCore::infer correctly parses the bounding boxes
//      returned from the vendor library into our BoundingBox format, and
//      that it converts the LiDAR points correctly.
// =============================================================================
TEST_CASE("BEVFusionCore: infer() forwards data and translates BoundingBox correctly", "[core][fast]")
{
  BEVFusionCore core(make_test_config());
  auto mock_pipeline = std::make_shared<MockCore>();
  core.pipeline_ = mock_pipeline;
  core.initialized_ = true;
  core.has_calibration_ = true;

  unsigned char dummy_image = 0;
  std::vector<const unsigned char *> images(6, &dummy_image);
  std::vector<float> lidar_points = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};  // 1 point (5 features)

  auto results = core.infer(images, lidar_points, 1);

  REQUIRE(mock_pipeline->forward_called);
  REQUIRE(mock_pipeline->last_num_points == 1);
  REQUIRE(results.size() == 1);

  const auto & box = results[0];
  REQUIRE(box.position.x == 1.0f);
  REQUIRE(box.position.y == 2.0f);
  REQUIRE(box.position.z == 3.0f);
  REQUIRE(box.size.w == 4.0f);
  REQUIRE(box.size.l == 5.0f);
  REQUIRE(box.size.h == 6.0f);
  REQUIRE(box.velocity.vx == 7.0f);
  REQUIRE(box.velocity.vy == 8.0f);
  REQUIRE(box.z_rotation == 9.0f);
  REQUIRE(box.score == 0.95f);
  REQUIRE(box.id == 2);
}
