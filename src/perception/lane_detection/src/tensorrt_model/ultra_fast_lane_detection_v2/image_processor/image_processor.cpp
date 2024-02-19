/* Copyright 2022 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
/* for general */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

/* for OpenCV */
#include <opencv2/opencv.hpp>

/* for My modules */
#include "bounding_box.h"
#include "common_helper.h"
#include "common_helper_cv.h"
#include "image_processor.h"
#include "lane_engine.h"
#include "tracker.h"

/*** Macro ***/
#define TAG "ImageProcessor"
#define PRINT(...) COMMON_HELPER_PRINT(TAG, __VA_ARGS__)
#define PRINT_E(...) COMMON_HELPER_PRINT_E(TAG, __VA_ARGS__)

/*** Global variable ***/
std::unique_ptr<LaneEngine> s_engine;
CommonHelper::NiceColorGenerator s_nice_color_generator(4);

/*** Function ***/
static void DrawFps(cv::Mat &mat, double time_inference, cv::Point pos, double font_scale,
                    int32_t thickness, cv::Scalar color_front, cv::Scalar color_back,
                    bool is_text_on_rect = true) {
  char text[64];
  static auto time_previous = std::chrono::steady_clock::now();
  auto time_now = std::chrono::steady_clock::now();
  double fps = 1e9 / (time_now - time_previous).count();
  time_previous = time_now;
  snprintf(text, sizeof(text), "FPS: %.1f, Inference: %.1f [ms]", fps, time_inference);
  CommonHelper::DrawText(mat, text, cv::Point(0, 0), 0.5, 2, CommonHelper::CreateCvColor(0, 0, 0),
                         CommonHelper::CreateCvColor(180, 180, 180), true);
}

int32_t ImageProcessor::Initialize(const ImageProcessor::InputParam &input_param) {
  if (s_engine) {
    PRINT_E("Already initialized\n");
    return -1;
  }

  s_engine.reset(new LaneEngine());
  if (s_engine->Initialize(input_param.work_dir, input_param.num_threads) != LaneEngine::kRetOk) {
    s_engine->Finalize();
    s_engine.reset();
    return -1;
  }
  return 0;
}

int32_t ImageProcessor::Finalize(void) {
  if (!s_engine) {
    PRINT_E("Not initialized\n");
    return -1;
  }

  if (s_engine->Finalize() != LaneEngine::kRetOk) {
    return -1;
  }

  return 0;
}

int32_t ImageProcessor::Command(int32_t cmd) {
  if (!s_engine) {
    PRINT_E("Not initialized\n");
    return -1;
  }

  switch (cmd) {
    case 0:
    default:
      PRINT_E("command(%d) is not supported\n", cmd);
      return -1;
  }
}

int32_t ImageProcessor::Process(cv::Mat &mat, ImageProcessor::Result &result,
                                std::vector<std::vector<float>> &lane_list) {
  if (!s_engine) {
    PRINT_E("Not initialized\n");
    return -1;
  }

  LaneEngine::Result engine_result;

  printf("  Process image\n");
  if (s_engine->Process(mat, engine_result) != LaneEngine::kRetOk) {
    return -1;
  }

  /* Display target area  */
  cv::rectangle(mat,
                cv::Rect(engine_result.crop.x, engine_result.crop.y, engine_result.crop.w,
                         engine_result.crop.h),
                CommonHelper::CreateCvColor(0, 0, 0), 2);

  /* Draw line */
  for (int32_t lane_index = 0; lane_index < engine_result.line_list.size(); lane_index++) {
    const auto &line = engine_result.line_list[lane_index];
    if (line.size() < 2) {
      continue;
    }
    for (const auto &p : line) {
      cv::circle(mat, cv::Point(p.first, p.second), 4,
                 s_nice_color_generator.Get((lane_index == 0 || lane_index == 3) ? 0 : 1), -1);
    }
    // for (size_t i = 0; i < line.size() - 1; i++)
    // {
    //     const auto &p1 = line[i];
    //     const auto &p2 = line[i + 1];
    //     cv::line(mat, cv::Point(p1.first, p1.second), cv::Point(p2.first, p2.second),
    //     s_nice_color_generator.Get((lane_index == 0 || lane_index == 3) ? 0 : 1), 4);
    // }
  }

  DrawFps(mat, engine_result.time_inference, cv::Point(0, 0), 0.5, 2,
          CommonHelper::CreateCvColor(0, 0, 0), CommonHelper::CreateCvColor(180, 180, 180), true);

  result.time_pre_process = engine_result.time_pre_process;
  result.time_inference = engine_result.time_inference;
  result.time_post_process = engine_result.time_post_process;

  return 0;
}
