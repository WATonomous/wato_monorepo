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
#ifndef LANE_ENGINE_
#define LANE_ENGINE_

/* for general */
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <memory>

/* for OpenCV */
#include <opencv2/opencv.hpp>

/* for My modules */
#include "inference_helper.h"
#include "bounding_box.h"


class LaneEngine {
public:
    enum {
        kRetOk = 0,
        kRetErr = -1,
    };

    template <typename T> 
    using Line = std::vector<std::pair<T, T>>;

    typedef struct Result_ {
        std::vector<Line<int32_t>> line_list;
        struct crop_ {
            int32_t x;
            int32_t y;
            int32_t w;
            int32_t h;
            crop_() : x(0), y(0), w(0), h(0) {}
        } crop;
        double                   time_pre_process;		// [msec]
        double                   time_inference;		// [msec]
        double                   time_post_process;	    // [msec]
        Result_() : time_pre_process(0), time_inference(0), time_post_process(0)
        {}
    } Result;

public:
    LaneEngine() {}
    ~LaneEngine() {}
    int32_t Initialize(const std::string& work_dir, const int32_t num_threads);
    int32_t Finalize(void);
    int32_t Process(const cv::Mat& original_mat, Result& result);

    void GenerateAnchor();
    std::vector<Line<float>> Pred2Coords(const std::vector<float>& loc_row, const std::vector<int32_t>& loc_row_dims, const std::vector<float>& exist_row, const std::vector<int32_t>& exist_row_dims, 
        const std::vector<float>& loc_col, const std::vector<int32_t>& loc_col_dims, const std::vector<float>& exist_col, const std::vector<int32_t>& exist_col_dims);

private:

private:
    std::unique_ptr<InferenceHelper> inference_helper_;
    std::vector<InputTensorInfo> input_tensor_info_list_;
    std::vector<OutputTensorInfo> output_tensor_info_list_;

    std::vector<float> row_anchor_;
    std::vector<float> col_anchor_;

};

#endif
