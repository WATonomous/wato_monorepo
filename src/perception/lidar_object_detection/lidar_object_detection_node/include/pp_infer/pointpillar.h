/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POINTPILLAR_H_
#define POINTPILLAR_H_

#include <memory>
#include "cuda_runtime.h"
#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "NvInferRuntime.h"
#include "postprocess.h"

#define PERFORMANCE_LOG 1

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"


// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char* msg) noexcept override {
        // suppress info-level message
        //if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR || severity == Severity::kINFO ) {
        if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
            std::cerr << "trt_infer: " << msg << std::endl;
        }
    }
};

class TRT {
  private:
    cudaEvent_t start, stop;

    float elapsedTime = 0.0f;
    Logger gLogger_;
    nvinfer1::IExecutionContext *context = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;

    cudaStream_t stream_;
  public:
    TRT(
      std::string modelFile,
      std::string engineFile,
      cudaStream_t stream,
      const std::string& data_type
    );
    ~TRT(void);

    int doinfer(void**buffers, bool do_profile);
    nvinfer1::Dims get_binding_shape(int index);
    int getPointSize();
};

class PointPillar {
  private:
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream_;
    //output of TRT
    std::shared_ptr<TRT> trt_;
    //output of TRT
    float *box_output = nullptr;
    int *box_num = nullptr;
    unsigned int box_size;
    std::vector<Bndbox> res;

  public:
    PointPillar(
      std::string modelFile,
      std::string engineFile,
      cudaStream_t stream,
      const std::string& data_type
    );
    ~PointPillar(void);
    int getPointSize();
    std::vector<Bndbox> doinfer(
      void*points_data,
      unsigned int* points_size,
      std::vector<Bndbox> &nms_pred,
      float nms_iou_thresh,
      int pre_nms_top_n,
      std::vector<std::string>& class_names,
      bool do_profile
    );
};

#endif
