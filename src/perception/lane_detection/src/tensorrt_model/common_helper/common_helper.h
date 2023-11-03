/* Copyright 2021 iwatake2222

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
#ifndef COMMON_HELPER_
#define COMMON_HELPER_

/* for general */
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <array>


#if defined(ANDROID) || defined(__ANDROID__)
#define CV_COLOR_IS_RGB
#include <android/log.h>
#define COMMON_HELPER_NDK_TAG "MyApp_NDK"
#define COMMON_HELPER_PRINT_(...) __android_log_print(ANDROID_LOG_INFO, COMMON_HELPER_NDK_TAG, __VA_ARGS__)
#else
#define COMMON_HELPER_PRINT_(...) printf(__VA_ARGS__)
#endif

#define COMMON_HELPER_PRINT(COMMON_HELPER__PRINT_TAG, ...) do { \
    COMMON_HELPER_PRINT_("[" COMMON_HELPER__PRINT_TAG "][%d] ", __LINE__); \
    COMMON_HELPER_PRINT_(__VA_ARGS__); \
} while(0);

#define COMMON_HELPER_PRINT_E(COMMON_HELPER__PRINT_TAG, ...) do { \
    COMMON_HELPER_PRINT_("[ERR: " COMMON_HELPER__PRINT_TAG "][%d] ", __LINE__); \
    COMMON_HELPER_PRINT_(__VA_ARGS__); \
} while(0);

namespace CommonHelper
{

float Sigmoid(float x);
float Logit(float x);
float SoftMaxFast(const float* src, float* dst, int32_t length);

}

#endif
