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

#ifndef CAMERA_DEBAYER__CAMERA_DEBAYER_CORE_HPP_
#define CAMERA_DEBAYER__CAMERA_DEBAYER_CORE_HPP_

#include <cuda_runtime.h>
#include <nppi_color_conversion.h>

namespace wato::perception::camera_debayer
{
    class CameraDebayerCore
    {
        public:
        // COnverts Bayer BGGR image to BGR
        void * debayer(
            const void * bayer_buffer, // gpu pointer to bayer image
            int width, int height, int step, // image dimensions, 
            cudaStream_t stream
        );
    };
}

#endif