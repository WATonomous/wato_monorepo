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

#include "camera_debayer/camera_debayer_core.hpp"

namespace wato::perception::camera_debayer
{

void * CameraDebayerCore::debayer(
  const void * bayer_buffer,
  int width, int height, int step,
  cudaStream_t stream)
{
  // Allocate a new buffer for the output BGR image (3 bytes per pixel for RGB)
  void * bgr_buffer;
  cudaMallocAsync(&bgr_buffer, width * height * 3, stream);

  NppiSize size = {width, height};
  nppiCFAToRGB_8u_C1C3R(
    (const Npp8u *)bayer_buffer, step,      // input: Bayer (1 channel)
    (Npp8u *)bgr_buffer, width * 3,         // output: RGB (3 channels)
    size, NPPI_BAYER_BGGR, NPPI_INTER_LINEAR);

  // NPP outputs RGB, but we need BGR for the rectify node — swap channels in-place
  const int swap[] = {2, 1, 0};
  nppiSwapChannels_8u_C3IR((Npp8u *)bgr_buffer, width * 3, size, swap);

  return bgr_buffer;
}

}  // namespace wato::perception::camera_debayer