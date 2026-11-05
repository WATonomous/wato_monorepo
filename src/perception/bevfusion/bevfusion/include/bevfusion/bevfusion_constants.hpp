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

namespace wato::perception::bevfusion
{

/**
 * @brief A structure to hold RGBA values used for drawing bounding boxes
 */
struct BoxColor
{
  float r = 1.0f;
  float g = 1.0f;
  float b = 1.0f;
  float a = 0.8f;
};

/**
 * @brief Retrieves the standard bounding box RGBA color for a given class ID.
 *        Follows nuScenes dataset color convention.
 *
 * @param class_id The ID of the class for which to get the color.
 * @return BoxColor The RGBA color for the given class ID.
 */
inline BoxColor getBoxColor(int class_id)
{
  switch (class_id) {
    case 0:
      return {0.0f, 1.0f, 0.0f, 0.8f};  // Car — green
    case 1:
      return {0.0f, 0.0f, 1.0f, 0.8f};  // Truck — blue
    case 2:
      return {1.0f, 0.5f, 0.0f, 0.8f};  // Construction vehicle — orange
    case 3:
      return {0.5f, 0.0f, 1.0f, 0.8f};  // Bus — purple
    case 4:
      return {0.0f, 1.0f, 1.0f, 0.8f};  // Trailer — cyan
    case 5:
      return {1.0f, 1.0f, 0.0f, 0.8f};  // Barrier — yellow
    case 6:
      return {1.0f, 0.0f, 1.0f, 0.8f};  // Motorcycle — magenta
    case 7:
      return {0.0f, 0.5f, 1.0f, 0.8f};  // Bicycle — sky blue
    case 8:
      return {1.0f, 0.0f, 0.0f, 0.8f};  // Pedestrian — red
    case 9:
      return {1.0f, 0.8f, 0.0f, 0.8f};  // Traffic cone — amber
    default:
      return {1.0f, 1.0f, 1.0f, 0.8f};  // Default white
  }
}

}  // namespace wato::perception::bevfusion
