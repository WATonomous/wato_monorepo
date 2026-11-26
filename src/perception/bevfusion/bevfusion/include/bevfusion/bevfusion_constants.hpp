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
 * @brief A structure to hold RGBA colour values.
 */
struct Color
{
  float r = 1.0f;
  float g = 1.0f;
  float b = 1.0f;
  float a = 0.8f;
};

/**
 * @brief Retrieves the standard bounding box RGBA colour for a given class ID.
 *        Follows nuScenes dataset colour convention.
 *
 * @param class_id The ID of the class for which to get the colour.
 * @return Color The RGBA colour for the given class ID.
 */
inline Color getBoxColor(int class_id)
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

/**
 * @brief Retrieves the RGBA colour for one of the six nuScenes map classes.
 *
 * @param class_id Segmentation class ID (see seg_class_names in params.yaml).
 * @return Color The RGBA colour for the given map class.
 */
inline Color getClassColor(int class_id)
{
  switch (class_id) {
    case 0:
      return {0.0f, 0.4f, 1.0f, 0.8f};  // Drivable area — blue
    case 1:
      return {1.0f, 0.5f, 0.0f, 0.8f};  // Ped crossing — orange
    case 2:
      return {0.0f, 1.0f, 0.0f, 0.8f};  // Walkway — green
    case 3:
      return {1.0f, 0.0f, 0.0f, 0.8f};  // Stop line — red
    case 4:
      return {0.6f, 0.0f, 1.0f, 0.8f};  // Carpark area — purple
    case 5:
      return {1.0f, 1.0f, 0.0f, 0.8f};  // Divider — yellow
    default:
      return {1.0f, 1.0f, 1.0f, 0.8f};  // Unknown — white
  }
}

}  // namespace wato::perception::bevfusion
