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

#include "track_eval/cost.hpp"

#include <algorithm>
#include <vector>

#include "track_eval/Accumulator.hpp"
#include "track_eval/lap.hpp"
#include "track_eval/Matches.hpp"

double costIoU(const vision_msgs::msg::BoundingBox2D & box_a, const vision_msgs::msg::BoundingBox2D & box_b)
{
  const double x1_a = box_a.center.position.x - box_a.size_x / 2, x1_b = box_b.center.position.x - box_b.size_x / 2;
  const double y1_a = box_a.center.position.y - box_a.size_y / 2, y1_b = box_b.center.position.y - box_b.size_y / 2;
  const double x2_a = box_a.center.position.x + box_a.size_x / 2, x2_b = box_b.center.position.x + box_b.size_x / 2;
  const double y2_a = box_a.center.position.y + box_a.size_y / 2, y2_b = box_b.center.position.y + box_b.size_y / 2;

  const double iw = std::min(x2_a, x2_b) - std::max(x1_a, x1_b);
  const double ih = std::min(y2_a, y2_b) - std::max(y1_a, y1_b);

  if (iw <= 0.0 || ih <= 0.0) return 1.0;

  const double intersection = iw * ih;

  const double area_a = box_a.size_x * box_a.size_y;
  const double area_b = box_b.size_x * box_b.size_y;
  const double un = area_a + area_b - intersection;

  if (un <= 0.0) return 1.0;

  return 1.0 - intersection / un;
}

std::vector<std::vector<double>> costMtx(
  const std::vector<vision_msgs::msg::Detection2D> & gts, const std::vector<vision_msgs::msg::Detection2D> & trks)
{
  int num_gts = gts.size(), num_trks = trks.size();
  std::vector<std::vector<double>> mtx(num_gts, std::vector<double>(num_trks, 0.0));

  for (int i = 0; i < num_gts; ++i) {
    for (int j = 0; j < num_trks; ++j) {
      mtx[i][j] = costIoU(gts[i].bbox, trks[j].bbox);
    }
  }

  return mtx;
}
