// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <semvecnet/polyline-postprocessing.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lanelets/lanelets_core.hpp"

namespace wato::perception::lanelets
{

visualization_msgs::msg::MarkerArray makePolylineMarkers(
  const std::vector<semvecnet::postprocessing::Polyline> & polylines,
  const std_msgs::msg::Header & map_header,
  const Matrix4 & ego_to_map);

cv::Mat renderSemVecNetPreview(
  const cv::Mat & semantic_rgb,
  const std::vector<semvecnet::postprocessing::Polyline> & polylines);

}  // namespace wato::perception::lanelets
