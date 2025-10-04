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

#include <Eigen/Core>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchwork/patchworkpp.h"

namespace wato::percpetion::patchworkpp
{

class GroundRemovalCore
{
public:
  explicit GroundRemovalCore(const patchwork::Params & params);

  void process(const Eigen::MatrixX3f & cloud);
  Eigen::MatrixX3f getGround() const;
  Eigen::MatrixX3f getNonground() const;
  double getTimeTaken() const;

  static Eigen::MatrixX3f pointCloud2ToEigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  static sensor_msgs::msg::PointCloud2 eigenToPointCloud2(
    const Eigen::MatrixX3f & points, const std_msgs::msg::Header & header);

private:
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
};

}  // namespace wato::percpetion::patchworkpp
