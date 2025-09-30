#pragma once

#include <memory>

#include <Eigen/Core>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchwork/patchworkpp.h"

namespace patchworkpp_ros {

class GroundRemovalCore {
 public:
  explicit GroundRemovalCore(const patchwork::Params &params);

  void process(const Eigen::MatrixX3f &cloud);
  Eigen::MatrixX3f getGround() const;
  Eigen::MatrixX3f getNonground() const;
  double getTimeTaken() const;

  static Eigen::MatrixX3f pointCloud2ToEigen(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);
  static sensor_msgs::msg::PointCloud2 eigenToPointCloud2(
      const Eigen::MatrixX3f &points,
      const std_msgs::msg::Header &header);

 private:
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
};

}  // namespace patchworkpp_ros
