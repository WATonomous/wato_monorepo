#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization::slam {

struct SmallGicpParams {
  double voxel_resolution = 1.0;
  int max_iterations = 30;
  double max_correspondence_distance = 1.0;
  //stops iteration if transformation isn't changing
  double transformation_epsilon = 1e-6;
  double euclidean_fitness_epsilon = 1.0;
};

class SmallGicpRegistrator {
public:
  explicit SmallGicpRegistrator(const SmallGicpParams &params);
  Eigen::Matrix4d align(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &source,
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &target,
      const Eigen::Matrix4d &initial_guess,
      double &fitness_score) const;
private:
  SmallGicpParams params_;
};

}