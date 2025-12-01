#include "lidar_localization/slam/s_gicp_registration"
#include <small_gicp/plc/pcl_registration.hpp>
#include <pcl/common/transforms.h>

namespace lidar_localization::slam {

SmallGicpRegistrator::SmallGicpRegistrator(const SmallGicpParams &params)
{
    params_  = params;
}

Eigen::Matrix4d SmallGicpRegistrator::align(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &source,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &target,
    const Eigen::Matrix4d &initial_guess,
    double &fitness_score) const
{
    //small_gicp registration object using PCL interface
    small_gicp::PCLGeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(params_.max_correspondence_distance);
    gicp.setMaximumIterations(params_.max_iterations);
    gicp.setTransformationEpsilon(params_.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(params_.euclidean_fitness_epsilon);
    gicp.setVoxelResolution(params_.voxel_resolution);

    //set up target and source input point clouds
    gicp.setInputSource(source)
    gicp.setInputTarget(target)

    //align point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI);
    gicp.align(*aligned)

    //fitness score, like a loss function
    fitness_score = gicp.getFitnessScore();

    //return transform matrix

    Eigen::Matrix4f transform_f= gicp.getFinalTransformation();
    
    return transform_f.cast<double>();

}

}