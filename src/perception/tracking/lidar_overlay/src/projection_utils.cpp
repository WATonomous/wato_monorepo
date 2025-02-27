#include "projection_utils.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

void ProjectionUtils::removeGroundPlane(PointCloud::Ptr& cloud) {
    if (cloud->empty()) {
        return;
    }

    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    ransac.setDistanceThreshold(0.6);
    ransac.setMaxIterations(1000);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    inliers_ptr->indices = inliers;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_ptr);
    extract.setNegative(true);
    PointCloud::Ptr cloud_filtered(new PointCloud());
    extract.filter(*cloud_filtered);

    cloud->swap(*cloud_filtered);
}