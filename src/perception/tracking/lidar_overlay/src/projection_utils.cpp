#include "projection_utils.hpp"



#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ProjectionUtils {
public:
    ProjectionUtils() {}

    PointCloud::Ptr removeGroundPlane(const PointCloud::Ptr& cloud, double distanceThreshold = 0.1, int maxIterations = 5000) {
        if (cloud->empty()) {
            return std::make_shared<PointCloud>();
        }

        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

        ransac.setDistanceThreshold(distanceThreshold);
        ransac.setMaxIterations(maxIterations);
        ransac.computeModel();
        ransac.getInliers(inliers);

        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
        inliers_ptr->indices = inliers;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_ptr);
        extract.setNegative(true); // Extract points *not* in the plane
        PointCloud::Ptr cloud_filtered(new PointCloud());
        extract.filter(*cloud_filtered);

        return cloud_filtered;
    }

private:
    // Any private member variables or helper functions can go here
    
};