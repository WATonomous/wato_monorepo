#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ProjectionUtils {
public:
    ProjectionUtils() {}
    static void removeGroundPlane(PointCloud::Ptr& cloud);

private:
   
};

#endif