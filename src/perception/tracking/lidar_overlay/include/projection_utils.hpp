#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <optional>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ProjectionUtils {
    public:
        ProjectionUtils();

        static void removeGroundPlane(PointCloud::Ptr& cloud);

        static std::optional<cv::Point2d> projectLidarToCamera(
            const geometry_msgs::msg::TransformStamped& transform,
            const std::array<double, 12>& p,
            const pcl::PointXYZ& pt,
            int image_width = 1600,
            int image_height = 900);

    private:
    
};

#endif