#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <opencv2/opencv.hpp>
#include <optional>
#include <random>

class ProjectionUtils {
 public:

  static std::optional<cv::Point2d> projectLidarToCamera(
      const geometry_msgs::msg::TransformStamped& transform, const std::array<double, 12>& p,
      const pcl::PointXYZ& pt);

  // CLUSTERING FUNCTIONS
  // -----------------------------------------------------------------------------------------

  static void euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                         double clusterTolerance, int minClusterSize,
                                         int maxClusterSize,
                                         std::vector<pcl::PointIndices>& cluster_indices);

  static void assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  const std::vector<pcl::PointIndices>& cluster_indices,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud);

  static void mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            double mergeTolerance);

  static void filterClusterbyDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     const std::vector<pcl::PointIndices>& cluster_indices,
                                     double densityWeight, double sizeWeight, double distanceWeight,
                                     double scoreThreshold);

  static bool computeClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     const pcl::PointIndices& cluster_indices,
                                     pcl::PointXYZ& centroid);

  // ROI FUNCTIONS
  // ------------------------------------------------------------------------------------------------
  static double computeMaxIOU8Corners(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr&  input_cloud,
    const pcl::PointIndices&                    cluster_indices,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>&               projection_matrix,
    const vision_msgs::msg::Detection2DArray&   detections,
    const float                                 object_detection_confidence);

  static void computeHighestIOUCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       std::vector<pcl::PointIndices>& cluster_indices,
                                       const vision_msgs::msg::Detection2DArray& detections,
                                       const geometry_msgs::msg::TransformStamped& transform,
                                       const std::array<double, 12>& projection_matrix,
                                       const float object_detection_confidence);

  // BOUNDING BOX FUNCTIONS
  // ----------------------------------------------------------------------------------------

    static visualization_msgs::msg::MarkerArray computeBoundingBox(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      const sensor_msgs::msg::PointCloud2& msg);

    static vision_msgs::msg::Detection3DArray compute3DDetection(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      const sensor_msgs::msg::PointCloud2& msg);

 private:
  static const int image_width_ = 1600;
  static const int image_height_ = 900;
};

#endif

