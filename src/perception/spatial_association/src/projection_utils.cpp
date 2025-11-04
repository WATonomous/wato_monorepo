#include "projection_utils.hpp"

// PRE-CLUSTER FILTERING
// ----------------------------------------------------------------------------------------------------------------------


std::optional<cv::Point2d> ProjectionUtils::projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped& transform, const std::array<double, 12>& p,
    const pcl::PointXYZ& pt) {
  /*
      Projects a 3D lidar point onto a 2d camera image using the given extrinsic transformation and
     camera projection matrix

      Purpose: Returns 2d image coordinates as a cv::Point2d object
  */

  geometry_msgs::msg::PointStamped lidar_point;
  lidar_point.point.x = pt.x;
  lidar_point.point.y = pt.y;
  lidar_point.point.z = pt.z;

  // Transform LiDAR point to camera frame
  geometry_msgs::msg::PointStamped camera_point;
  tf2::doTransform(lidar_point, camera_point, transform);

  // ignore points behind the camera
  if (camera_point.point.z < 1) {
    return std::nullopt;
  }

  // Convert the projection matrix (std::array) to Eigen::Matrix<double, 3, 4>
  Eigen::Matrix<double, 3, 4> projection_matrix;
  projection_matrix << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11];

  // Convert camera point to Eigen vector (homogeneous coordinates)
  Eigen::Vector4d camera_point_eigen(camera_point.point.x, camera_point.point.y,
                                     camera_point.point.z, 1.0);

  // Project the point onto the image plane using the projection matrix
  Eigen::Vector3d projected_point = projection_matrix * camera_point_eigen;

  // Normalize the projected coordinates
  cv::Point2d proj_pt;
  proj_pt.x = projected_point.x() / projected_point.z();
  proj_pt.y = projected_point.y() / projected_point.z();

  // within the image bounds
  if (proj_pt.x >= 0 && proj_pt.x < image_width_ && proj_pt.y >= 0 && proj_pt.y < image_height_) {
    return proj_pt;
  }

  return std::nullopt;
}

// CLUSTERING FUNCTIONS
// ------------------------------------------------------------------------------------------------

void ProjectionUtils::euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 double clusterTolerance, int minClusterSize,
                                                 int maxClusterSize,
                                                 std::vector<pcl::PointIndices>& cluster_indices) {
  /*
      Segments distinct groups of point clouds based on cluster tolerance (euclidean distance) and
     size constraints

      Purpose: Populates a vector of cluster_indices, tells which indexes the cluster is in the
     original cloud
  */

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minClusterSize);
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
}

void ProjectionUtils::assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          const std::vector<pcl::PointIndices>& cluster_indices,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud) {
  if (cloud->empty() || cluster_indices.empty()) return;

  clustered_cloud->clear();  // Clear previous data
  clustered_cloud->points.reserve(cloud->size());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  for (const auto& indices : cluster_indices) {
    int r = dis(gen);
    int g = dis(gen);
    int b = dis(gen);
    for (const auto& index : indices.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud->points[index].x;
      point.y = cloud->points[index].y;
      point.z = cloud->points[index].z;
      point.r = r;
      point.g = g;
      point.b = b;
      clustered_cloud->points.push_back(point);
    }
  }
  clustered_cloud->width = clustered_cloud->points.size();
  clustered_cloud->height = 1;
  clustered_cloud->is_dense = true;
  clustered_cloud->header = cloud->header;
}

void ProjectionUtils::mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    double mergeTolerance) {
  /*
      merges two clusters based on the euclidean distance between their centroids, determined by
     merge tolerance

      Purpose: updates cluster_indices with the merged clusters
  */

  if (cloud->empty() || cluster_indices.empty()) return;

  std::vector<bool> merged(cluster_indices.size(), false);

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    Eigen::Vector4f centroid_i;
    pcl::compute3DCentroid(*cloud, cluster_indices[i].indices, centroid_i);

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;  // Skip if already merged

      // Compute centroid of cluster j
      Eigen::Vector4f centroid_j;
      pcl::compute3DCentroid(*cloud, cluster_indices[j].indices, centroid_j);

      // euclidean distance
      double distance = (centroid_i - centroid_j).norm();

      if (distance < mergeTolerance) {
        cluster_indices[i].indices.insert(cluster_indices[i].indices.end(),
                                          cluster_indices[j].indices.begin(),
                                          cluster_indices[j].indices.end());
        merged[j] = true;
      }
    }
  }

  // Remove merged clusters from the list
  std::vector<pcl::PointIndices> filtered_clusters;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (!merged[i]) {
      filtered_clusters.push_back(cluster_indices[i]);
    }
  }
  cluster_indices = filtered_clusters;
}

void ProjectionUtils::filterClusterbyDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const std::vector<pcl::PointIndices>& cluster_indices,
                                             double densityWeight, double sizeWeight,
                                             double distanceWeight, double scoreThreshold) {
  /*
      Applies weighted scoring of size, density, and distance to filter out clusters, with values
     normalized using arbitrary expected values

      Purpose: updates cluster_indices with removed clusters that are too big, too far, or too
     sparse
  */

  if (cloud->empty() || cluster_indices.empty()) return;

  // Define maximum expected values for normalization
  const double max_density = 700.0;  // Maximum density (points/m³)
  const double max_size =
      12.0;  // Maximum cluster size; the diagonal length of its extents (meters)
  const double max_distance = 60.0;  // Maximum distance of a cluster (meters)

  std::vector<pcl::PointIndices> filtered_clusters;

  for (const auto& clusters : cluster_indices) {
    if (clusters.indices.size() < 10) continue;  // Skip small clusters

    // Initialize min and max values for cluster bounds
    double min_x = std::numeric_limits<double>::max(),
           max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max(),
           max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max(),
           max_z = std::numeric_limits<double>::lowest();
    double total_distance = 0.0;

    // Calculate cluster bounds and total distance from origin
    for (const auto& index : clusters.indices) {
      const auto& pt = cloud->points[index];
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
      min_z = std::min(min_z, static_cast<double>(pt.z));
      max_z = std::max(max_z, static_cast<double>(pt.z));
      total_distance += std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    }

    // calculate cluster size (the diagonal length of the extents of the cluster)
    double cluster_size =
        std::sqrt((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y) +
                  (max_z - min_z) * (max_z - min_z));

    // calculate cluster density (points per unit volume)
    double cluster_volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
    double density = cluster_volume > 0 ? clusters.indices.size() / cluster_volume : 0;

    // calculate average distance from origin
    double avg_distance = total_distance / clusters.indices.size();

    // normalize the factors
    double normalized_density = density / max_density;
    double normalized_size = cluster_size / max_size;
    double normalized_distance = avg_distance / max_distance;

    //  weighted score
    double score = (normalized_density * densityWeight) + (normalized_size * sizeWeight) +
                   (normalized_distance * distanceWeight);

    if (score < scoreThreshold) {
      filtered_clusters.push_back(clusters);
    }
  }

  // Replace the original cluster indices with the filtered ones
  const_cast<std::vector<pcl::PointIndices>&>(cluster_indices) = filtered_clusters;
}

bool ProjectionUtils::computeClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const pcl::PointIndices& cluster_indices,
                                             pcl::PointXYZ& centroid) {
  if (cloud->empty() || cluster_indices.indices.empty()) return false;

  // Compute centroid of the cluster
  Eigen::Vector4f centroid_eigen;
  pcl::compute3DCentroid(*cloud, cluster_indices, centroid_eigen);

  // Assign values
  centroid.x = centroid_eigen[0];
  centroid.y = centroid_eigen[1];
  centroid.z = centroid_eigen[2];

  return true;
}


double ProjectionUtils::computeMaxIOU8Corners(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr&  input_cloud,
  const pcl::PointIndices&                    cluster_indices,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>&               projection_matrix,
  const vision_msgs::msg::Detection2DArray&   detections,
  const float                                 object_detection_confidence)
{
  // 1) compute 3D axis-aligned min/max
  float min_x =  std::numeric_limits<float>::infinity(),
        max_x = -std::numeric_limits<float>::infinity();
  float min_y =  std::numeric_limits<float>::infinity(),
        max_y = -std::numeric_limits<float>::infinity();
  float min_z =  std::numeric_limits<float>::infinity(),
        max_z = -std::numeric_limits<float>::infinity();

  for (auto idx : cluster_indices.indices) {
    const auto &P = input_cloud->points[idx];
    min_x = std::min(min_x, P.x);  max_x = std::max(max_x, P.x);
    min_y = std::min(min_y, P.y);  max_y = std::max(max_y, P.y);
    min_z = std::min(min_z, P.z);  max_z = std::max(max_z, P.z);
  }
  if (cluster_indices.indices.empty()) {
    return 0.0;
  }

  // 2) build all eight corners of the AABB
  std::array<pcl::PointXYZ,8> corners = {{
    {min_x, min_y, min_z}, {min_x, min_y, max_z},
    {min_x, max_y, min_z}, {min_x, max_y, max_z},
    {max_x, min_y, min_z}, {max_x, min_y, max_z},
    {max_x, max_y, min_z}, {max_x, max_y, max_z}
  }};

  // 3) project each corner & form a tight 2D rect
  double u0 =  std::numeric_limits<double>::infinity(),
         v0 =  std::numeric_limits<double>::infinity();
  double u1 = -std::numeric_limits<double>::infinity(),
         v1 = -std::numeric_limits<double>::infinity();

  for (auto &C : corners) {
    auto uv = projectLidarToCamera(transform, projection_matrix, C);
    if (!uv) continue;
    u0 = std::min(u0, uv->x);
    v0 = std::min(v0, uv->y);
    u1 = std::max(u1, uv->x);
    v1 = std::max(v1, uv->y);
  }
  if (u1 <= u0 || v1 <= v0) {
    return 0.0;
  }
  cv::Rect cluster_rect(u0, v0, u1 - u0, v1 - v0);

  // 4) compare to each detection bbox, return the best IoU
  double best_iou = 0.0;
  for (auto &det : detections.detections) {
    if (!det.results.empty() &&
        det.results[0].hypothesis.score < object_detection_confidence)
    {
      continue;
    }
    const auto &b = det.bbox;
    cv::Rect det_rect(
      b.center.position.x - b.size_x/2,
      b.center.position.y - b.size_y/2,
      b.size_x, b.size_y
    );
    double inter = (cluster_rect & det_rect).area();
    double uni   = cluster_rect.area() + det_rect.area() - inter;
    if (uni > 0.0) {
      best_iou = std::max(best_iou, inter/uni);
    }
  }
  return best_iou;
}

void ProjectionUtils::computeHighestIOUCluster(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
  std::vector<pcl::PointIndices>& cluster_indices,
  const vision_msgs::msg::Detection2DArray& detections,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>& projection_matrix,
  const float object_detection_confidence)
{
if (input_cloud->empty() || cluster_indices.empty()) {
  return;
}

double best_overall_iou = 0.0;
std::vector<pcl::PointIndices> kept_clusters;

for (auto &cluster : cluster_indices) {
  // computeMaxIOU8Corners projects the 8 AABB corners and returns the best IoU
  double iou = computeMaxIOU8Corners(
    input_cloud,
    cluster,
    transform,
    projection_matrix,
    detections,
    object_detection_confidence
  );

  if (iou > best_overall_iou) {
    best_overall_iou = iou;
    kept_clusters.push_back(cluster);
  }
}

// only keep the cluster(s) with highest IoU
cluster_indices = std::move(kept_clusters);
}

// BOUNDING BOX FUNCTIONS
// --------------------------------------------------------------------------------------------

visualization_msgs::msg::MarkerArray ProjectionUtils::computeBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg) {
  /*
      calculates the bounding box enclosing a cluster using the min/max points and the orientation
     using minAreaRect the function finds the smallest enclosing rotated rectangle for the points in
     the cluster

      Purpose: returns the marker array of all the bounding boxes created around each cluster
  */

  visualization_msgs::msg::MarkerArray marker_array;

  if (cloud->empty()) return marker_array;

  int id = 0;
  for (const auto& cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Vector4f min_point = Eigen::Vector4f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector4f max_point = Eigen::Vector4f::Constant(std::numeric_limits<float>::lowest());

    // get the min/max points
    for (const auto& index : cluster.indices) {
      const auto& pt = cloud->points[index];
      min_point.x() = std::min(min_point.x(), pt.x);
      min_point.y() = std::min(min_point.y(), pt.y);
      min_point.z() = std::min(min_point.z(), pt.z);

      max_point.x() = std::max(max_point.x(), pt.x);
      max_point.y() = std::max(max_point.y(), pt.y);
      max_point.z() = std::max(max_point.z(), pt.z);
    }

    // Compute bounding box center and size
    Eigen::Vector3f bbox_center = 0.5f * (min_point.head<3>() + max_point.head<3>());
    Eigen::Vector3f bbox_size = max_point.head<3>() - min_point.head<3>();

    // Calculate orientation using minAreaRect as done in getBoundingBox
    double rz = 0;

    {
      std::vector<cv::Point2f> points;
      for (const auto& index : cluster.indices) {
        cv::Point2f pt;
        pt.x = cloud->points[index].x;
        pt.y = cloud->points[index].y;
        points.push_back(pt);
      }

      cv::RotatedRect box = cv::minAreaRect(points);
      rz = box.angle * 3.14 / 180;  // Convert angle to radians

      // Update position and size with the rotated bounding box data
      bbox_center.x() = box.center.x;
      bbox_center.y() = box.center.y;
      bbox_size.x() = box.size.width;
      bbox_size.y() = box.size.height;
    }

    // initialize marker
    visualization_msgs::msg::Marker bbox_marker;
    bbox_marker.header = msg.header;
    bbox_marker.ns = "bounding_boxes";
    bbox_marker.id = id++;
    bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbox_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    bbox_marker.pose.position.x = bbox_center.x();
    bbox_marker.pose.position.y = bbox_center.y();
    bbox_marker.pose.position.z = bbox_center.z();

    // Set orientation using the calculated rotation
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, rz);  // Set only the Z-rotation
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(quat);
    bbox_marker.pose.orientation = msg_quat;

    // Size
    bbox_marker.scale.x = bbox_size.x();
    bbox_marker.scale.y = bbox_size.y();
    bbox_marker.scale.z = bbox_size.z();

    // Color
    bbox_marker.color.r = 0.0;
    bbox_marker.color.g = 0.0;
    bbox_marker.color.b = 0.0;
    bbox_marker.color.a = 0.2;

    bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.15);

    marker_array.markers.push_back(bbox_marker);
  }

  return marker_array;
}

// compute detection 3d array
vision_msgs::msg::Detection3DArray ProjectionUtils::compute3DDetection(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg)
{
  vision_msgs::msg::Detection3DArray det_arr;
  // copy the same header you used for the markers
  det_arr.header = msg.header;

  int id = 0;
  for (const auto &cluster : cluster_indices) {
    // 1) compute axis-aligned min/max (same as you do in computeBoundingBox)
    Eigen::Vector4f min_pt(+INFINITY, +INFINITY, +INFINITY, 1.0),
                    max_pt(-INFINITY, -INFINITY, -INFINITY, 1.0);
    for (auto idx : cluster.indices) {
      const auto &P = cloud->points[idx];
      min_pt.x() = std::min(min_pt.x(), P.x);
      min_pt.y() = std::min(min_pt.y(), P.y);
      min_pt.z() = std::min(min_pt.z(), P.z);
      max_pt.x() = std::max(max_pt.x(), P.x);
      max_pt.y() = std::max(max_pt.y(), P.y);
      max_pt.z() = std::max(max_pt.z(), P.z);
    }

    // 2) fill in a Detection3D
    vision_msgs::msg::Detection3D det;
    det.header = msg.header;

    // Hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hypo;
    hypo.hypothesis.class_id = "cluster";    // or whatever label
    hypo.hypothesis.score    = 1.0;          // or your confidence metric
    det.results.push_back(hypo);

    // Bounding box center
    geometry_msgs::msg::Pose &pose = det.bbox.center;
    pose.position.x = 0.5 * (min_pt.x() + max_pt.x());
    pose.position.y = 0.5 * (min_pt.y() + max_pt.y());
    pose.position.z = 0.5 * (min_pt.z() + max_pt.z());

    // Orientation — if you want axis-aligned, just leave it identity
    pose.orientation.w = 1.0;

    // Size
    det.bbox.size.x = max_pt.x() - min_pt.x();
    det.bbox.size.y = max_pt.y() - min_pt.y();
    det.bbox.size.z = max_pt.z() - min_pt.z();

    det_arr.detections.push_back(det);
    ++id;
  }

  return det_arr;
}