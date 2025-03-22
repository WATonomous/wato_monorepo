#include "lidar_overlay.hpp"

LidarImageOverlay::LidarImageOverlay() : Node("lidar_image_overlay") {
    // SUBSCRIBERS -------------------------------------------------------------------------------------------------

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/annotated_img", 10, std::bind(&LidarImageOverlay::imageCallback, this, std::placeholders::_1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP", 10, std::bind(&LidarImageOverlay::lidarCallback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/CAM_FRONT/camera_info", 10, std::bind(&LidarImageOverlay::cameraInfoCallback, this, std::placeholders::_1));
    dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", 10, std::bind(&LidarImageOverlay::detsCallback, this, std::placeholders::_1));

    // PUBLISHERS --------------------------------------------------------------------------------------------------

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lidar_overlayed_image", 10);

    filtered_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_lidar", 10);
    cluster_centroid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_centroid", 10);

    bounding_box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LidarImageOverlay::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    camInfo_ = msg;
    camera_info_sub_.reset();
}

void LidarImageOverlay::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    std::unique_lock<std::shared_mutex> image_lock(image_mutex_);
    try {
        image_data_ = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void LidarImageOverlay::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    std::unique_lock<std::shared_mutex> lidar_lock(lidar_mutex_);

    // store the lidar msg
    latest_lidar_msg_ = *msg; 
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(latest_lidar_msg_, *point_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(point_cloud);
    voxel.setLeafSize(0.1f, 0.1f, 0.1f); // adjust parameters, higher values means less detailed, but faster compute
    voxel.filter(*point_cloud);

    // Remove the ground plane
    float distanceThreshold = 0.4; // Distance threshold for RANSAC plane fitting
    int maxIterations = 1200; // Maximum number of RANSAC iterations
    ProjectionUtils::removeGroundPlane(point_cloud, distanceThreshold, maxIterations);
    // store the pcl point cloud after RANSAC
    filtered_point_cloud_ = point_cloud;
}

void LidarImageOverlay::detsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    // Check if data is received
    if (!image_data_) {
        RCLCPP_WARN(this->get_logger(), "Have not received image data yet");
        return;
    }
    if (!tf_buffer_->canTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Transform not available");
        return;
    }
    if (!filtered_point_cloud_) {
        RCLCPP_WARN(this->get_logger(), "Lidar data not available");
        return;
    }

    try {
        transform = tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return;
    }

    // CLUSTERING -----------------------------------------------------------------------------------------------

    std::unique_lock<std::shared_mutex> lidar_lock(lidar_mutex_);

    // Remove outliers from the LiDAR point cloud
/*     int meanK = 30; // Number of neighbors to analyze for each point
    double stddevMulThresh = 1.5; // Standard deviation multiplier threshold

    ProjectionUtils::removeOutliers(filtered_point_cloud_, meanK, stddevMulThresh); */

    // Cluster the point cloud using DBSCAN
    double clusterTolerance = 1.2; // Distance tolerance for clustering
    int minClusterSize = 30; // Minimum number of points in a cluster
    int maxClusterSize = 1000; // Maximum number of points in a cluster

    // Merge clusters distance tolerance

    // Perform DBSCAN clustering, populate cluster_indices

    std::vector<pcl::PointIndices> cluster_indices;
    ProjectionUtils::dbscanCluster(filtered_point_cloud_, clusterTolerance, minClusterSize, maxClusterSize, cluster_indices);


    double densityWeight = 0.4;  // Density is least important
    double sizeWeight = 0.6;     // Size is most important
    double distanceWeight = 0.5; // Distance is moderately important
    double scoreThreshold = 0.7; // Clusters with a score below 0.7 are kept, increase to keep more clusters

    ProjectionUtils::filterClusterbyDensity(filtered_point_cloud_, cluster_indices, densityWeight, sizeWeight, distanceWeight, scoreThreshold); 

    float mergeTolerance = 1.5;
    ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, mergeTolerance);

     // filter by 2d bounding boxes
     //ProjectionUtils::filterClusterByBoundingBox(filtered_point_cloud_, cluster_indices, *msg, transform, camInfo_->p);

    // Assign colors to the clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ProjectionUtils::assignClusterColors(filtered_point_cloud_, cluster_indices, colored_clustered_cloud);


    // Convert the filtered and clustered point cloud to a ROS message
    sensor_msgs::msg::PointCloud2 filtered_lidar_msg;
    pcl::toROSMsg(*colored_clustered_cloud, filtered_lidar_msg);
    filtered_lidar_msg.header = latest_lidar_msg_.header;
    filtered_lidar_pub_->publish(filtered_lidar_msg);

    visualization_msgs::msg::MarkerArray bbox_msg;
    bbox_msg = ProjectionUtils::computeBoundingBox(filtered_point_cloud_, cluster_indices, latest_lidar_msg_);
    bounding_box_pub_->publish(bbox_msg);

    // -----------------------------------------------------------------------------------------------

    std::unique_lock<std::shared_mutex> image_lock(image_mutex_);

    // Make a copy of the image
    cv::Mat image = image_data_->image.clone();  // Clone the image to avoid modifying the original

    for (const auto& point : filtered_point_cloud_->points) {
        auto projected_point = ProjectionUtils::projectLidarToCamera(transform, camInfo_->p, point);
        if (projected_point) {
            cv::circle(image, *projected_point, 2, cv::Scalar(0, 255, 0), -1);  
        }
    }

    pcl::PointCloud<pcl::PointXYZ> centroid_cloud;
    for (const auto& cluster : cluster_indices) {
        // Step 1: Compute the centroid of the cluster
        pcl::PointXYZ centroid;
        ProjectionUtils::computeClusterCentroid(filtered_point_cloud_, cluster, centroid);
        centroid_cloud.push_back(centroid);

        if (ProjectionUtils::pointIn2DBoundingBox(centroid, *msg, transform, camInfo_->p)) {
            RCLCPP_INFO(this->get_logger(), "Centroid detected in 2D bounding box");
        }

        auto projected_centroid = ProjectionUtils::projectLidarToCamera(transform, camInfo_->p, centroid);
        if (projected_centroid) {
            cv::circle(image, *projected_centroid, 6, cv::Scalar(0, 0, 255), -1);  
        }
    }

    // Publish centroid cloud
    sensor_msgs::msg::PointCloud2 centroid_msg;
    pcl::toROSMsg(centroid_cloud, centroid_msg);
    centroid_msg.header = latest_lidar_msg_.header;
    cluster_centroid_pub_->publish(centroid_msg);

    // Publish the overlaid image
    auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    output_msg->header = msg->header;
    output_msg->encoding = "bgr8";
    image_pub_->publish(*output_msg);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarImageOverlay>());
    rclcpp::shutdown();
    return 0;
}