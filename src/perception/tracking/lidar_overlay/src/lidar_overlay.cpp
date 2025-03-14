#include "lidar_overlay.hpp"

LidarImageOverlay::LidarImageOverlay() : Node("lidar_image_overlay") {
    // SUBSCRIBERS -------------------------------------------------------------------------------------------------
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/annotated_img", 10, std::bind(&LidarImageOverlay::imageCallback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP", 10, std::bind(&LidarImageOverlay::lidarCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/CAM_FRONT/camera_info", 10, std::bind(&LidarImageOverlay::cameraInfoCallback, this, std::placeholders::_1));

    dets_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", 10, std::bind(&LidarImageOverlay::detsCallback, this, std::placeholders::_1));

    // PUBLISHERS --------------------------------------------------------------------------------------------------
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/lidar_overlayed_image", 10);

    filtered_lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_lidar", 10);

    bounding_box_pub_ = create_publisher<visualization_msgs::msg::Marker>("/bounding_boxes", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LidarImageOverlay::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::copy(msg->p.begin(), msg->p.end(), projection_matrix_.begin());
}

void LidarImageOverlay::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        image_data_ = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void LidarImageOverlay::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_lidar_msg_ = *msg; 
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(latest_lidar_msg_, *point_cloud);

    // Remove the ground plane

    ProjectionUtils::removeGroundPlane(point_cloud);
    filtered_point_cloud_ = point_cloud;
    
}

void LidarImageOverlay::detsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    // Check if data is received
    if (!image_data_) {
        RCLCPP_WARN(this->get_logger(), "Have not received image data yet");
        return;
    }
    if (!projection_matrix_[0]) {
        RCLCPP_WARN(this->get_logger(), "Have not received camera info yet");
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

    // CLUSTERING -----------------------------------------------------------------------------------------------

    // Remove outliers from the LiDAR point cloud
    int meanK = 100; // Number of neighbors to analyze for each point
    double stddevMulThresh = 2.0; // Standard deviation multiplier threshold

    ProjectionUtils::removeOutliers(filtered_point_cloud_, meanK, stddevMulThresh);

    // Cluster the point cloud using DBSCAN
    double clusterTolerance = 1.5; // Distance tolerance for clustering
    int minClusterSize = 100; // Minimum number of points in a cluster
    int maxClusterSize = 700; // Maximum number of points in a cluster


    // Merge clusters distance tolerance

    // Perform DBSCAN clustering, populate cluster_indices
    std::vector<pcl::PointIndices> cluster_indices;
    ProjectionUtils::dbscanCluster(filtered_point_cloud_, clusterTolerance, minClusterSize, maxClusterSize, cluster_indices);

    //size_t before_merge = cluster_indices.size();
    ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, 3.0);
    //RCLCPP_INFO(this->get_logger(), "[DEBUG] Merging: %ld -> %ld clusters", before_merge, cluster_indices.size());

    // Assign colors to the clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ProjectionUtils::assignClusterColors(filtered_point_cloud_, cluster_indices, colored_clustered_cloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& cluster : cluster_indices) {
        ProjectionUtils::filterClusterByBoundingBox(
            filtered_point_cloud_,
            cluster,
            *msg,
            tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero),
            projection_matrix_,
            bbox_filtered_cloud
        );
    }

    // Convert the filtered and clustered point cloud to a ROS message
    sensor_msgs::msg::PointCloud2 filtered_lidar_msg;
    pcl::toROSMsg(*bbox_filtered_cloud, filtered_lidar_msg);
    filtered_lidar_msg.header = latest_lidar_msg_.header;
    filtered_lidar_pub_->publish(filtered_lidar_msg);

    // -----------------------------------------------------------------------------------------------

    // Make a copy of the image
    cv::Mat image = image_data_->image.clone();

    // draw the filtered lidar points on the image
    /* for (const auto& point : filtered_point_cloud_->points) {
        // Project the LiDAR point to the camera frame
        auto projected_point = ProjectionUtils::projectLidarToCamera(
            tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero),
            projection_matrix_,
            point
        );
        // Check if the projected point is valid and within the image bounds
        if (projected_point) {
            // Draw the projected point on the image
            cv::circle(image, *projected_point, 3, cv::Scalar(0, 255, 0), -1); // Green circle with radius 3
        }
    } */

    for (const auto& cluster : cluster_indices) {
        // Step 1: Compute the centroid of the cluster
        pcl::PointXYZ centroid;
        ProjectionUtils::computeClusterCentroid(filtered_point_cloud_, cluster, centroid);

        // Step 2: Project the centroid onto the 2D image plane
        auto projected_centroid = ProjectionUtils::projectLidarToCamera3D(
            tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero),
            projection_matrix_,
            centroid
        );

        // Step 3: Draw the projected centroid on the image
        if (projected_centroid) {
            cv::circle(image, cv::Point2d(projected_centroid->x, projected_centroid->y), 10, cv::Scalar(0, 0, 255), -1); // Red circle with radius 5
        }
    }
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