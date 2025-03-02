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
    ProjectionUtils::removeGroundPlane(point_cloud);
    filtered_point_cloud_ = point_cloud;
    
}

void LidarImageOverlay::detsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    // check if data is recieved 
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
    // -----------------------------------------------------------------------------------------------

    // make a copy of the image
    cv::Mat image = image_data_->image.clone();

    // define where the bounding box is using the 2d detections
    for (const auto& detection : msg->detections) {
        cv::Rect bbox(detection.bbox.center.position.x - detection.bbox.size_x / 2,
                        detection.bbox.center.position.y - detection.bbox.size_y / 2,
                        detection.bbox.size_x,
                        detection.bbox.size_y);

        // Find closest LiDAR point within bounding box
        pcl::PointXYZ closest_point;
        double min_distance = std::numeric_limits<double>::max();
        bool point_found = false;

        for (const auto& point : *filtered_point_cloud_) {
            auto projected_point = ProjectionUtils::projectLidarToCamera(tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero), projection_matrix_, point, image.cols, image.rows);
        
            // draw the filtered lidar points
            if (projected_point) {
                cv::circle(image, *projected_point, 3, cv::Scalar(0, 255, 0), -1); 
            }
            
            // draw and find distance of each object ---------------------------------------------------

            // check if the point is within the bounding box
            if (projected_point && bbox.contains(*projected_point)) {
            double distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
            // draw the points within the bounding boxes
            cv::circle(image, *projected_point, 5, cv::Scalar(0, 0, 255), -1);
            // finds the point closest
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
                point_found = true;
            }
            }
      }

/*       // draw the distance
      if (point_found){
        auto projected_point = ProjectionUtils::projectLidarToCamera(tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero), projection_matrix_, closest_point, image.cols, image.rows);
        if (projected_point){
            
            cv::circle(image, *projected_point, 5, cv::Scalar(0, 0, 255), -1);
            
             // Calculate bounding box center for text placement
             cv::Point text_position(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

             // Format distance as string
             std::stringstream ss;
             ss << std::fixed << std::setprecision(2) << min_distance << "m";
             std::string distance_str = ss.str();
 
             // Draw distance text
             cv::putText(image, distance_str, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
      } */
    }
    
    // Publish the filtered lidar data
    sensor_msgs::msg::PointCloud2 filtered_lidar_msg;
    pcl::toROSMsg(*filtered_point_cloud_, filtered_lidar_msg);
    filtered_lidar_msg.header = latest_lidar_msg_.header;
    filtered_lidar_pub_->publish(filtered_lidar_msg);

    // Publish the overlayed image
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