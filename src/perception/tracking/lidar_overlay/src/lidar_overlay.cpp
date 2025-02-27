#include "lidar_overlay.hpp"



class LidarImageOverlay : public rclcpp::Node {
public:
    LidarImageOverlay() : Node("lidar_image_overlay") {
        // SUBSCRIBERS -------------------------------------------------------------------------------------------------
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/annotated_img", 10, std::bind(&LidarImageOverlay::imageCallback, this, std::placeholders::_1));
        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIDAR_TOP", 10, std::bind(&LidarImageOverlay::lidarCallback, this, std::placeholders::_1));
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/CAM_FRONT/camera_info", 10, std::bind(&LidarImageOverlay::cameraInfoCallback, this, std::placeholders::_1));

      //  dets_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      //      "/detections", 10, std::bind(&LidarImageOverlay::detsCallback, this, std::placeholders::_1));

        // PUBLISHERS --------------------------------------------------------------------------------------------------
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("lidar_overlayed_image", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    // IMAGE -----------------------------------------------------------------------------------------------------------
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv_bridge::CvImagePtr image_data_;
    std::array<double, 12> projection_matrix_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // LIDAR -----------------------------------------------------------------------------------------------------------

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // DETECTIONS ------------------------------------------------------------------------------------------------------

    //void detsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);


    // PROJECTION ------------------------------------------------------------------------------------------------------

    std::optional<cv::Point2d> projectLidarToCamera(
        const geometry_msgs::msg::TransformStamped& transform,
        const std::array<double, 12>& p,
        const pcl::PointXYZ& pt,
        int image_width,
        int image_height);

    // SUBSCRIBERS -----------------------------------------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
   // rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // PUBLISHERS ------------------------------------------------------------------------------------------------------

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    
};

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
    if (!image_data_) {
        RCLCPP_WARN(this->get_logger(), "Have not received image data yet");
        return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *point_cloud);

    // Get the transform from LiDAR to camera
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("CAM_FRONT", "LIDAR_TOP", tf2::TimePointZero);
    } 
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "TF2 exception: %s", ex.what());
        return;
    }

    // Project LiDAR points onto the image
    cv::Mat image = image_data_->image.clone();
    for (const auto& point : *point_cloud) {
        auto projected_point = projectLidarToCamera(transform, projection_matrix_, point, image.cols, image.rows);
        if (projected_point) {
            cv::circle(image, *projected_point, 3, cv::Scalar(0, 255, 0), -1); // Draw a green circle
        }
    }

    // Publish the overlayed image
    auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub_->publish(*output_msg);
}

std::optional<cv::Point2d> LidarImageOverlay::projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& p,
    const pcl::PointXYZ& pt,
    int image_width,
    int image_height) {
    // Convert LiDAR point to geometry_msgs::Point
    geometry_msgs::msg::Point orig_pt;
    orig_pt.x = pt.x;
    orig_pt.y = pt.y;
    orig_pt.z = pt.z;

    // Transform LiDAR point to camera frame
    geometry_msgs::msg::Point trans_pt;
    tf2::doTransform(orig_pt, trans_pt, transform);

    // Reject points behind the camera (z < 1)
    if (trans_pt.z < 1) {
        return std::nullopt;
    }

    // Project the point onto the image plane using the camera projection matrix
    double u = p[0] * trans_pt.x + p[1] * trans_pt.y + p[2] * trans_pt.z + p[3];
    double v = p[4] * trans_pt.x + p[5] * trans_pt.y + p[6] * trans_pt.z + p[7];
    double w = p[8] * trans_pt.x + p[9] * trans_pt.y + p[10] * trans_pt.z + p[11];

    // Normalize the projected coordinates
    cv::Point2d proj_pt;
    proj_pt.x = u / w;
    proj_pt.y = v / w;

    // Check if the projected point is within the image bounds
    if (proj_pt.x >= 0 && proj_pt.x < image_width && proj_pt.y >= 0 && proj_pt.y < image_height) {
        return proj_pt;
    }

    // Return nullopt if the point is outside the image bounds
    return std::nullopt;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarImageOverlay>());
    rclcpp::shutdown();
    return 0;
}