#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "common_msgs/msg/combined_sensor.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::PointCloud2>;

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode() : Node("sensor_fusion_node")
    {
        camera_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, "/CAM_FRONT/image_rect_compressed");
        lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/LIDAR_TOP");
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> approximate_policy;

        // sync_ = std::make_shared<message_filters::TimeSynchronizer<approximate_policy>(approximate_policy(10), *camera_sub_, *lidar_sub_)>();
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(10), *camera_sub_, *lidar_sub_);
        sync_->registerCallback(&SensorFusionNode::callback, this);

        combined_sensor_pub_ = this->create_publisher<common_msgs::msg::CombinedSensor>("/combined_sensor", 1);
        RCLCPP_INFO(this->get_logger(), "Sensor fusion node has been initialized");
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud)
    {
      // Create a combined sensor msg
      auto combined_sensor_msg = std::make_unique<common_msgs::msg::CombinedSensor>();
      combined_sensor_msg->header = image->header;
      combined_sensor_msg->image = *image;
      combined_sensor_msg->lidar = *pointcloud;
      combined_sensor_pub_->publish(std::move(combined_sensor_msg));
      RCLCPP_INFO(this->get_logger(), "Published combined sensor message");
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> camera_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;
    std::shared_ptr<rclcpp::Publisher<common_msgs::msg::CombinedSensor>> combined_sensor_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionNode>());
    rclcpp::shutdown();

    return 0;
}
