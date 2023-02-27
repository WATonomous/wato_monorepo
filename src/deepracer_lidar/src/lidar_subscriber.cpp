#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <string>
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.

using std::placeholders::_1;

// ROS2 node for reading lidar sensor data and produces visualization using OpenCV
class ReadingLaser : public rclcpp::Node {
public:

  std::string subscribe_topic = "/rplidar_ros/scan";
  std::string publish_topic = "/lidar_vis";
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_publisher;

  ReadingLaser() : Node("reading_laser") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscribe_topic, default_qos,
        std::bind(&ReadingLaser::topic_callback, this, _1));

    vis_publisher = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, default_qos);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {


    // Create a new 480x480 image
    int IMAGE_WIDTH = 480;
    float IMAGE_CENTER = (float)IMAGE_WIDTH/2;
    cv::Mat cv_image(cv::Size(IMAGE_WIDTH, IMAGE_WIDTH), CV_8UC3);
    cv_image = cv::Scalar(0,0,0);

 
    // Generate an image where each pixel is a random color
    // cv::randu(cv_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // angle delta in rads
    float angle_increment = _msg->angle_increment;
    float cur_angle = _msg->angle_min;
    float radius = 25;

    // create a blank image
    // draw the lidar points
    for (auto m : _msg->ranges) {
        

        int dist = m*radius;
        float x = cos(cur_angle)*dist+IMAGE_CENTER;
        float y = sin(cur_angle)*dist+IMAGE_CENTER;
        cur_angle += angle_increment;
        
        cv::Point center(x, y);//Declaring the center point
        cv::Scalar line_Color(0, 255, 0);//Color of the circle

        // RCLCPP_INFO(this->get_logger(), "Drawing point: (%f %f), range: %f %d %f %f", x,y, m, _msg->ranges.size(), angle_increment, cur_angle);
        int thickness = 1;//thickens of the line
        int circle_size = 1;
        cv::circle(cv_image, center, circle_size, line_Color, thickness);
    }

    RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->angle_min,
                _msg->angle_max);

    // Convert cv image to ros2 message
    auto pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image)
        .toImageMsg();

    vis_publisher->publish(*pub_msg.get());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadingLaser>();
  RCLCPP_INFO(node->get_logger(), "Starting laser node...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}