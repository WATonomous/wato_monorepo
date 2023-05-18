#include <memory>

#include "sample_node.hpp"

SampleNode::SampleNode()
: Node("sample"), sample_(world_modeling::hd_map::Sample()), routing_(world_modeling::hd_map::HDMapRouting())
{
  sample_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("hd_map", ADVERTISING_FREQ);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1009),
      std::bind(&SampleNode::sample_publish, this));

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void SampleNode::sample_publish()
{
  lanelet::PointLayer& points = this->routing_.map_->pointLayer;

  lanelet::Point3d aPoint = *points.begin();

  std_msgs::msg::Header header; // empty header
  header.stamp = rclcpp::Clock().now(); // time
  
  auto marker = visualization_msgs::msg::Marker(); 
  marker.header = header;
  marker.id = 8;
  
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  std::vector<geometry_msgs::msg::Point> rosPoints;

  auto pt = geometry_msgs::msg::Point();
  pt.x = aPoint.x();
  pt.y = aPoint.y();
  pt.z = aPoint.z();

  rosPoints.push_back(pt);

  marker.points = rosPoints;

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  sample_pub_->publish(marker);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}
