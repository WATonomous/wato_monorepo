#include "occupancy_segmentation_node.hpp"
#include <iostream>
#include <memory>

OccupancySegmentationNode::OccupancySegmentationNode() : Node("occupancy_segmentation") {
  // Declare parameters
  this->declare_parameter<int>("num_zones", 4);
  this->declare_parameter<double>("l_min", 2.7);
  this->declare_parameter<double>("l_max", 80.0);
  this->declare_parameter<double>("md", 0.3);
  this->declare_parameter<double>("mh", -1.1);
  this->declare_parameter<int>("min_num_points", 10);
  this->declare_parameter<int>("num_seed_points", 20);
  this->declare_parameter<double>("th_seeds", 20);
  this->declare_parameter<double>("uprightness_thresh", 0.5);
  this->declare_parameter<int>("num_rings_of_interest", 4);
  this->declare_parameter<double>("sensor_height", 1.7);
  this->declare_parameter<double>("global_el_thresh", 0.0);
  this->declare_parameter<std::vector<int>>("zone_rings", std::vector<int>{2, 4, 4, 4});
  this->declare_parameter<std::vector<int>>("zone_sectors", std::vector<int>{16, 32, 54, 32});
  this->declare_parameter<std::vector<double>>("flatness_thr",
                                               std::vector<double>{0.0005, 0.000725, 0.001, 0.001});
  this->declare_parameter<std::vector<double>>("elevation_thr",
                                               std::vector<double>{0.523, 0.746, 0.879, 1.125});
  this->declare_parameter<bool>("adaptive_selection_en", bool(false));
  this->declare_parameter<std::string>("lidar_input_topic", std::string("/velodyne_points"));
  this->declare_parameter<std::string>("ground_output_topic", std::string("/ground_points"));
  this->declare_parameter<std::string>("nonground_output_topic", std::string("/nonground_points"));

  // Retrieve parameters
  int num_zones = this->get_parameter("num_zones").as_int();
  double l_min = this->get_parameter("l_min").as_double();
  double l_max = this->get_parameter("l_max").as_double();
  double md = this->get_parameter("md").as_double();
  double mh = this->get_parameter("mh").as_double();
  int min_num_points = this->get_parameter("min_num_points").as_int();
  int num_seed_points = this->get_parameter("num_seed_points").as_int();
  double th_seeds = this->get_parameter("th_seeds").as_double();
  double uprightness_thresh = this->get_parameter("uprightness_thresh").as_double();
  int num_rings_of_interest = this->get_parameter("num_rings_of_interest").as_int();
  double sensor_height = this->get_parameter("sensor_height").as_double();
  double global_el_thresh = this->get_parameter("global_el_thresh").as_double();
  auto zone_rings = this->get_parameter("zone_rings").as_integer_array();
  auto zone_sectors = this->get_parameter("zone_sectors").as_integer_array();
  auto flatness_thr = this->get_parameter("flatness_thr").as_double_array();
  auto elevation_thr = this->get_parameter("elevation_thr").as_double_array();
  bool adaptive_selection_en = this->get_parameter("adaptive_selection_en").as_bool();
  std::string lidar_input_topic = this->get_parameter("lidar_input_topic").as_string();
  std::string ground_output_topic = this->get_parameter("ground_output_topic").as_string();
  std::string nonground_output_topic = this->get_parameter("nonground_output_topic").as_string();

  _patchwork = OccupancySegmentationCore<PointXYZIRT>(
      num_zones, l_min, l_max, md, mh, min_num_points, num_seed_points, th_seeds,
      uprightness_thresh, num_rings_of_interest, sensor_height, global_el_thresh, zone_rings,
      zone_sectors, flatness_thr, elevation_thr, adaptive_selection_en);

  _subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_input_topic, 10,
      std::bind(&OccupancySegmentationNode::subscription_callback, this, std::placeholders::_1));

  _ground_publisher =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_output_topic, 10);
  _nonground_publisher =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(nonground_output_topic, 10);
}

void OccupancySegmentationNode::subscription_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud) {
  pcl::PointCloud<PointXYZIRT> temp_cloud;
  RCLCPP_DEBUG(this->get_logger(), "Header incoming: %s", lidar_cloud->header.frame_id.c_str());
  pcl::fromROSMsg(*lidar_cloud, temp_cloud);

  pcl::PointCloud<PointXYZIRT> ground;
  pcl::PointCloud<PointXYZIRT> nonground;

  ground.clear();
  nonground.clear();
  ground.header = temp_cloud.header;
  nonground.header = temp_cloud.header;

  auto begin = Clock::now();
  _patchwork.segment_ground(temp_cloud, ground, nonground);
  auto end = Clock::now();

  int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
  RCLCPP_DEBUG(this->get_logger(), "Runtime for Patchwork: %i ms", duration);
  RCLCPP_DEBUG(this->get_logger(), "Temp_cloud points %i", static_cast<int>(temp_cloud.size()));
  RCLCPP_DEBUG(this->get_logger(), "Ground points %i", static_cast<int>(ground.size()));
  RCLCPP_DEBUG(this->get_logger(), "Non ground points %i", static_cast<int>(nonground.size()));

  sensor_msgs::msg::PointCloud2 ground_msg;
  sensor_msgs::msg::PointCloud2 nonground_msg;

  pcl::toROSMsg(ground, ground_msg);
  pcl::toROSMsg(nonground, nonground_msg);
  RCLCPP_DEBUG(this->get_logger(), "Header outgoing: %s", ground_msg.header.frame_id.c_str());

  _ground_publisher->publish(ground_msg);
  _nonground_publisher->publish(nonground_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
