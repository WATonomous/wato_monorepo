#include "rclcpp/rclcpp.hpp"

#include <pcl/conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <map>
#include <algorithm>
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

class VoxelizerNode : public rclcpp::Node {
  struct Color {
    int r;
    int g;
    int b;
  };
  std::map<std::string, Color> detection_color = {
    {"car", Color{255, 165, 0}},
    {"pedestrian", Color{0, 0, 255}},
    {"bicycle", Color{255, 255, 0}},
    {"motorbike", Color{230, 230, 250}},
    {"truck", Color{25, 230, 25}},
    {"bus", Color{150, 150, 100}},
    {"trailer", Color{250, 230, 25}},
    {"barrier", Color{30, 230, 224}},
    {"construction_vehicle", Color{198, 124, 200}},
    {"traffic_cone", Color{159, 230, 224}},
  };
 public:
  VoxelizerNode();

 private:

  std::string lidar_topic_;
  std::string sub_topic_;
  std::string pub_topic_;

  Color get_color(const std::string idx);

  void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void voxel_publish(const sensor_msgs::msg::PointCloud2 voxel_grid);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxelgrid_pub_;

  const double voxel_size;
};
