#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include "world_modeling_msgs/msg/voxel_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/header.hpp"


class SegNode : public rclcpp::Node
{
  public:
    static constexpr int ADVERTISING_FREQ = 20;
    SegNode();

  private:
  void voxel_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void voxel_publish(const sensor_msgs::msg::PointCloud2 voxel_grid);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxelgrid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr seg_grid_pub_;

  float voxel_size = 0.1;
};
