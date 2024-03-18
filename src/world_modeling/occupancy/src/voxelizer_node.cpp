#include "voxelizer_node.hpp"

#include <memory>

VoxelizerNode::VoxelizerNode()
    : Node("voxelizer"), voxel_size{declare_parameter<double>("voxel_size", 0.1)} {
  RCLCPP_INFO(this->get_logger(), "Voxelizer Node\n");

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "LIDAR_TOP", 10, std::bind(&VoxelizerNode::pointcloud_callback, this, std::placeholders::_1));
  voxelgrid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid_test", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voxel_grid_marker", 10);
}

void VoxelizerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*msg, *cloud2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(cloud2);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

  pcl::PCLPointCloud2::Ptr cloud_filtered2(new pcl::PCLPointCloud2());
  voxel_filter.filter(*cloud_filtered2);

  auto pub_msg = sensor_msgs::msg::PointCloud2();
  pcl_conversions::fromPCL(*cloud_filtered2, pub_msg);

  this->voxel_publish(pub_msg);
}

void VoxelizerNode::voxel_publish(const sensor_msgs::msg::PointCloud2 voxel_grid) {
  auto marker = visualization_msgs::msg::Marker();

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock().now();  // time
  header.frame_id = "LIDAR_TOP";

  marker.header = header;

  marker.type = 6;  // Cube List

  marker.scale.x = voxel_size;
  marker.scale.y = voxel_size;
  marker.scale.z = voxel_size;
  marker.frame_locked = true;

  for (sensor_msgs::PointCloud2ConstIterator<float> it(voxel_grid, "x"); it != it.end(); ++it) {
    auto point = geometry_msgs::msg::Point();
    // re-quantize voxel grid by rounding to nearest voxel_size grid
    point.x = int(it[0] / voxel_size) * voxel_size;
    point.y = int(it[1] / voxel_size) * voxel_size;
    point.z = int(it[2] / voxel_size) * voxel_size;
    marker.points.push_back(point);
  }

  // RCLCPP_INFO(this->get_logger(), "Publishing Voxel Grid Message...\n");
  voxelgrid_pub_->publish(voxel_grid);
  marker_pub_->publish(marker);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelizerNode>());
  rclcpp::shutdown();
  return 0;
}
