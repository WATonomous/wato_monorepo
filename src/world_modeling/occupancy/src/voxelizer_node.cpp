#include "voxelizer_node.hpp"

#include <memory>

VoxelizerNode::VoxelizerNode()
    : Node("voxelizer"), voxel_size{declare_parameter<double>("voxel_size", 1.0)} {
  RCLCPP_INFO(this->get_logger(), "Voxelizer Node\n");

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "LIDAR_TOP", 10, std::bind(&VoxelizerNode::pointcloud_callback, this, std::placeholders::_1));
  voxelgrid_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_grid_marker", 10);
}

void VoxelizerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*msg, *pcl_cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(pcl_cloud);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

  pcl::PCLPointCloud2::Ptr pcl_voxelized_pointcloud(new pcl::PCLPointCloud2());
  voxel_filter.filter(*pcl_voxelized_pointcloud);

  auto ros_voxelized_pointcloud = sensor_msgs::msg::PointCloud2();
  pcl_conversions::fromPCL(*pcl_voxelized_pointcloud, ros_voxelized_pointcloud);

  auto marker_array = visualization_msgs::msg::MarkerArray();
  auto marker = visualization_msgs::msg::Marker();
  marker.header = msg->header;
  marker.type = 6;  // Cube List
  marker.scale.x = voxel_size;
  marker.scale.y = voxel_size;
  marker.scale.z = voxel_size;
  marker.frame_locked = true;

  for (sensor_msgs::PointCloud2ConstIterator<float> it(ros_voxelized_pointcloud, "x");
       it != it.end(); ++it) {
    auto point = geometry_msgs::msg::Point();
    // re-quantize voxel grid by rounding to nearest voxel_size grid
    point.x = std::round(it[0] / voxel_size) * voxel_size;
    point.y = std::round(it[1] / voxel_size) * voxel_size;
    point.z = std::round(it[2] / voxel_size) * voxel_size;
    marker.points.push_back(point);
  }
  marker_array.markers.push_back(marker);

  voxelgrid_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelizerNode>());
  rclcpp::shutdown();
  return 0;
}
