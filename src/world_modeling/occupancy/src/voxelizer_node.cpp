// based on a tutorial by https://github.com/noshluk2/ros2_learners/commits?author=noshluk2, altered to fit watonomous use cases
#include <memory>

#include "voxelizer_node.hpp"

VoxelizerNode::VoxelizerNode() : Node("voxelizer") {
  RCLCPP_INFO(this->get_logger(), "Voxelizer Node\n");

  pointcloud_sub_ = this->create_subscription<pcl::PCLPointCloud2>(
    "PointCloud", ADVERTISING_FREQ,
    std::bind(
      &VoxelizerNode::voxel_sub_callback, this,
      std::placeholders::_1));

  voxelgrid_pub_ =
    this->create_publisher<pcl::VoxelGrid>("VoxelGrid", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}


void VoxelizerNode::voxel_sub_callback(const pcl::PCLPointCloud2::SharedPtr msg)
{
  auto cloud = msg
  pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.05,0.05,0.05);
  voxel_filter.filter(*voxel_cloud);

  this->voxel_publish(voxel_filter);
}

void VoxelizerNode::voxel_publish(const pcl::VoxelGrid<pcl::PCLPointCloud2>SharedPtr msg)
{
  auto pub_msg = msg //I should probably just pass the msg but I am just going to copy the sample node.
  RCLCPP_INFO(this->get_logger(), "Publishing Voxel Grid Message...\n");
  voxel_pub_->publish(pub_msg);
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelizerNode>());
  rclcpp::shutdown();
  return 0;
}
