#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "world_modeling_msgs/msg/voxel_grid.hpp"


class VoxelizerNode : public rclcpp::Node
{
  public:
    static constexpr int ADVERTISING_FREQ = 20;
    VoxelizerNode();

  private:
  void VoxelizerNode::voxel_sub_callback(const pcl::PCLPointCloud2::Ptr msg);

  void VoxelizerNode::voxel_publish(const pcl::VoxelGrid<pcl::PCLPointCloud2>Ptr msg);

  rclcpp::Subscription<pcl::PCLPointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<pcl::VoxelGrid>::SharedPtr voxelgrid_pub_;

  world_modeling::occupancy::Voxilizer voxilizer_;
};
