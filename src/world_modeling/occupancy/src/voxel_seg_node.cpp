#include <memory>

#include "voxel_seg_node.hpp"

SegNode::SegNode() : Node("segmentation") {
  RCLCPP_INFO(this->get_logger(), "SEG Node\n");

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "LIDAR_TOP", ADVERTISING_FREQ,
    std::bind(
      &SegNode::voxel_sub_callback, this,
      std::placeholders::_1));

  voxelgrid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid_test", ADVERTISING_FREQ);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voxel_grid_marker", ADVERTISING_FREQ);
  seg_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("seg_voxel_grid", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}


void SegNode::voxel_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  float depthThreshold = 0.25;

  // Container for original & filtered data
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
  pcl_conversions::toPCL(*msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


  for (int p=0; p<temp_cloud->points.size(); ++p)
  {
    // remove point if it's within the threshold range
    if(temp_cloud->points[p].x <= depthThreshold){
      temp_cloud->points[p] = temp_cloud->points[temp_cloud->points.size()-1];
      temp_cloud->points.resize(temp_cloud->points.size()-1);
      --p;
    }


  }
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*temp_cloud, *cloud2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(cloud2);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

  pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2());
  voxel_filter.filter(*cloud_filtered2);

  auto pub_msg = sensor_msgs::msg::PointCloud2();
  pcl_conversions::fromPCL(*cloud_filtered2, pub_msg);

  this->voxel_publish(pub_msg);
}

void SegNode::voxel_publish(const sensor_msgs::msg::PointCloud2 voxel_grid)
{

  auto marker = visualization_msgs::msg::Marker();

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock().now(); // time
  header.frame_id = "LIDAR_TOP";

  marker.header = header;

  marker.type = 6; // Cube List

  marker.scale.x = voxel_size;
  marker.scale.y = voxel_size;
  marker.scale.z = voxel_size;
  marker.frame_locked = true;

  for (sensor_msgs::PointCloud2ConstIterator<float> it(voxel_grid, "x"); it != it.end(); ++it) {
    auto point = geometry_msgs::msg::Point();
    // re-quantize voxel grid by rounding to nearest voxel_size grid
    point.x = int(it[0]/voxel_size)*voxel_size;
    point.y = int(it[1]/voxel_size)*voxel_size;
    point.z = int(it[2]/voxel_size)*voxel_size;
    marker.points.push_back(point);
  }

  RCLCPP_INFO(this->get_logger(), "Publishing Voxel Grid Message...\n");
  voxelgrid_pub_->publish(voxel_grid);
  voxelgrid_pub_->publish(voxel_grid);
  marker_pub_->publish(marker);
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegNode>());
  rclcpp::shutdown();
  return 0;
}
