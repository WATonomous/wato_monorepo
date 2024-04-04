#include "voxelizer_node.hpp"

#include <memory>

VoxelizerNode::VoxelizerNode()
    : Node("voxelizer"), voxel_size{declare_parameter<double>("voxel_size", 0.2)} {
  this->declare_parameter<std::string>("lidar_plc_topic", "/LIDAR_TOP");
  this->declare_parameter<std::string>("detection3d_topic", "/detection3d");
  this->declare_parameter<std::string>("publish_vis_topic", "/voxel_grid_marker");
  
  this->get_parameter("lidar_pcl_topic", lidar_topic_);
  this->get_parameter("detection3d_topic", sub_topic_);
  this->get_parameter("publish_vis_topic", pub_topic_);
  RCLCPP_INFO(this->get_logger(), lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Voxelizer Node\n");
  //RCLCPP_INFO(this->get_logger(), "lidarpcl: %s", s.c_str());

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 10, std::bind(&VoxelizerNode::pointcloud_callback, this, std::placeholders::_1));
  detection3d_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      sub_topic_, 10, std::bind(&VoxelizerNode::detection3d_callback, this, std::placeholders::_1));
  voxelgrid_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic_, 10);
}

VoxelizerNode::Color VoxelizerNode::get_color(const std::string idx) {
  Color color{255, 22, 80};
  if (idx != "") {
    auto it = this->detection_color.find(idx);
    if (it != this->detection_color.end()) {
      color.r = it->second.r;
      color.g = it->second.g;
      color.b = it->second.b;
    }
  }
  return color;
}

void VoxelizerNode::detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
  // Code takes inspiration from https://github.com/NovoG93/vision_msgs_rviz_plugins/
  auto marker_array = visualization_msgs::msg::MarkerArray();

  for (size_t i = 0; i < msg->detections.size(); i++) {
    auto det = msg->detections[i];

    // get color associated to the label/class of the 3d detection
    VoxelizerNode::Color color = {130, 128, 244};
    if (det.results.size() > 0) {
      color = get_color(det.results[0].hypothesis.class_id);
      // more than one class, get result with highest confidence score
      auto iter = std::max_element(
        det.results.begin(),
        det.results.end(),
        [](const auto & a, const auto & b)
        {
          return a.hypothesis.score < b.hypothesis.score;
        });
      auto result_with_highest_score = *iter;
      color = get_color(result_with_highest_score.hypothesis.class_id);
    }

    // Create Marker
    auto marker = visualization_msgs::msg::Marker();
    marker.header = msg->header;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = det.bbox.center;
    marker.scale.x = det.bbox.size.x * voxel_size;
    marker.scale.y = det.bbox.size.y * voxel_size;
    marker.scale.z = det.bbox.size.z * voxel_size;

    marker.color.a = 1;
    marker.color.r = color.r / 255.0;
    marker.color.g = color.g / 255.0;
    marker.color.b = color.b / 255.0;

    marker.frame_locked = true;

    marker_array.markers.push_back(marker);
  }
  voxelgrid_pub_->publish(marker_array);
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
