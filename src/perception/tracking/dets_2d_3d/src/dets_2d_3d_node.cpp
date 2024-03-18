#include "dets_2d_3d_node.hpp"
#include "projection_utils.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>


TrackingNode::TrackingNode() : Node("dets_2d_3d"), transformInited_{false} , lidarCloud_{new pcl::PointCloud<pcl::PointXYZ>()}
{
  // setup paramaters
  this->declare_parameter("camera_info_topic", "/CAM_FRONT/camera_info");
  this->declare_parameter("lidar_topic", "/LIDAR_TOP");
  this->declare_parameter("detections_topic", "/detections");

  this->declare_parameter("publish_detections_topic", "/detections_3d");
  this->declare_parameter("publish_markers_topic", "/markers_3d");
  this->declare_parameter("publish_clusters_topic", "/clustered_pc");

  this->declare_parameter("camera_frame", "CAM_FRONT");
  this->declare_parameter("lidar_frame", "LIDAR_TOP");

  this->declare_parameter("clustering_distances", std::vector<double>{5, 30, 45, 60});
  this->declare_parameter("clustering_thresholds", std::vector<double>{0.5, 1.1, 1.6, 2.1, 2.6});
  this->declare_parameter("cluster_size_min", 20.);
  this->declare_parameter("cluster_size_max", 100000.);
  this->declare_parameter("cluster_merge_threshold", 1.5);

  lidarFrame_ = this->get_parameter("camera_frame").as_string();
  cameraFrame_ = this->get_parameter("lidar_frame").as_string();

  // setup pub subs
  camInfo_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    this->get_parameter("camera_info_topic").as_string(), 10, 
    std::bind(&TrackingNode::readCameraInfo, this, std::placeholders::_1));

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("lidar_topic").as_string(), 10, 
    std::bind(&TrackingNode::receiveLidar, this, std::placeholders::_1));

  det_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    this->get_parameter("detections_topic").as_string(), 10,
    std::bind(&TrackingNode::receiveDetections, this, std::placeholders::_1));

  det3d_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    this->get_parameter("publish_detections_topic").as_string(), 10);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    this->get_parameter("publish_markers_topic").as_string(), 10);
  pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("publish_clusters_topic").as_string(), 10);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // initialize common params used by utils

  // 0 => 0-15m d=0.5, 1 => 15-30 d=1, 2 => 30-45 d=1.6, 3 => 45-60 d=2.1, 4 => >60   d=2.6
  ProjectionUtils::clustering_distances_ = this->get_parameter("clustering_distances").as_double_array();
  ProjectionUtils::clustering_thresholds_ = this->get_parameter("clustering_thresholds").as_double_array(); 

  ProjectionUtils::cluster_size_min_ = this->get_parameter("cluster_size_min").as_double();
  ProjectionUtils::cluster_size_max_ = this->get_parameter("cluster_size_max").as_double();
  ProjectionUtils::cluster_merge_threshold_ = this->get_parameter("cluster_merge_threshold").as_double();;

}

void TrackingNode::receiveLidar(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud_msg)
{
  // save latest lidar info
  std::lock_guard<std::mutex> guard_lidar(lidarCloud_mutex_);
  

  sensor_msgs::msg::PointCloud2 pointCloud = *pointCloud_msg;
  pcl::fromROSMsg(pointCloud, *lidarCloud_);
}

void TrackingNode::readCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::cout << msg->distortion_model << std::endl;
  camInfo_ = msg;
  camInfo_subscriber_.reset();
}

void TrackingNode::receiveDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (!transformInited_)
  {
    try {
      transform_ = tf_buffer_ ->lookupTransform(cameraFrame_, lidarFrame_, msg->header.stamp);
      transformInited_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
      return;
    }
  }

  // remove floor from lidar cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloudNoFloor(new pcl::PointCloud<pcl::PointXYZ>());
  {
    std::lock_guard<std::mutex> guard_lidar(lidarCloud_mutex_);
    ProjectionUtils::removeFloor(lidarCloud_, lidarCloudNoFloor);

    if (lidarCloudNoFloor->size() == 0) return; // do not process more, if all the points are floor points
    RCLCPP_INFO(this->get_logger(), "receive detection %ld vs %ld", lidarCloud_->size(), lidarCloudNoFloor->size());
  }

  // transform all lidar pts to 2d points in the camera
  std::vector<geometry_msgs::msg::Point> lidar2dProjs;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inCameraPoints(new pcl::PointCloud<pcl::PointXYZ>());
  for (const pcl::PointXYZ& pt : lidarCloudNoFloor->points)
  {    
    std::optional<geometry_msgs::msg::Point> proj = ProjectionUtils::projectLidarToCamera(transform_, camInfo_->p, pt);
    if (proj) 
    {
      inCameraPoints->emplace_back(pt);
      lidar2dProjs.emplace_back(*proj);
    }
  }
  
  // publish both rviz markers (for visualization) and a detection array
  visualization_msgs::msg::MarkerArray markerArray3d;
  vision_msgs::msg::Detection3DArray detArray3d;
  pcl::PointCloud<pcl::PointXYZRGB> mergedClusters; // coloured pc to visualize points in each cluster

  // process each detection in det array
  int bboxId = 0;
  for (const vision_msgs::msg::Detection2D& det : msg->detections)
  {
    vision_msgs::msg::BoundingBox2D bbox = det.bbox;

    // process lidarcloud with extrinsic trans from lidar to cam frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>());
    ProjectionUtils::pointsInBbox(inlierPoints, inCameraPoints, lidar2dProjs, bbox);
    if (inlierPoints->size() == 0) {
      RCLCPP_INFO(this->get_logger(), "no inliers found for detection %s", det.results[0].hypothesis.class_id.c_str());
      continue;
    }

    // clustering
    auto clusterAndBBoxes = ProjectionUtils::getClusteredBBoxes(inlierPoints);
    std::vector<std::shared_ptr<Cluster>> clusters = clusterAndBBoxes.first; // needed? for viz purposes only
    std::vector<vision_msgs::msg::BoundingBox3D> allBBoxes = clusterAndBBoxes.second;

    if (clusters.size() == 0 || allBBoxes.size() == 0) continue;

    // score each 3d bbox based on iou with teh original 2d bbox & the density of points in the cluster
    int bestIndex = highestIOUScoredBBox(allBBoxes, bbox, clusters);
    vision_msgs::msg::BoundingBox3D bestBBox = allBBoxes[bestIndex];

    // for visauzliation : adds detection to cluster pc visualization & the marker array & the 3d detection array
    mergedClusters += *(clusters[bestIndex]->getCloud());

    vision_msgs::msg::Detection3D det3d;
    det3d.bbox  = bestBBox;
    det3d.results = det.results;
    detArray3d.detections.emplace_back(det3d);

    visualization_msgs::msg::Marker marker;
    marker.type =  visualization_msgs::msg::Marker::CUBE;
    marker.scale = bestBBox.size;
    marker.pose = bestBBox.center;
    marker.header.frame_id = lidarFrame_;
    marker.header.stamp = msg->header.stamp;
    marker.id = bboxId;
    markerArray3d.markers.push_back(marker);
    ++bboxId;
  }

  det3d_publisher_->publish(detArray3d);
  marker_publisher_->publish(markerArray3d);

  sensor_msgs::msg::PointCloud2 pubCloud;
  pcl::toROSMsg(mergedClusters, pubCloud);
  pubCloud.header.frame_id = lidarFrame_;
  pubCloud.header.stamp = msg->header.stamp;

  pc_publisher_->publish(pubCloud);

  RCLCPP_INFO(this->get_logger(), "published %ld 3d detections\n\n", markerArray3d.markers.size());
  
}

int TrackingNode::highestIOUScoredBBox(
  const std::vector<vision_msgs::msg::BoundingBox3D> bboxes,
  const vision_msgs::msg::BoundingBox2D& detBBox,
  const std::vector<std::shared_ptr<Cluster>>& clusters)
{
  int bestScore = 0;
  int bestBBox = 0;

  for (size_t i=0; i<bboxes.size(); ++i)
  {
    vision_msgs::msg::BoundingBox3D b = bboxes[i];
    // project the 3d corners of the bbox to 2d camera frame
    pcl::PointXYZ top_left;
    top_left.x = b.center.position.x - b.size.x/2;
    top_left.y = b.center.position.y - b.size.y/2;
    top_left.z = b.center.position.z - b.size.z/2;

    pcl::PointXYZ bottom_right;
    bottom_right.x = b.center.position.x + b.size.x/2;
    bottom_right.y = b.center.position.y + b.size.y/2;
    bottom_right.z = b.center.position.z + b.size.z/2;

    std::optional<geometry_msgs::msg::Point> top_left2d = ProjectionUtils::projectLidarToCamera(transform_, camInfo_->p, top_left);
    std::optional<geometry_msgs::msg::Point> bottom_right2d = ProjectionUtils::projectLidarToCamera(transform_, camInfo_->p, bottom_right);

    if (!top_left2d || !bottom_right2d) return 0;

    vision_msgs::msg::BoundingBox2D bbox2d;
    bbox2d.center.position.x = ((top_left2d->x) + (bottom_right2d->x))/2;
    bbox2d.center.position.y = ((top_left2d->y) + (bottom_right2d->y))/2;
    bbox2d.size_x = abs(top_left2d->x - bottom_right2d->x);
    bbox2d.size_y = abs(top_left2d->y - bottom_right2d->y);

    double iou = iouScore(bbox2d, detBBox);

    // score also includes density of points (num points/ volume) in the 3d bbox
    double density = clusters[i]->size() / (b.size.x * b.size.y * b.size.z);
    double score = iou + density;

    if (score > bestScore)
    {
      bestScore = score;
      bestBBox = i;
    }
  }
  return bestBBox;
}

double TrackingNode::overlapBoundingBox(const vision_msgs::msg::BoundingBox2D& boxA, const vision_msgs::msg::BoundingBox2D& boxB)
{
  double overlapHeight =
      std::min(boxA.center.position.y + boxA.size_y/2, boxB.center.position.y + boxB.size_y/2) -
      std::max(boxA.center.position.y - boxA.size_y/2, boxB.center.position.y - boxB.size_y/2);
  if (overlapHeight <= 0) return 0;
  
  double overlapWidth = std::min(boxA.center.position.x + boxA.size_x/2, boxB.center.position.x + boxB.size_x/2) -
                        std::max(boxA.center.position.x - boxA.size_x/2, boxB.center.position.x - boxB.size_x/2);
  if (overlapWidth <= 0) return 0;
  
  return overlapHeight * overlapWidth;
}

double TrackingNode::iouScore(const vision_msgs::msg::BoundingBox2D& bboxA, const vision_msgs::msg::BoundingBox2D& bboxB)
{
  double overlap = overlapBoundingBox(bboxA, bboxB);
  return (overlap /
          (bboxA.size_x * bboxA.size_y + bboxB.size_x * bboxB.size_y - overlap));

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
