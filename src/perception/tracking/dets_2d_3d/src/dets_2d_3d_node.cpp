#include "dets_2d_3d_node.hpp"
#include "det_utils.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>

TrackingNode::TrackingNode() : Node("dets_2d_3d"), lidarCloud_{new pcl::PointCloud<pcl::PointXYZ>()}, transformInited_{false} 
{
  camInfo_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_FRONT/camera_info", 10, 
    std::bind(&TrackingNode::readCameraInfo, this, std::placeholders::_1));

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/LIDAR_TOP", 10, 
    std::bind(&TrackingNode::receiveLidar, this, std::placeholders::_1));

  det_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections", 10,
    std::bind(&TrackingNode::receiveDetections, this, std::placeholders::_1));

  det3d_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/detections_3d", 10);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers_3d", 10);
  pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/trans_pc", 10);
  pc_publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/trans_pc2", 10);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
        transform_ = tf_buffer_ ->lookupTransform("CAM_FRONT", "LIDAR_TOP", msg->header.stamp);
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
    DetUtils::removeFloor(lidarCloud_, lidarCloudNoFloor);

    if (lidarCloudNoFloor->size() == 0) return; // end if there is only a floor
    RCLCPP_INFO(this->get_logger(), "receive detection %ld vs %ld", lidarCloud_->size(), lidarCloudNoFloor->size());
  }

  // transform all lidar pts to 2d
  std::vector<geometry_msgs::msg::Point> lidar2dProjs;
  for (const pcl::PointXYZ& pt : lidarCloudNoFloor->points)
  {
    geometry_msgs::msg::Point proj = DetUtils::projectLidarToCamera(transform_, camInfo_->p, pt);
    if (proj.z >= 0) 
    {
      lidar2dProjs.emplace_back(proj);
      // RCLCPP_INFO(this->get_logger(), "transform to 2d %f , %f, %f", proj.x, proj.y, proj.z);
    }
  }

  // temporarily also publish viz markers
  visualization_msgs::msg::MarkerArray markerArray3d;
  vision_msgs::msg::Detection3DArray detArray3d;

  pcl::PointCloud<pcl::PointXYZ> mergedClusters;

  // process each detection in det array
  int bboxId = 0;
  for (const vision_msgs::msg::Detection2D& det : msg->detections)
  {
    vision_msgs::msg::BoundingBox2D bbox = det.bbox;

    // process lidarcloud with extrinsic trans from lidar to cam frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>());
    DetUtils::pointsInBbox(inlierPoints, lidarCloudNoFloor, lidar2dProjs, bbox);
    if (inlierPoints->size() == 0) continue;

    mergedClusters += *inlierPoints;
    
    RCLCPP_INFO(this->get_logger(), "restrict to 2d %ld vs %ld", inlierPoints->size(), lidarCloudNoFloor->size());

    // clustering
    auto clusterAndBBoxes = DetUtils::getClusteredBBoxes(inlierPoints);

    RCLCPP_INFO(this->get_logger(), "(x,y) : (%f, %f) size:(%f, %f) : get merged cloud %ld clustered bboxes  %ld", 
      bbox.center.position.x, bbox.center.position.y, bbox.size_x, bbox.size_y,
      clusterAndBBoxes.first.size(), clusterAndBBoxes.second.size());
    
    std::vector<std::shared_ptr<Cluster>> clusters = clusterAndBBoxes.first; // needed? for viz purposes only
    std::vector<vision_msgs::msg::BoundingBox3D> allBBoxes = clusterAndBBoxes.second;

    if (clusters.size() == 0 || allBBoxes.size() == 0) continue;

    // [TODO] fix scoring bboxes by projecting & iou
    int bestIndex = highestIOUScoredBBox(allBBoxes, bbox);
    vision_msgs::msg::BoundingBox3D bestBBox = allBBoxes[bestIndex];
    // mergedClusters += *(clusters[bestIndex]->getCloud());

    RCLCPP_INFO(this->get_logger(), "scored bboxes");

    // find 3d box that encloses clustered pointcloud & add to det array
    vision_msgs::msg::Detection3D det3d;
    det3d.bbox  = bestBBox;
    det3d.results = det.results;
    detArray3d.detections.emplace_back(det3d);

    // make marker & add to array (TEMP PUBLISH ALL BBOXES, NOT JUST "BEST")
    for (const auto& maybeBbox : allBBoxes)
    {
      visualization_msgs::msg::Marker marker;
      marker.type =  visualization_msgs::msg::Marker::CUBE;
      marker.scale = maybeBbox.size;
      marker.pose = maybeBbox.center;
      marker.header.frame_id = "CAM_FRONT_RIGHT";
      marker.header.stamp = msg->header.stamp;
      marker.id = bboxId;

      markerArray3d.markers.push_back(marker);
      ++bboxId;
    }
  }

  det3d_publisher_->publish(detArray3d);
  marker_publisher_->publish(markerArray3d);

  sensor_msgs::msg::PointCloud2 pubCloud2;
  pcl::toROSMsg(mergedClusters, pubCloud2);
  pubCloud2.header.frame_id = "LIDAR_TOP";
  pubCloud2.header.stamp = msg->header.stamp;

  sensor_msgs::msg::PointCloud2 pubCloud;
  pcl::toROSMsg(*lidarCloudNoFloor, pubCloud);
  pubCloud.header.frame_id = "LIDAR_TOP";
  pubCloud.header.stamp = msg->header.stamp;

  pc_publisher2_->publish(pubCloud2);
  pc_publisher_->publish(pubCloud);

  RCLCPP_INFO(this->get_logger(), "published 3d detection %ld", markerArray3d.markers.size());
}

int TrackingNode::highestIOUScoredBBox(
  const std::vector<vision_msgs::msg::BoundingBox3D> bboxes,
  const vision_msgs::msg::BoundingBox2D& detBBox)
{
  int bestScore = 0;
  int bestBBox = 0;

  for (int i=0; i<bboxes.size(); ++i)
  {
    vision_msgs::msg::BoundingBox3D b = bboxes[i];
    // project bbox center to 2d
    pcl::PointXYZ top_left;
    top_left.x = b.center.position.x - b.size.x/2;
    top_left.y = b.center.position.y - b.size.y/2;
    top_left.z = b.center.position.z - b.size.z/2;

    pcl::PointXYZ bottom_right;
    bottom_right.x = b.center.position.x + b.size.x/2;
    bottom_right.y = b.center.position.y + b.size.y/2;
    bottom_right.z = b.center.position.z + b.size.z/2;

    geometry_msgs::msg::Point top_left2d = DetUtils::projectLidarToCamera(transform_, camInfo_->p, top_left);
    geometry_msgs::msg::Point bottom_right2d = DetUtils::projectLidarToCamera(transform_, camInfo_->p, bottom_right);

    vision_msgs::msg::BoundingBox2D bbox2d;
    bbox2d.center.position.x = ((top_left2d.x/top_left2d.z) + (bottom_right2d.x/bottom_right2d.z))/2;
    bbox2d.center.position.y = ((top_left2d.y/top_left2d.z) + (bottom_right2d.y/bottom_right2d.z))/2;
    bbox2d.size_x = abs(top_left2d.x - bottom_right2d.x);
    bbox2d.size_y = abs(top_left2d.y - bottom_right2d.y);

    double iou = iouScore(bbox2d, detBBox);
    RCLCPP_INFO(this->get_logger(), "pos: %f, %f, %f, size: %f, %f, %f : iou %f", 
      b.center.position.x, b.center.position.y, b.center.position.z,
      b.size.x, b.size.y, b.size.z, iou);

    if (iou > bestScore)
    {
      bestScore = iou;
      bestBBox = i;
    }
  }
  return bestBBox;
}

double TrackingNode::overlapBoundingBox(const vision_msgs::msg::BoundingBox2D& boxA, const vision_msgs::msg::BoundingBox2D& boxB)
{
  double overlapHeight =
      std::min(boxA.center.position.y + boxA.size_y/2, boxB.center.position.y + boxB.size_y) -
      std::max(boxA.center.position.y, boxB.center.position.y);
  if (overlapHeight <= 0) return 0;
  double overlapWidth = std::min(boxA.center.position.x + boxA.size_x, boxB.center.position.x + boxB.size_x) -
                        std::max(boxA.center.position.x, boxB.center.position.x);
  if (overlapWidth <= 0) return 0;
  return overlapHeight * overlapWidth;
}

double TrackingNode::iouScore(const vision_msgs::msg::BoundingBox2D& bboxA, const vision_msgs::msg::BoundingBox2D& bboxB)
{
  double overlap = overlapBoundingBox(bboxA, bboxB);
  // If overlap is 0, it returns 0
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
