#ifndef OCCUPANCY_SEGMENTATION_NODE_HPP_
#define OCCUPANCY_SEGMENTATION_NODE_HPP_
#define PCL_NO_PRECOMPILE

#include "rclcpp/rclcpp.hpp"

#include "occupancy_segmentation_core.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;


struct PointXYZIRT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  u_int16_t ring;
  float time;
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16; 

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (u_int16_t, ring, ring)
                                   (float, time, time)
)

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class OccupancySegmentationNode : public rclcpp::Node {
 public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Transformer node constructor.
   */
  OccupancySegmentationNode();

 private:

  // Object that handles data processing and validation.
  OccupancySegmentationCore<PointXYZIRT> _patchwork;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _nonground_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _ground_publisher;

  void subscription_callback(const  sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud);
};

#endif  // TRANSFORMER_NODE_HPP_
