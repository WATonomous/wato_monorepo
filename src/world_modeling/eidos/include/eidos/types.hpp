#pragma once

#include <pcl/kdtree/kdtree_flann.h>  // Must precede PCL_NO_PRECOMPILE

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>

namespace eidos {

// Primary point type used throughout the system
using PointType = pcl::PointXYZI;

// Raw point type with ring and time info (Velodyne format, used as common representation)
struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Ouster raw point type
struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Unified raw point type
using PointXYZIRT = VelodynePointXYZIRT;

// 6-DOF pose point type for keyframe storage
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PoseType = PointXYZIRPYT;

// Sensor type enumeration
enum class SensorType { VELODYNE, OUSTER, LIVOX };

// SLAM state machine
enum class SlamState { INITIALIZING, RELOCALIZING, TRACKING };

// Utility: compute distance of a single point from origin
inline float pointDistance(const PointType& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

// Utility: compute distance between two points
inline float pointDistance(const PointType& p1, const PointType& p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y) +
                   (p1.z - p2.z) * (p1.z - p2.z));
}

// Utility: convert PoseType to gtsam::Pose3
inline gtsam::Pose3 poseTypeToGtsamPose3(const PoseType& p) {
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(static_cast<double>(p.roll),
                         static_cast<double>(p.pitch),
                         static_cast<double>(p.yaw)),
    gtsam::Point3(static_cast<double>(p.x),
                  static_cast<double>(p.y),
                  static_cast<double>(p.z)));
}

// Utility: convert gtsam::Pose3 to PoseType
inline PoseType gtsamPose3ToPoseType(const gtsam::Pose3& pose, double time = 0.0) {
  PoseType p;
  p.x = static_cast<float>(pose.translation().x());
  p.y = static_cast<float>(pose.translation().y());
  p.z = static_cast<float>(pose.translation().z());
  p.roll = static_cast<float>(pose.rotation().roll());
  p.pitch = static_cast<float>(pose.rotation().pitch());
  p.yaw = static_cast<float>(pose.rotation().yaw());
  p.time = time;
  p.intensity = 0;
  return p;
}

// Utility: convert PoseType to Eigen Affine3f
inline Eigen::Affine3f poseTypeToAffine3f(const PoseType& p) {
  return pcl::getTransformation(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
}

// Utility: convert roll/pitch/yaw + xyz to gtsam::Pose3
inline gtsam::Pose3 rpyxyzToGtsamPose3(const float transform[6]) {
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(transform[0], transform[1], transform[2]),
    gtsam::Point3(transform[3], transform[4], transform[5]));
}

// Utility: convert roll/pitch/yaw + xyz to Eigen Affine3f
inline Eigen::Affine3f rpyxyzToAffine3f(const float transform[6]) {
  return pcl::getTransformation(
    transform[3], transform[4], transform[5],
    transform[0], transform[1], transform[2]);
}

// Utility: transform a point cloud by a PoseType
inline pcl::PointCloud<PointType>::Ptr transformPointCloud(
    const pcl::PointCloud<PointType>::Ptr& cloud_in, const PoseType& pose) {
  auto cloud_out = pcl::make_shared<pcl::PointCloud<PointType>>();
  Eigen::Affine3f transform = poseTypeToAffine3f(pose);
  pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
  return cloud_out;
}

// Utility: convert ROS timestamp to seconds
template <typename T>
inline double stamp2Sec(const T& stamp) {
  return rclcpp::Time(stamp).seconds();
}

}  // namespace eidos

// PCL point type registration (must be in global scope)
POINT_CLOUD_REGISTER_POINT_STRUCT(eidos::VelodynePointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
  (uint16_t, ring, ring)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(eidos::OusterPointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
  (uint32_t, t, t)(uint16_t, reflectivity, reflectivity)
  (uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

POINT_CLOUD_REGISTER_POINT_STRUCT(eidos::PointXYZIRPYT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
  (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))
