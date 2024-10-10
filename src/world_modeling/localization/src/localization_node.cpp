#include <memory>

#include "localization_node.hpp"

LocalizationNode::LocalizationNode() : Node("sample"), sample_(world_modeling::hd_map::Sample()), odom_counter_(0), gps_counter_(0)
{
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps", ADVERTISING_FREQ,
      std::bind(&LocalizationNode::gps_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", ADVERTISING_FREQ,
      std::bind(&LocalizationNode::odometry_callback, this, std::placeholders::_1));

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));

  // Set the default noise model for the GPS measurements
  gps_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));

  initial_estimate_ = gtsam::Values();

  enu_ = GeographicLib::LocalCartesian(0, 0, 0); // Default reference point - call setupReference() to change
}

void LocalizationNode::setupReference(const sensor_msgs::msg::NavSatFix::SharedPtr ref_point)
{
  double ref_lat = ref_point->latitude;
  double ref_lon = ref_point->longitude;
  double ref_alt = ref_point->altitude;

  // Set the reference point for the ENU frame
  enu_ = GeographicLib::LocalCartesian(ref_lat, ref_lon, ref_alt);
}

gtsam::Point3 LocalizationNode::convertGPSToENU(const sensor_msgs::msg::NavSatFix::SharedPtr gpsPoint)
{
  double lat = gpsPoint->latitude;
  double lon = gpsPoint->longitude;
  double alt = gpsPoint->altitude;

  // Convert GPS to ENU
  double x, y, z;
  enu_.Forward(lat, lon, alt, x, y, z);

  return gtsam::Point3(x, y, z);
}

void LocalizationNode::optimize()
{
  // Create a Levenberg-Marquardt optimizer
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);

  gtsam::Values result = optimizer.optimize();

  result.print("Final result:\n");
}

void LocalizationNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gtsam::Point3 enu_point = convertGPSToENU(msg);

  RCLCPP_INFO(this->get_logger(), "Recieved GPS data %d.. \n", gps_counter_);
  RCLCPP_INFO(this->get_logger(), "Lat: %f, Long: %f, Alt: %f\n", msg->latitude, msg->longitude,
              msg->altitude);
  RCLCPP_INFO(this->get_logger(), "ENU: x: %f, y: %f, z: %f\n", enu_point.x(), enu_point.y(),
              enu_point.z());

  // Add the GPS measurement to the graph as a factor
  graph_.add(gtsam::GPSFactor(gps_counter_, enu_point, gps_noise_model_));

  gps_counter_++;
}

void LocalizationNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract position (x, y, z) and orientation (roll, pitch, yaw) from the odometry message
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

  RCLCPP_INFO(this->get_logger(), "Received 3D odometry data %d.. \n", odom_counter_);
  RCLCPP_INFO(this->get_logger(), "Odometry pose: x: %f, y: %f, z: %f\n", x, y, z);

  // Create Pose3 from odometry data (translation and rotation)
  gtsam::Pose3 odom_pose(rotation, gtsam::Point3(x, y, z));

  // Add initial pose guess to the graph
  initial_estimate_.insert(odom_counter_, odom_pose);

  odom_counter_++;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}